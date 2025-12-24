#include <AccelStepper.h>
#include <math.h>

// -------------------- Pin definitions --------------------
// Stepper motor
constexpr int DIR_PIN    = 2;
constexpr int STEP_PIN   = 9;   // User moved to pin 9
constexpr int ENABLE_PIN = 8;

// Encoder (KY-040)
constexpr int PIN_CLK = 7;
constexpr int PIN_DT  = 6;

// Limit switches (active LOW with INPUT_PULLUP)
constexpr int LIMIT_LEFT_PIN  = 4;   // Left end of rail
constexpr int LIMIT_RIGHT_PIN = 5;   // Right end of rail

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// -------------------- Motor tuning (configurable via serial) --------------------
// These are default values, can be changed at runtime via web UI
float manualSpeed = 6000;       // steps/sec
float manualAccel = 10000;      // steps/sec^2
long manualStep = 200;          // steps per button press

float oscillateSpeed = 2000;    // oscillation speed
float oscillateAccel = 4000;    // oscillation acceleration
long oscillateStep = 600;       // oscillation amplitude (steps)

float swingupSpeed = 10000;     // swing-up max speed
float swingupAccel = 6000;      // swing-up acceleration
float swingupKp = 20.0f;        // swing-up proportional gain

constexpr float MAX_SPEED = 16000;  // AccelStepper practical limit

// Disable motor after idle for this many ms (saves power, stops pulsing)
constexpr unsigned long IDLE_TIMEOUT_MS = 500;

// Optional software limits
constexpr bool USE_SOFT_LIMITS = false;
constexpr long MIN_POS = 0;
constexpr long MAX_POS = 20000;

// -------------------- Encoder settings --------------------
// KY-040: typically 20 detents → 40 transitions per rev
constexpr int ENCODER_STEPS_PER_REV = 40;

// -------------------- State variables --------------------
unsigned long lastMoveTime = 0;
bool motorEnabled = true;
bool oscillating = false;
int oscillateDirection = 1;  // 1 = right, -1 = left
long oscillateCenter = 0;

// Speed multiplier - doubles with each 'P' press
int speedLevel = 0;  // 0 = 1x, 1 = 2x, 2 = 4x, etc.
float currentOscillateSpeed = oscillateSpeed;
float currentOscillateAccel = oscillateAccel;

// Encoder state
int lastCLK;
long encoderPosition = 0;
float currentAngle = 0.0f;

// Swing-up state
bool swingUpMode = false;
float previousAngle = 0.0f;
unsigned long lastSwingUpTime = 0;
long swingUpCenter = 0;  // Cart position when swing-up started
constexpr unsigned long SWINGUP_INTERVAL_MS = 50;  // Control loop interval (50ms for encoder resolution)
unsigned long lastDebugTime = 0;  // For debug output

// PD Control parameters for swing-up (Kp is configurable, others fixed for now)
constexpr float KD_SWINGUP = 0.0f;                 // Derivative gain
constexpr float KC_SWINGUP = 0.0f;                 // Centering gain
constexpr long MAX_CART_OFFSET = 3000;             // Maximum cart offset from center (steps)

// Serial input buffer for configuration commands
String serialBuffer = "";

// Command queue system
const int QUEUE_SIZE = 16;  // Maximum commands in queue
String commandQueue[QUEUE_SIZE];
int queueHead = 0;  // Next command to execute
int queueTail = 0;  // Where to add new commands
int queueCount = 0; // Current number of commands in queue
bool processingCommand = false;  // True when motor is executing a command

// Angle where pendulum hangs down (adjust if your encoder zero is different)
constexpr float ANGLE_DOWN = 0.0f;    // 0° = hanging down
constexpr float ANGLE_UP = 180.0f;    // 180° = inverted (goal)

// Set to -1 if pendulum stalls or goes wrong direction
constexpr int SWINGUP_DIRECTION = 1;  // 1 or -1 to invert pumping direction

static long clampLong(long v, long lo, long hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Queue management functions
bool queueIsFull() {
  return queueCount >= QUEUE_SIZE;
}

bool queueIsEmpty() {
  return queueCount == 0;
}

void queueClear() {
  queueHead = 0;
  queueTail = 0;
  queueCount = 0;
  processingCommand = false;
  Serial.print("Queue cleared (");
  Serial.print(queueCount);
  Serial.println(" items)");
}

bool queueAdd(String cmd) {
  if (queueIsFull()) {
    Serial.println("Queue full, command rejected");
    return false;
  }
  commandQueue[queueTail] = cmd;
  queueTail = (queueTail + 1) % QUEUE_SIZE;
  queueCount++;
  return true;
}

String queuePeek() {
  if (queueIsEmpty()) return "";
  return commandQueue[queueHead];
}

String queuePop() {
  if (queueIsEmpty()) return "";
  String cmd = commandQueue[queueHead];
  queueHead = (queueHead + 1) % QUEUE_SIZE;
  queueCount--;
  return cmd;
}

void enableMotor() {
  if (!motorEnabled) {
    digitalWrite(ENABLE_PIN, LOW);  // Enable DRV8825 (active LOW)
    motorEnabled = true;
  }
}

void disableMotor() {
  if (motorEnabled) {
    digitalWrite(ENABLE_PIN, HIGH);  // Disable DRV8825
    motorEnabled = false;
  }
}

void updateOscillateSpeed() {
  float multiplier = (float)(1 << speedLevel);  // 2^speedLevel
  currentOscillateSpeed = oscillateSpeed * multiplier;
  currentOscillateAccel = oscillateAccel * multiplier;
  
  // Cap at practical AccelStepper limit
  if (currentOscillateSpeed > MAX_SPEED) currentOscillateSpeed = MAX_SPEED;
  if (currentOscillateAccel > MAX_SPEED * 2) currentOscillateAccel = MAX_SPEED * 2;
  
  if (oscillating) {
    stepper.setMaxSpeed(currentOscillateSpeed);
    stepper.setAcceleration(currentOscillateAccel);
    // Re-issue move command to apply new speed immediately
    long target = oscillateCenter + (oscillateDirection * oscillateStep);
    stepper.moveTo(target);
  }
  
  Serial.print("Speed level: ");
  Serial.print(speedLevel);
  Serial.print(" (");
  Serial.print((int)multiplier);
  Serial.print("x) = ");
  Serial.print((int)currentOscillateSpeed);
  Serial.println(" steps/sec");
}

// Read encoder and update angle
void readEncoder() {
  int currentCLK = digitalRead(PIN_CLK);
  
  if (currentCLK != lastCLK) {
    if (digitalRead(PIN_DT) != currentCLK) {
      encoderPosition++;   // clockwise
    } else {
      encoderPosition--;   // counter-clockwise
    }
    
    // Convert position to angle and wrap to [0, 360)
    currentAngle = fmod((encoderPosition * 360.0f) / ENCODER_STEPS_PER_REV, 360.0f);
    if (currentAngle < 0) currentAngle += 360.0f;
  }
  
  lastCLK = currentCLK;
}

// Get current angle (call readEncoder() first to update)
float getAngle() {
  return currentAngle;
}

// Check limit switches - only stops if moving TOWARDS the limit
bool checkLimits() {
  bool leftHit = (digitalRead(LIMIT_LEFT_PIN) == LOW);
  bool rightHit = (digitalRead(LIMIT_RIGHT_PIN) == LOW);
  
  long distanceToGo = stepper.distanceToGo();
  bool movingLeft = (distanceToGo < 0);
  bool movingRight = (distanceToGo > 0);
  
  // Only stop if moving towards the triggered limit
  if ((leftHit && movingLeft) || (rightHit && movingRight)) {
    // Stop immediately and clear everything
    stepper.stop();
    stepper.setCurrentPosition(stepper.currentPosition());  // Clear target
    oscillating = false;
    swingUpMode = false;
    processingCommand = false;
    
    // Clear the entire command queue
    queueClear();
    
    if (leftHit) {
      Serial.println("LIMIT: Left switch hit! Queue cleared.");
    }
    if (rightHit) {
      Serial.println("LIMIT: Right switch hit! Queue cleared.");
    }
    return true;
  }
  return false;
}

// Check if movement in a direction is allowed
bool canMoveLeft() {
  return (digitalRead(LIMIT_LEFT_PIN) == HIGH);
}

bool canMoveRight() {
  return (digitalRead(LIMIT_RIGHT_PIN) == HIGH);
}

void setup() {
  // Stepper setup
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Enable DRV8825 (active LOW)

  stepper.setMaxSpeed(manualSpeed);
  stepper.setAcceleration(manualAccel);

  // Encoder setup
  pinMode(PIN_CLK, INPUT_PULLUP);
  pinMode(PIN_DT, INPUT_PULLUP);
  lastCLK = digitalRead(PIN_CLK);

  // Limit switch setup
  pinMode(LIMIT_LEFT_PIN, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  // Flush any garbage in serial buffer
  while (Serial.available()) Serial.read();
  
  Serial.println("Commands: L=left, R=right, O=oscillate, W=swing-up, P=speed up, A=angle, S=stop");
  lastMoveTime = millis();
}

void loop() {
  stepper.run();
  
  // Always read encoder
  readEncoder();
  
  // Check limit switches
  checkLimits();

  // Check if current command is finished (motor stopped)
  if (processingCommand && stepper.distanceToGo() == 0 && !oscillating && !swingUpMode) {
    processingCommand = false;  // Ready for next command
  }
  
  // Process next command from queue if motor is ready
  if (!processingCommand && !oscillating && !swingUpMode && !queueIsEmpty()) {
    String nextCmd = queuePop();
    if (nextCmd.length() > 0) {
      executeCommand(nextCmd);
    }
  }
  
  // Handle oscillation mode
  if (oscillating && stepper.distanceToGo() == 0) {
    // Reached target, reverse direction
    oscillateDirection *= -1;
    long target = oscillateCenter + (oscillateDirection * oscillateStep);
    stepper.moveTo(target);
    lastMoveTime = millis();
  }

  // Handle swing-up mode using PD control (smooth, proportional response)
  if (swingUpMode && (millis() - lastSwingUpTime >= SWINGUP_INTERVAL_MS)) {
    // Calculate angular velocity (degrees per interval)
    float angularVelocity = currentAngle - previousAngle;
    
    // Handle wrap-around (e.g., 359° -> 1° should be +2°, not -358°)
    if (angularVelocity > 180.0f) angularVelocity -= 360.0f;
    if (angularVelocity < -180.0f) angularVelocity += 360.0f;
    
    // Scale velocity to degrees per second (for consistent tuning)
    float dt = SWINGUP_INTERVAL_MS / 1000.0f;
    float angularVelocityPerSec = angularVelocity / dt;
    
    // Energy-based swing-up: pump signal = cos(angle) * angular_velocity
    // cos(angle) is positive near bottom (0°), negative near top (180°)
    // This pumps energy when pendulum swings through bottom
    float angleRad = currentAngle * (3.14159f / 180.0f);
    float cosAngle = cos(angleRad);
    float pumpSignal = cosAngle * angularVelocityPerSec * SWINGUP_DIRECTION;
    
    // Current cart state
    long currentPos = stepper.currentPosition();
    long drift = currentPos - swingUpCenter;
    
    // PD Control:
    // P (proportional): Cart offset proportional to pump signal
    // D (derivative): Damping based on angular acceleration (smooths response)
    // C (centering): Pull cart back toward starting position
    
    float pTerm = swingupKp * pumpSignal;                    // Proportional to pump signal
    float dTerm = -KD_SWINGUP * angularVelocityPerSec;        // Damping (opposes fast swings)
    float cTerm = -KC_SWINGUP * (float)drift;                 // Centering force
    
    // Calculate desired cart offset from center
    float desiredOffset = pTerm + dTerm + cTerm;
    
    // Clamp to maximum allowed offset
    if (desiredOffset > MAX_CART_OFFSET) desiredOffset = MAX_CART_OFFSET;
    if (desiredOffset < -MAX_CART_OFFSET) desiredOffset = -MAX_CART_OFFSET;
    
    // Calculate target position
    long target = swingUpCenter + (long)desiredOffset;
    
    // Respect limit switches
    if (target > currentPos && !canMoveRight()) {
      target = currentPos;
    } else if (target < currentPos && !canMoveLeft()) {
      target = currentPos;
    }
    
    stepper.moveTo(target);
    lastMoveTime = millis();
    
    // Debug output every 500ms
    if (millis() - lastDebugTime > 500) {
      Serial.print("ang=");
      Serial.print(currentAngle, 1);
      Serial.print(" vel=");
      Serial.print(angularVelocityPerSec, 1);
      Serial.print(" pump=");
      Serial.print(pumpSignal, 1);
      Serial.print(" offset=");
      Serial.print((long)desiredOffset);
      Serial.print(" target=");
      Serial.println(target);
      lastDebugTime = millis();
    }
    
    previousAngle = currentAngle;
    lastSwingUpTime = millis();
  }

  // Disable motor after idle timeout (only when not oscillating or swing-up)
  if (!oscillating && !swingUpMode && stepper.distanceToGo() == 0) {
    if (motorEnabled && (millis() - lastMoveTime > IDLE_TIMEOUT_MS)) {
      disableMotor();
    }
  }

  // Read serial commands
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    
    // If newline, process buffer
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processCommand(serialBuffer);
        serialBuffer = "";
      }
    }
    // Single character commands (immediate)
    else if (serialBuffer.length() == 0 && 
             (c == 'L' || c == 'R' || c == 'O' || c == 'W' || c == 'P' || c == 'A' || c == 'S' || c == '?')) {
      processCommand(String(c));
    }
    // Multi-character command (buffer until newline)
    else {
      serialBuffer += c;
      // Safety: limit buffer size
      if (serialBuffer.length() > 20) {
        serialBuffer = "";
      }
    }
  }
}

// Process incoming command - immediate commands execute now, others get queued
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  
  char c = cmd.charAt(0);
  
  // Immediate commands - always execute right away
  // A = get angle, S = stop, ? = settings, config commands (contain ':')
  if (c == 'A' || c == 'S' || c == '?' || cmd.indexOf(':') > 0) {
    executeCommand(cmd);
    return;
  }
  
  // Movement commands get queued
  if (c == 'L' || c == 'R' || c == 'O' || c == 'W' || c == 'P') {
    if (queueAdd(cmd)) {
      Serial.print("Queued: ");
      Serial.print(cmd);
      Serial.print(" (queue: ");
      Serial.print(queueCount);
      Serial.println(")");
    }
    return;
  }
  
  // Unknown commands - try to execute immediately
  executeCommand(cmd);
}

// Execute a command directly (called from queue or for immediate commands)
void executeCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  
  char c = cmd.charAt(0);
  
  // Single character commands
  if (cmd.length() == 1) {
    if (c == 'L') {
      if (!canMoveLeft()) {
        Serial.println("Blocked: left limit");
      } else {
        oscillating = false;
        swingUpMode = false;
        processingCommand = true;  // Mark as processing
        stepper.setMaxSpeed(manualSpeed);
        stepper.setAcceleration(manualAccel);
        enableMotor();
        long target = stepper.currentPosition() - manualStep;
        if (USE_SOFT_LIMITS) target = clampLong(target, MIN_POS, MAX_POS);
        stepper.moveTo(target);
        lastMoveTime = millis();
      }
    }
    else if (c == 'R') {
      if (!canMoveRight()) {
        Serial.println("Blocked: right limit");
      } else {
        oscillating = false;
        swingUpMode = false;
        processingCommand = true;  // Mark as processing
        stepper.setMaxSpeed(manualSpeed);
        stepper.setAcceleration(manualAccel);
        enableMotor();
        long target = stepper.currentPosition() + manualStep;
        if (USE_SOFT_LIMITS) target = clampLong(target, MIN_POS, MAX_POS);
        stepper.moveTo(target);
        lastMoveTime = millis();
      }
    }
    else if (c == 'O') {
      if (!canMoveLeft() || !canMoveRight()) {
        Serial.println("Cannot oscillate: at limit switch");
      } else {
        swingUpMode = false;
        processingCommand = false;
        queueClear();  // Clear queue - oscillate takes over
        speedLevel = 0;
        updateOscillateSpeed();
        enableMotor();
        oscillating = true;
        oscillateCenter = stepper.currentPosition();
        oscillateDirection = 1;
        stepper.setMaxSpeed(currentOscillateSpeed);
        stepper.setAcceleration(currentOscillateAccel);
        stepper.moveTo(oscillateCenter + oscillateStep);
        lastMoveTime = millis();
        Serial.println("Oscillating... (queue cleared)");
      }
    }
    else if (c == 'W') {
      oscillating = false;
      processingCommand = false;
      queueClear();  // Clear queue - swing-up takes over
      swingUpMode = true;
      swingUpCenter = stepper.currentPosition();
      previousAngle = currentAngle;
      lastSwingUpTime = millis();
      enableMotor();
      stepper.setMaxSpeed(swingupSpeed);
      stepper.setAcceleration(swingupAccel);
      Serial.println("Swing-up mode... (queue cleared)");
    }
    else if (c == 'P') {
      speedLevel++;
      if (speedLevel > 5) speedLevel = 5;
      updateOscillateSpeed();
    }
    else if (c == 'A') {
      // Send angle, position, limit switch status, and queue size
      Serial.print("ang=");
      Serial.print(currentAngle, 1);
      Serial.print(" pos=");
      Serial.print(stepper.currentPosition());
      Serial.print(" limL=");
      Serial.print(digitalRead(LIMIT_LEFT_PIN) == LOW ? 1 : 0);
      Serial.print(" limR=");
      Serial.print(digitalRead(LIMIT_RIGHT_PIN) == LOW ? 1 : 0);
      Serial.print(" queue=");
      Serial.println(queueCount);
    }
    else if (c == 'S') {
      // Stop everything and clear queue
      oscillating = false;
      swingUpMode = false;
      speedLevel = 0;
      processingCommand = false;
      queueClear();  // Clear all pending commands
      stepper.setMaxSpeed(manualSpeed);
      stepper.setAcceleration(manualAccel);
      stepper.stop();
      Serial.println("Stopped, queue cleared");
    }
    else if (c == '?') {
      // Print current settings
      printSettings();
    }
  }
  // Configuration commands: XX:value
  else if (cmd.length() > 3 && cmd.charAt(2) == ':') {
    String prefix = cmd.substring(0, 2);
    float value = cmd.substring(3).toFloat();
    
    if (prefix == "MS") {
      manualSpeed = constrain(value, 100, MAX_SPEED);
      Serial.print("Manual Speed: "); Serial.println(manualSpeed);
    }
    else if (prefix == "MA") {
      manualAccel = constrain(value, 100, 20000);
      Serial.print("Manual Accel: "); Serial.println(manualAccel);
    }
    else if (prefix == "MT") {
      manualStep = constrain((long)value, 10, 5000);
      Serial.print("Manual Step: "); Serial.println(manualStep);
    }
    else if (prefix == "OS") {
      oscillateSpeed = constrain(value, 100, MAX_SPEED);
      currentOscillateSpeed = oscillateSpeed;
      Serial.print("Oscillate Speed: "); Serial.println(oscillateSpeed);
    }
    else if (prefix == "OA") {
      oscillateAccel = constrain(value, 100, 20000);
      currentOscillateAccel = oscillateAccel;
      Serial.print("Oscillate Accel: "); Serial.println(oscillateAccel);
    }
    else if (prefix == "OT") {
      oscillateStep = constrain((long)value, 50, 5000);
      Serial.print("Oscillate Step: "); Serial.println(oscillateStep);
    }
    else if (prefix == "WS") {
      swingupSpeed = constrain(value, 100, MAX_SPEED);
      Serial.print("Swingup Speed: "); Serial.println(swingupSpeed);
    }
    else if (prefix == "WA") {
      swingupAccel = constrain(value, 100, 20000);
      Serial.print("Swingup Accel: "); Serial.println(swingupAccel);
    }
    else if (prefix == "WK") {
      swingupKp = constrain(value, 1, 200);
      Serial.print("Swingup Kp: "); Serial.println(swingupKp);
    }
    else {
      Serial.print("Unknown config: "); Serial.println(prefix);
    }
  }
}

// Print all current settings
void printSettings() {
  Serial.println("=== Current Settings ===");
  Serial.print("Manual:    speed="); Serial.print(manualSpeed);
  Serial.print(" accel="); Serial.print(manualAccel);
  Serial.print(" step="); Serial.println(manualStep);
  Serial.print("Oscillate: speed="); Serial.print(oscillateSpeed);
  Serial.print(" accel="); Serial.print(oscillateAccel);
  Serial.print(" step="); Serial.println(oscillateStep);
  Serial.print("Swingup:   speed="); Serial.print(swingupSpeed);
  Serial.print(" accel="); Serial.print(swingupAccel);
  Serial.print(" Kp="); Serial.println(swingupKp);
}
