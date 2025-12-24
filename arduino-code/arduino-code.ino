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

// -------------------- Manual tuning --------------------
// For NEMA 23 + DRV8825 at FULL STEP (200 steps/rev)
// NO JUMPERS on M0, M1, M2 — maximum speed!
constexpr float MANUAL_SPEED = 6000;     // steps/sec (~225 RPM)
constexpr float MANUAL_ACCEL = 10000;    // steps/sec^2

// Each key press moves this many steps
constexpr long MANUAL_STEP = 200;        // 1 full revolution at full step

// Oscillation base settings
constexpr float OSCILLATE_BASE_SPEED = 2000;   // start slower so doubling works
constexpr float OSCILLATE_BASE_ACCEL = 4000;
constexpr long OSCILLATE_STEP = 600;           // 3 revolutions each way at full step
constexpr float MAX_SPEED = 10000;             // AccelStepper practical limit on Uno

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
float currentOscillateSpeed = OSCILLATE_BASE_SPEED;
float currentOscillateAccel = OSCILLATE_BASE_ACCEL;

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

// PD Control parameters for swing-up
constexpr float SWINGUP_SPEED = 10000;             // Max cart speed
constexpr float SWINGUP_ACCEL = 6000;              // Cart acceleration
constexpr float KP_SWINGUP = 20.0f;                // Proportional gain (cart steps per pump signal unit)
constexpr float KD_SWINGUP = 0.0f;                 // Derivative gain (start with 0, add later if needed)
constexpr float KC_SWINGUP = 0.0f;                 // Centering gain (start with 0)
constexpr long MAX_CART_OFFSET = 3000;             // Maximum cart offset from center (steps)

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
  currentOscillateSpeed = OSCILLATE_BASE_SPEED * multiplier;
  currentOscillateAccel = OSCILLATE_BASE_ACCEL * multiplier;
  
  // Cap at practical AccelStepper limit
  if (currentOscillateSpeed > MAX_SPEED) currentOscillateSpeed = MAX_SPEED;
  if (currentOscillateAccel > MAX_SPEED * 2) currentOscillateAccel = MAX_SPEED * 2;
  
  if (oscillating) {
    stepper.setMaxSpeed(currentOscillateSpeed);
    stepper.setAcceleration(currentOscillateAccel);
    // Re-issue move command to apply new speed immediately
    long target = oscillateCenter + (oscillateDirection * OSCILLATE_STEP);
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
    // Stop immediately
    stepper.stop();
    stepper.setCurrentPosition(stepper.currentPosition());  // Clear target
    oscillating = false;
    swingUpMode = false;
    
    if (leftHit) {
      Serial.println("LIMIT: Left switch hit!");
    }
    if (rightHit) {
      Serial.println("LIMIT: Right switch hit!");
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

  stepper.setMaxSpeed(MANUAL_SPEED);
  stepper.setAcceleration(MANUAL_ACCEL);

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

  // Handle oscillation mode
  if (oscillating && stepper.distanceToGo() == 0) {
    // Reached target, reverse direction
    oscillateDirection *= -1;
    long target = oscillateCenter + (oscillateDirection * OSCILLATE_STEP);
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
    
    float pTerm = KP_SWINGUP * pumpSignal;                    // Proportional to pump signal
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

  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == 'L') {
      if (!canMoveLeft()) {
        Serial.println("Blocked: left limit");
      } else {
        oscillating = false;
        swingUpMode = false;
        stepper.setMaxSpeed(MANUAL_SPEED);
        stepper.setAcceleration(MANUAL_ACCEL);
        enableMotor();
        long target = stepper.currentPosition() - MANUAL_STEP;
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
        stepper.setMaxSpeed(MANUAL_SPEED);
        stepper.setAcceleration(MANUAL_ACCEL);
        enableMotor();
        long target = stepper.currentPosition() + MANUAL_STEP;
        if (USE_SOFT_LIMITS) target = clampLong(target, MIN_POS, MAX_POS);
        stepper.moveTo(target);
        lastMoveTime = millis();
      }
    }
    else if (c == 'O') {
      // Check if we can oscillate (not at a limit)
      if (!canMoveLeft() || !canMoveRight()) {
        Serial.println("Cannot oscillate: at limit switch");
      } else {
        // Start oscillation (reset speed level)
        swingUpMode = false;
        speedLevel = 0;
        updateOscillateSpeed();
        enableMotor();
        oscillating = true;
        oscillateCenter = stepper.currentPosition();
        oscillateDirection = 1;
        stepper.setMaxSpeed(currentOscillateSpeed);
        stepper.setAcceleration(currentOscillateAccel);
        stepper.moveTo(oscillateCenter + OSCILLATE_STEP);
        lastMoveTime = millis();
        Serial.println("Oscillating...");
      }
    }
    else if (c == 'W') {
      // Start swing-up mode
      oscillating = false;
      swingUpMode = true;
      swingUpCenter = stepper.currentPosition();  // Remember center position
      previousAngle = currentAngle;
      lastSwingUpTime = millis();
      enableMotor();
      stepper.setMaxSpeed(SWINGUP_SPEED);
      stepper.setAcceleration(SWINGUP_ACCEL);
      Serial.println("Swing-up mode: pumping energy...");
    }
    else if (c == 'P') {
      // Double the speed
      speedLevel++;
      if (speedLevel > 5) speedLevel = 5;  // Cap at 32x to prevent insane speeds
      updateOscillateSpeed();
    }
    else if (c == 'A') {
      // Print current angle
      Serial.print("Angle: ");
      Serial.print(currentAngle, 2);
      Serial.println(" deg");
    }
    else if (c == 'S') {
      oscillating = false;
      swingUpMode = false;
      speedLevel = 0;
      stepper.setMaxSpeed(MANUAL_SPEED);
      stepper.setAcceleration(MANUAL_ACCEL);
      stepper.stop();
      Serial.println("Stopped");
    }
  }
}
