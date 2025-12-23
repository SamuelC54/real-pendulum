#include <AccelStepper.h>

// -------------------- Pin definitions --------------------
constexpr int DIR_PIN    = 2;
constexpr int STEP_PIN   = 9;   // User moved to pin 9
constexpr int ENABLE_PIN = 8;

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

unsigned long lastMoveTime = 0;
bool motorEnabled = true;
bool oscillating = false;
int oscillateDirection = 1;  // 1 = right, -1 = left
long oscillateCenter = 0;

// Speed multiplier - doubles with each 'P' press
int speedLevel = 0;  // 0 = 1x, 1 = 2x, 2 = 4x, etc.
float currentOscillateSpeed = OSCILLATE_BASE_SPEED;
float currentOscillateAccel = OSCILLATE_BASE_ACCEL;

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

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Enable DRV8825 (active LOW)

  stepper.setMaxSpeed(MANUAL_SPEED);
  stepper.setAcceleration(MANUAL_ACCEL);

  Serial.begin(115200);
  // Flush any garbage in serial buffer
  while (Serial.available()) Serial.read();
  
  Serial.println("Commands: L=left, R=right, O=oscillate, P=speed up, S=stop");
  lastMoveTime = millis();
}

void loop() {
  stepper.run();

  // Handle oscillation mode
  if (oscillating && stepper.distanceToGo() == 0) {
    // Reached target, reverse direction
    oscillateDirection *= -1;
    long target = oscillateCenter + (oscillateDirection * OSCILLATE_STEP);
    stepper.moveTo(target);
    lastMoveTime = millis();
  }

  // Disable motor after idle timeout (only when not oscillating)
  if (!oscillating && stepper.distanceToGo() == 0) {
    if (motorEnabled && (millis() - lastMoveTime > IDLE_TIMEOUT_MS)) {
      disableMotor();
    }
  }

  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == 'L') {
      oscillating = false;
      stepper.setMaxSpeed(MANUAL_SPEED);
      stepper.setAcceleration(MANUAL_ACCEL);
      enableMotor();
      long target = stepper.currentPosition() - MANUAL_STEP;
      if (USE_SOFT_LIMITS) target = clampLong(target, MIN_POS, MAX_POS);
      stepper.moveTo(target);
      lastMoveTime = millis();
    }
    else if (c == 'R') {
      oscillating = false;
      stepper.setMaxSpeed(MANUAL_SPEED);
      stepper.setAcceleration(MANUAL_ACCEL);
      enableMotor();
      long target = stepper.currentPosition() + MANUAL_STEP;
      if (USE_SOFT_LIMITS) target = clampLong(target, MIN_POS, MAX_POS);
      stepper.moveTo(target);
      lastMoveTime = millis();
    }
    else if (c == 'O') {
      // Start oscillation (reset speed level)
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
    else if (c == 'P') {
      // Double the speed
      speedLevel++;
      if (speedLevel > 5) speedLevel = 5;  // Cap at 32x to prevent insane speeds
      updateOscillateSpeed();
    }
    else if (c == 'S') {
      oscillating = false;
      speedLevel = 0;
      stepper.setMaxSpeed(MANUAL_SPEED);
      stepper.setAcceleration(MANUAL_ACCEL);
      stepper.stop();
      Serial.println("Stopped");
    }
  }
}
