#include <AccelStepper.h>

// -------------------- Pin definitions --------------------
constexpr int DIR_PIN    = 2;
constexpr int STEP_PIN   = 9;   // User moved to pin 9
constexpr int ENABLE_PIN = 8;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// -------------------- Manual tuning --------------------
// For NEMA 23 + DRV8825 at 1/8 microstepping (1600 steps/rev)
// Install jumpers on M0 and M1 (M2 open)
constexpr float MANUAL_SPEED = 6000;     // steps/sec (~225 RPM)
constexpr float MANUAL_ACCEL = 10000;    // steps/sec^2

// Each key press moves this many steps
constexpr long MANUAL_STEP = 1600;       // 1 full revolution at 1/8 microstepping

// Oscillation settings
constexpr float OSCILLATE_SPEED = 16000;  // faster for oscillation
constexpr float OSCILLATE_ACCEL = 30000;
constexpr long OSCILLATE_STEP = 4800;     // 3 revolutions each way

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

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Enable DRV8825 (active LOW)

  stepper.setMaxSpeed(MANUAL_SPEED);
  stepper.setAcceleration(MANUAL_ACCEL);

  Serial.begin(115200);
  // Flush any garbage in serial buffer
  while (Serial.available()) Serial.read();
  
  Serial.println("Commands: L=left, R=right, O=oscillate, S=stop");
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
      // Start oscillation
      enableMotor();
      oscillating = true;
      oscillateCenter = stepper.currentPosition();
      oscillateDirection = 1;
      stepper.setMaxSpeed(OSCILLATE_SPEED);
      stepper.setAcceleration(OSCILLATE_ACCEL);
      stepper.moveTo(oscillateCenter + OSCILLATE_STEP);
      lastMoveTime = millis();
      Serial.println("Oscillating...");
    }
    else if (c == 'S') {
      oscillating = false;
      stepper.setMaxSpeed(MANUAL_SPEED);
      stepper.setAcceleration(MANUAL_ACCEL);
      stepper.stop();
      Serial.println("Stopped");
    }
  }
}
