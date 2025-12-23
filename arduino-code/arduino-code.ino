#include <AccelStepper.h>

// -------------------- Pin definitions --------------------
constexpr int DIR_PIN    = 2;
constexpr int STEP_PIN   = 3;
constexpr int ENABLE_PIN = 8;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// -------------------- Manual tuning --------------------
// For NEMA 23 + DRV8825 at 1/8 microstepping (1600 steps/rev)
// Install jumpers on M0 and M1 (M2 open)
constexpr float MANUAL_SPEED = 4000;   // steps/sec (~150 RPM)
constexpr float MANUAL_ACCEL = 8000;   // steps/sec^2

// Each key press moves this many steps
constexpr long MANUAL_STEP = 800;      // 1/2 revolution at 1/8 microstepping

// Optional software limits (disable by setting USE_SOFT_LIMITS=false)
constexpr bool USE_SOFT_LIMITS = false;
constexpr long MIN_POS = 0;
constexpr long MAX_POS = 20000;

static long clampLong(long v, long lo, long hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable DRV8825

  stepper.setEnablePin(ENABLE_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();

  stepper.setMaxSpeed(MANUAL_SPEED);
  stepper.setAcceleration(MANUAL_ACCEL);

  Serial.begin(115200);
  Serial.println("Manual mode: send 'L' or 'R' (and 'S' to stop).");
}

void loop() {
  stepper.run();

  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == 'L') {
      long target = stepper.currentPosition() - MANUAL_STEP;
      if (USE_SOFT_LIMITS) target = clampLong(target, MIN_POS, MAX_POS);
      stepper.moveTo(target);
    }
    else if (c == 'R') {
      long target = stepper.currentPosition() + MANUAL_STEP;
      if (USE_SOFT_LIMITS) target = clampLong(target, MIN_POS, MAX_POS);
      stepper.moveTo(target);
    }
    else if (c == 'S') {
      stepper.stop(); // decelerates to stop
    }
  }
}
