#include <AccelStepper.h>

// -------------------- Pin definitions --------------------
constexpr int DIR_PIN    = 2;
constexpr int STEP_PIN   = 3;
constexpr int ENABLE_PIN = 8;

constexpr int SWITCH_HOME_PIN = 5; // Home switch (switch 1)
constexpr int SWITCH_END_PIN  = 6; // End switch  (switch 2)

// -------------------- Stepper setup --------------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// DRV8825 microstepping (1,2,4,8,16,32)
constexpr int MICROSTEPS = 16;

// NEMA23: 1.8° = 200 full steps per revolution
constexpr long FULL_STEPS_PER_REV = 200;
constexpr long STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS;

// -------------------- Speeds --------------------
constexpr float HOMING_SPEED = 150;     // slow speed for homing
constexpr float HOMING_ACCEL = 100;

constexpr float NORMAL_SPEED = 400;
constexpr float NORMAL_ACCEL = 200;

// After centering: fast small back-and-forth
constexpr float JIGGLE_SPEED = 2000;    // fast
constexpr float JIGGLE_ACCEL = 4000;    // snappy accel
constexpr long  JIGGLE_AMPLITUDE_STEPS = 400; // adjust: +/- steps around center

// -------------------- State machine --------------------
enum HomingState {
  MOVE_TO_HOME,
  MOVE_TO_END,
  MOVE_TO_CENTER,
  JIGGLE
};

HomingState state = MOVE_TO_HOME;

// Stores total travel distance between switches
long totalTravelSteps = 0;
long centerPosition = 0;

// Jiggle direction toggle
bool jiggleToPositive = true;

void setup() {
  // Switches use INPUT_PULLUP (pressed = LOW)
  pinMode(SWITCH_HOME_PIN, INPUT_PULLUP);
  pinMode(SWITCH_END_PIN, INPUT_PULLUP);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable DRV8825

  stepper.setEnablePin(ENABLE_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();

  // Start with homing speed
  stepper.setMaxSpeed(HOMING_SPEED);
  stepper.setAcceleration(HOMING_ACCEL);

  // Move toward the home switch (negative direction assumed)
  stepper.moveTo(-1000000); // large number to guarantee reaching the switch
}

void loop() {
  stepper.run();

  switch (state) {

    // -------------------- Step 1: Find home switch --------------------
    case MOVE_TO_HOME:
      if (digitalRead(SWITCH_HOME_PIN) == LOW) {
        stepper.stop();
        stepper.setCurrentPosition(0); // Home position = 0

        // Prepare to move toward end switch
        stepper.setMaxSpeed(HOMING_SPEED);
        stepper.setAcceleration(HOMING_ACCEL);
        stepper.moveTo(1000000); // large positive move

        state = MOVE_TO_END;
      }
      break;

    // -------------------- Step 2: Find end switch --------------------
    case MOVE_TO_END:
      if (digitalRead(SWITCH_END_PIN) == LOW) {
        stepper.stop();

        // Save total travel distance (position when end switch hit)
        totalTravelSteps = stepper.currentPosition();

        // Compute and move to center
        centerPosition = totalTravelSteps / 2;

        stepper.setMaxSpeed(NORMAL_SPEED);
        stepper.setAcceleration(NORMAL_ACCEL);
        stepper.moveTo(centerPosition);

        state = MOVE_TO_CENTER;
      }
      break;

    // -------------------- Step 3: Go to center --------------------
    case MOVE_TO_CENTER:
      if (stepper.distanceToGo() == 0) {
        // Switch to jiggle motion
        stepper.setMaxSpeed(JIGGLE_SPEED);
        stepper.setAcceleration(JIGGLE_ACCEL);

        // Start by going slightly negative (or positive, your choice)
        jiggleToPositive = true;
        stepper.moveTo(centerPosition + JIGGLE_AMPLITUDE_STEPS);

        state = JIGGLE;
      }
      break;

    // -------------------- Step 4: Fast back and forth around center --------------------
    case JIGGLE:
      // When we reached the target, flip to the other side
      if (stepper.distanceToGo() == 0) {
        jiggleToPositive = !jiggleToPositive;

        long target = jiggleToPositive
          ? (centerPosition + JIGGLE_AMPLITUDE_STEPS)
          : (centerPosition - JIGGLE_AMPLITUDE_STEPS);

        stepper.moveTo(target);
      }
      break;
  }
}
