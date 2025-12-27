/*
 * Inverted Pendulum - Velocity Streaming Arduino Firmware
 * 
 * This is a SIMPLIFIED Arduino that acts as a sensor/actuator interface.
 * All control logic lives in the Python controller.
 * 
 * RECEIVES: Velocity commands "V1234" or "V-1234" (steps/second)
 * SENDS:    Sensor data "A:180.5,P:1000,LL:0,LR:1\n" at 50Hz
 */

#include <AccelStepper.h>

// ============ PIN CONFIGURATION ============
// Encoder
constexpr int ENCODER_CLK = 2;    // Must be interrupt pin
constexpr int ENCODER_DT  = 8;

// Stepper (DM542 driver)
constexpr int STEP_PIN = 9;
constexpr int DIR_PIN  = 10;

// Limit switches (active LOW with INPUT_PULLUP)
constexpr int LIMIT_LEFT_PIN  = 4;
constexpr int LIMIT_RIGHT_PIN = 5;

// ============ MOTOR CONFIGURATION ============
constexpr long MAX_SPEED = 50000;       // Max steps/second for DM542
constexpr long ACCELERATION = 100000;   // Steps/second² - high for responsive velocity changes

// ============ ENCODER CONFIGURATION ============
constexpr int ENCODER_PPR = 600;        // Pulses per revolution
constexpr float DEGREES_PER_PULSE = 360.0f / (ENCODER_PPR * 4);  // Quadrature = 4x

// ============ COMMUNICATION ============
constexpr unsigned long BAUD_RATE = 115200;
constexpr unsigned long DATA_INTERVAL_MS = 20;  // 50Hz sensor output

// ============ GLOBAL STATE ============
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Encoder state
volatile long encoderCount = 0;
volatile int lastEncoded = 0;

// Timing
unsigned long lastDataSend = 0;

// Current commanded velocity
long targetVelocity = 0;

// ============ ENCODER ISR ============
void encoderISR() {
  int MSB = digitalRead(ENCODER_CLK);
  int LSB = digitalRead(ENCODER_DT);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderCount--;
  }
  lastEncoded = encoded;
}

// ============ SETUP ============
void setup() {
  Serial.begin(BAUD_RATE);
  
  // Encoder pins
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);
  
  // Initialize encoder state
  int MSB = digitalRead(ENCODER_CLK);
  int LSB = digitalRead(ENCODER_DT);
  lastEncoded = (MSB << 1) | LSB;
  
  // Limit switch pins
  pinMode(LIMIT_LEFT_PIN, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT_PIN, INPUT_PULLUP);
  
  // Stepper configuration for velocity mode
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  
  Serial.println("VELOCITY_MODE_READY");
}

// ============ READ SERIAL COMMANDS ============
void processSerial() {
  static char buffer[32];
  static int bufIndex = 0;
  
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (bufIndex > 0) {
        buffer[bufIndex] = '\0';
        parseCommand(buffer);
        bufIndex = 0;
      }
    } else if (bufIndex < 31) {
      buffer[bufIndex++] = c;
    }
  }
}

void parseCommand(const char* cmd) {
  // Velocity command: V1234 or V-1234
  if (cmd[0] == 'V') {
    targetVelocity = atol(cmd + 1);
    // Clamp to max speed
    if (targetVelocity > MAX_SPEED) targetVelocity = MAX_SPEED;
    if (targetVelocity < -MAX_SPEED) targetVelocity = -MAX_SPEED;
  }
  // Zero encoder: Z
  else if (cmd[0] == 'Z') {
    noInterrupts();
    encoderCount = 0;
    interrupts();
    stepper.setCurrentPosition(0);
    Serial.println("ZEROED");
  }
  // Stop: S
  else if (cmd[0] == 'S') {
    targetVelocity = 0;
  }
}

// ============ CHECK LIMIT SWITCHES ============
bool checkLimits() {
  bool leftHit = (digitalRead(LIMIT_LEFT_PIN) == LOW);
  bool rightHit = (digitalRead(LIMIT_RIGHT_PIN) == LOW);
  
  // If moving towards a triggered limit, stop
  // Note: positive velocity = physical left, negative = physical right (inverted)
  if ((leftHit && targetVelocity > 0) || (rightHit && targetVelocity < 0)) {
    targetVelocity = 0;
    stepper.setSpeed(0);
    return true;
  }
  return false;
}

// ============ SEND SENSOR DATA ============
void sendSensorData() {
  // Read encoder with interrupts disabled for consistency
  noInterrupts();
  long count = encoderCount;
  interrupts();
  
  // Calculate angle (0-360, with 180 = hanging down)
  float angle = fmod(count * DEGREES_PER_PULSE, 360.0f);
  if (angle < 0) angle += 360.0f;
  
  // Read limit switches
  bool leftLimit = (digitalRead(LIMIT_LEFT_PIN) == LOW);
  bool rightLimit = (digitalRead(LIMIT_RIGHT_PIN) == LOW);
  
  // Get current position
  long position = stepper.currentPosition();
  
  // Send compact data format
  // A:angle,P:position,LL:left,LR:right,V:velocity
  Serial.print("A:");
  Serial.print(angle, 1);
  Serial.print(",P:");
  Serial.print(position);
  Serial.print(",LL:");
  Serial.print(leftLimit ? 1 : 0);
  Serial.print(",LR:");
  Serial.print(rightLimit ? 1 : 0);
  Serial.print(",V:");
  Serial.println(stepper.speed());
}

// ============ MAIN LOOP ============
void loop() {
  // Process incoming commands
  processSerial();
  
  // Check limit switches
  checkLimits();
  
  // Update motor velocity
  stepper.setSpeed(targetVelocity);
  stepper.runSpeed();
  
  // Send sensor data at fixed interval
  unsigned long now = millis();
  if (now - lastDataSend >= DATA_INTERVAL_MS) {
    lastDataSend = now;
    sendSensorData();
  }
}

