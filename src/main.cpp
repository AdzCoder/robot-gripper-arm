#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <movingAvg.h>
#include <Servo.h>

/**
 * @file main.cpp
 * @brief Sensor-controlled DC motor gripper with closed-loop grip force
 * @author Adil Wahab Bhatti
 * @version 4.1
 * @date 2026-09-03
 *
 * @description
 * Arduino gripper control system. An ultrasonic sensor decides when to close and
 * motor current is regulated to a setpoint to decide how hard, giving a grip that
 * holds without crushing. Pad force is sampled and filtered for logging but does
 * not feed the control loop. A joystick rotates the gripper and arms the system.
 *
 * @hardware
 * - Arduino Uno
 * - INA219 Current Sensor
 * - HC-SR04 Ultrasonic Sensor
 * - FSR Force Sensor
 * - Servo Motor
 * - DC Motor with Driver
 * - KY-023 Joystick Module
 *
 * @dependencies
 * - Adafruit_INA219 >= 1.2.3
 * - movingAvg >= 2.3.1
 * - Servo >= 1.2.1
 */

// ============================================================================
// CONFIGURATION & CONSTANTS
// ============================================================================

const bool DEBUG_ENABLED = true;

// Pin Definitions
namespace Pins
{
  const int SERVO = 11;
  const int MOTOR_PWM = 3;
  const int MOTOR_DIR = 8;
  const int FORCE_SENSOR = A0;
  const int ULTRASONIC_TRIG = 6;
  const int ULTRASONIC_ECHO = 5;
  const int JOYSTICK_X = A2;
  const int JOYSTICK_Y = A1;
  const int JOYSTICK_BUTTON = 4;
}

// System Limits & Thresholds
namespace Limits
{
  const int SERVO_MIN_ANGLE = 0;
  const int SERVO_MAX_ANGLE = 180;
  const int SERVO_START_ANGLE = 90;
  const int MOTOR_SPEED = 70;
  const int PWM_MIN = 0;
  const int PWM_MAX = 255;
  const float GRIP_DISTANCE_THRESHOLD = 12.0; // cm
  const float JOYSTICK_DEADZONE = 0.07;
  const float JOYSTICK_GRIP_THRESHOLD = 0.3;
  const float LOOSENING_TIMEOUT = 9000.0; // ms
}

// PID Controller Parameters
//
// All three gains are tunable. KP is the live one; KI and KD are currently zero:
//
//   - calculatePID() accumulates its result into the output, so the KP term
//     already supplies integral action. A KI term adds double-integral
//     behaviour on top of it.
//   - Current is smoothed by a 35-sample moving average and lags the true motor
//     current. Derivative action on a heavily filtered signal amplifies that lag
//     instead of damping the response.
//
// The gains carry units (PWM counts per mA of error, per iteration) and only
// mean anything against the real motor, gearbox and pad.
namespace PID
{
  const double KP = 0.02;     // tunable
  const double KI = 0.00;     // tunable
  const double KD = 0.000000; // tunable
  const float SETPOINT = 130.0; // mA
}

// Moving Average Buffer Sizes
namespace BufferSizes
{
  const int CURRENT = 35;
  const int FORCE = 20;
  const int DISTANCE = 100;
}

// ============================================================================
// TYPES & ENUMS
// ============================================================================

// TIGHTENING covers the whole closing-and-holding phase: the controller regulates
// continuously while an object is present and does not latch a separate held state.
enum class GripperMode
{
  OFF = 0,
  TIGHTENING = 1,
  LOOSENING = -1,
  LOOSENED = -2
};

struct SensorData
{
  float current = 0.0;
  float avgCurrent = 0.0;
  float distance = 0.0;
  float avgDistance = 0.0;
  int force = 0;
  int avgForce = 0;
  float joystickX = 0.0;
  float joystickY = 0.0;
  bool buttonPressed = false;
};

struct SystemState
{
  bool running = false;
  GripperMode gripperMode = GripperMode::OFF;
  float servoAngle = Limits::SERVO_START_ANGLE;
  double pidOutput = 0.0;
  double error = 0.0;
  double prevError = 0.0;
  double integral = 0.0;
  unsigned long currentTime = 0;
  unsigned long prevTime = 0;
  float deltaTimeMs = 0.0; // milliseconds; KI and KD are scaled in these units
  float looseningTime = 0.0; // milliseconds
};

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Adafruit_INA219 currentSensor;
Servo servoMotor;
movingAvg currentFilter(BufferSizes::CURRENT);
movingAvg forceFilter(BufferSizes::FORCE);
movingAvg distanceFilter(BufferSizes::DISTANCE);

SensorData sensors;
SystemState state;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

float readUltrasonicDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Timeout caps the blocking wait: the HC-SR04's ~4 m ceiling is a ~23 ms
  // round trip, so 25 ms bounds the loop instead of pulseIn's 1 s default.
  long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0)
  {
    return 0.0; // No echo within range; treated as "no reading" by the caller.
  }
  return (duration * 0.034) / 2.0;
}

float getJoystickRatio(int pin)
{
  float rawValue = analogRead(pin);
  return (rawValue * 2.0 / 1023.0) - 1.0;
}

void updateSensors()
{
  sensors.current = currentSensor.getCurrent_mA();
  if (sensors.current != 0)
  {
    sensors.avgCurrent = currentFilter.reading(sensors.current);
  }

  sensors.distance = readUltrasonicDistance(Pins::ULTRASONIC_TRIG, Pins::ULTRASONIC_ECHO);
  if (sensors.distance != 0)
  {
    sensors.avgDistance = distanceFilter.reading(sensors.distance);
  }

  sensors.force = analogRead(Pins::FORCE_SENSOR);
  sensors.avgForce = forceFilter.reading(sensors.force);

  sensors.joystickX = getJoystickRatio(Pins::JOYSTICK_X);
  sensors.joystickY = getJoystickRatio(Pins::JOYSTICK_Y);
  sensors.buttonPressed = (digitalRead(Pins::JOYSTICK_BUTTON) == LOW);
}

void updateTiming()
{
  state.prevTime = state.currentTime;
  state.currentTime = micros();
  state.deltaTimeMs = (state.currentTime - state.prevTime) / 1000.0;
}

/**
 * Incremental (velocity-form) controller: the result is added to the previous
 * output rather than written to it, which shifts the order of action of every
 * term by one.
 *
 *   KP term -> integral action    (accumulating a proportional term integrates)
 *   KI term -> double-integral action
 *   KD term -> proportional action
 *
 * With KI and KD at zero the loop runs as pure integral action with gain KP,
 * driving steady-state current error to zero and holding the grip. A true
 * velocity-form PI would take the proportional part as KP * (error - prevError).
 *
 * constrain() clamps the accumulator itself, bounding integral wind-up in place
 * rather than only limiting what reaches the motor.
 */
void calculatePID()
{
  state.prevError = state.error;
  state.error = PID::SETPOINT - sensors.avgCurrent;

  double proportional = PID::KP * state.error;
  state.integral += PID::KI * state.error * state.deltaTimeMs;

  // KD is applied before the division, so a zero interval evaluates 0.0 / 0.0 as
  // NaN rather than zero, and NaN survives constrain() to reach analogWrite().
  double derivative = 0.0;
  if (state.deltaTimeMs > 0.0)
  {
    derivative = PID::KD * (state.error - state.prevError) / state.deltaTimeMs;
  }

  double pidValue = proportional + state.integral + derivative;
  state.pidOutput += pidValue;
  state.pidOutput = constrain(state.pidOutput, Limits::PWM_MIN, Limits::PWM_MAX);
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void stopGripper()
{
  analogWrite(Pins::MOTOR_PWM, 0);
}

void startTightening()
{
  stopGripper();
  delay(25);

  state.gripperMode = GripperMode::TIGHTENING;
  digitalWrite(Pins::MOTOR_DIR, HIGH);

  analogWrite(Pins::MOTOR_PWM, Limits::MOTOR_SPEED * 0.5);
  delay(25);
  analogWrite(Pins::MOTOR_PWM, Limits::MOTOR_SPEED);
  delay(25);

  state.integral = 0;
  state.pidOutput = Limits::MOTOR_SPEED;
}

void startLoosening()
{
  stopGripper();
  delay(25);

  state.gripperMode = GripperMode::LOOSENING;
  digitalWrite(Pins::MOTOR_DIR, LOW);

  analogWrite(Pins::MOTOR_PWM, Limits::MOTOR_SPEED * 0.5);
  delay(25);
  analogWrite(Pins::MOTOR_PWM, Limits::MOTOR_SPEED);

  state.looseningTime = 0;
}

void controlGripper()
{
  // Triggers on the instantaneous reading; the 100-sample distance average adds
  // roughly a second of lag to a grip trigger and is kept for logging only. The
  // > 0 guard is required because readUltrasonicDistance() returns 0.0 on echo
  // timeout, which is below the threshold and would otherwise clamp the gripper
  // shut whenever the ultrasonic sensor drops out.
  bool objectInRange = (sensors.distance > 0.0) &&
                       (sensors.distance < Limits::GRIP_DISTANCE_THRESHOLD) &&
                       (abs(sensors.joystickX) < Limits::JOYSTICK_GRIP_THRESHOLD);

  if (objectInRange)
  {
    if (state.gripperMode != GripperMode::TIGHTENING)
    {
      startTightening();
    }
    else
    {
      calculatePID();
      analogWrite(Pins::MOTOR_PWM, state.pidOutput);
      delay(10);
    }
  }
  else if (state.gripperMode != GripperMode::LOOSENED)
  {
    if (state.gripperMode != GripperMode::LOOSENING)
    {
      startLoosening();
    }
    else
    {
      state.looseningTime += state.deltaTimeMs;
      if (state.looseningTime > Limits::LOOSENING_TIMEOUT)
      {
        state.gripperMode = GripperMode::LOOSENED;
        stopGripper();
        state.servoAngle = Limits::SERVO_START_ANGLE;
        servoMotor.write(state.servoAngle);
      }
    }
  }
}

void controlServo()
{
  if (abs(sensors.joystickY) > Limits::JOYSTICK_DEADZONE)
  {
    state.servoAngle += (sensors.joystickY * 0.6);
    state.servoAngle = constrain(state.servoAngle, Limits::SERVO_MIN_ANGLE, Limits::SERVO_MAX_ANGLE);
    servoMotor.write(state.servoAngle);
  }
}

void handleSystemToggle()
{
  static bool prevButtonState = false;

  if (sensors.buttonPressed && !prevButtonState)
  {
    state.running = !state.running;

    while (digitalRead(Pins::JOYSTICK_BUTTON) == LOW)
    {
      delay(10);
    }
    delay(50);
  }

  prevButtonState = sensors.buttonPressed;

  if (!state.running && state.gripperMode != GripperMode::OFF)
  {
    state.gripperMode = GripperMode::OFF;
    stopGripper();
  }
}

// ============================================================================
// DEBUG
// ============================================================================

void printDebugInfo()
{
  if (!DEBUG_ENABLED)
    return;

  Serial.print("Current: ");
  Serial.print(sensors.avgCurrent);
  Serial.print(" | Setpoint: ");
  Serial.print(PID::SETPOINT);
  Serial.print(" | Output: ");
  Serial.print(state.pidOutput);
  Serial.print(" | Distance: ");
  Serial.print(sensors.avgDistance);
  Serial.print(" | Running: ");
  Serial.print(state.running ? "YES" : "NO");
  Serial.print(" | Mode: ");
  Serial.println(static_cast<int>(state.gripperMode));
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup()
{
  if (DEBUG_ENABLED)
  {
    Serial.begin(9600);
    while (!Serial)
      delay(1);
    Serial.println("=== Gripper Control System Initializing ===");
  }

  if (!currentSensor.begin())
  {
    if (DEBUG_ENABLED)
      Serial.println("ERROR: INA219 initialization failed!");
  }
  else if (DEBUG_ENABLED)
  {
    Serial.println("✓ Current sensor initialized");
  }
  currentFilter.begin();

  if (!servoMotor.attach(Pins::SERVO))
  {
    if (DEBUG_ENABLED)
      Serial.println("ERROR: Servo initialization failed!");
  }
  else if (DEBUG_ENABLED)
  {
    Serial.println("✓ Servo motor initialized");
  }

  servoMotor.write(state.servoAngle);
  while (servoMotor.read() != state.servoAngle)
    delay(10);

  pinMode(Pins::MOTOR_PWM, OUTPUT);
  pinMode(Pins::MOTOR_DIR, OUTPUT);
  pinMode(Pins::FORCE_SENSOR, INPUT);
  forceFilter.begin();
  pinMode(Pins::ULTRASONIC_TRIG, OUTPUT);
  pinMode(Pins::ULTRASONIC_ECHO, INPUT);
  distanceFilter.begin();
  pinMode(Pins::JOYSTICK_X, INPUT);
  pinMode(Pins::JOYSTICK_Y, INPUT);
  pinMode(Pins::JOYSTICK_BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (DEBUG_ENABLED)
    Serial.println("=== System Ready ===");
}

void loop()
{
  updateSensors();
  updateTiming();
  handleSystemToggle();

  if (state.running)
  {
    controlGripper();
    controlServo();
  }

  printDebugInfo();
  delay(10);
}
