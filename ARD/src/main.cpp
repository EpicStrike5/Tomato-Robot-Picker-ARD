#include <Arduino.h>
#include <AccelStepper.h>  // For controlling stepper motors
#include <Servo.h>         // For controlling servo motors
#include "CommandParser.h" // Your custom command parsing library
#include "DCMotor.h"       // Your DC motor control library

// --- Pin Definitions ---
// DC Motors (Base Movement)
const int L_MOTOR_PWM_PIN = 9;
const int L_MOTOR_DIR_PIN = 22;
const int R_MOTOR_PWM_PIN = 8;
const int R_MOTOR_DIR_PIN = 23;

// Stepper Motor (Elevation - Cascade Lift)
const int ELEV_STEP_PIN = 35;
const int ELEV_DIR_PIN = 39;
const int ELEV_ENA_PIN = 49;

// Stepper Motor (Extension - Arm)
const int EXT_STEP_PIN = 28;
const int EXT_DIR_PIN = 30;
const int EXT_ENA_PIN = 51;

// Servo Motors (Tool - Gripper & Cutter)
const int GRIPPER_SERVO_PIN = 13;
const int CUTTER_SERVO_PIN = 12;

// --- Motor Objects ---
DCMotor leftMotor(L_MOTOR_PWM_PIN, L_MOTOR_DIR_PIN);
DCMotor rightMotor(R_MOTOR_PWM_PIN, R_MOTOR_DIR_PIN);

AccelStepper elevStepper(AccelStepper::DRIVER, ELEV_STEP_PIN, ELEV_DIR_PIN);
AccelStepper extStepper(AccelStepper::DRIVER, EXT_STEP_PIN, EXT_DIR_PIN);

Servo gripperServo;
Servo cutterServo;

// --- Robot Physical Constants (Base Movement: METERS and SECONDS) ---
const float WHEEL_DIA_M = 0.0765f;
const float WHEEL_RAD_M = WHEEL_DIA_M / 2.0f;
const float TRACK_WIDTH_M = 0.300f;

// ROBOT_REFERENCE_MAX_SPEED_MPS: This is the theoretical maximum speed of a wheel in meters per second.
// Derived from:
// 1. Motor Rated RPM (after internal gearbox): 19.2 RPM
// 2. External Spur Gear Reduction: Motor Gear (30 teeth) / Wheel Gear (50 teeth) = 0.6
// 3. Final Wheel RPM = Motor RPM * Spur Gear Reduction = 19.2 RPM * 0.6 = 11.52 RPM
// 4. Wheel Diameter = 76.5 mm = 0.0765 m
// 5. Wheel Circumference = PI * Wheel Diameter = PI * 0.0765 m approx 0.24033 m
// 6. Final Wheel RPS (Revolutions Per Second) = Final Wheel RPM / 60 = 11.52 / 60 = 0.192 RPS
// 7. Max Speed (m/s) = Final Wheel RPS * Wheel Circumference = 0.192 RPS * 0.24033 m/rev approx 0.046143 m/s
const float ROBOT_REFERENCE_MAX_SPEED_MPS = 0.04614f;

// --- Motor Speed Adjustment Factors (Tune These!) ---
const float LEFT_MOTOR_SPEED_ADJUST_FACTOR = 0.95f;
const float RIGHT_MOTOR_SPEED_ADJUST_FACTOR = 1.0f;

// --- Stepper Configuration for ELEVATION Mechanism (Units: MILLIMETERS, SECONDS, STEPS) ---
const int ELEV_STEPS_PER_REV = 200;
const float ELEV_PULLEY_DIA_MM = 40.0f;
const float ELEV_PULLEY_CIRC_MM = PI * ELEV_PULLEY_DIA_MM;
const int ELEV_STAGES = 3;
const float ELEV_STEPS_PER_MM_PULLEY = (float)ELEV_STEPS_PER_REV / ELEV_PULLEY_CIRC_MM;
const float ELEV_STEPS_PER_MM_ACTUAL = ELEV_STEPS_PER_MM_PULLEY / ELEV_STAGES;

const int ELEV_MAX_REVS = 20;
const long ELEV_MIN_STEPS = 0;
const long ELEV_MAX_STEPS = ((long)ELEV_MAX_REVS * ELEV_STEPS_PER_REV) + 300;

const float ELEV_MAX_SPEED_SPS = 200.0f;
const float ELEV_ACCEL_SPS2 = 50.0f;

// --- Stepper Configuration for EXTENSION Arm (Units: MILLIMETERS, SECONDS, STEPS) ---
const int EXT_STEPS_PER_REV = 1600;
const float EXT_LEAD_MM = 3.0f;
const float EXT_STEPS_PER_MM = (float)EXT_STEPS_PER_REV / EXT_LEAD_MM;

const float EXT_MAX_SPEED_SPS = 2000.0f;
const float EXT_ACCEL_SPS2 = 1000.0f;
// Define EXT_MIN_STEPS and EXT_MAX_STEPS if you have known limits for the arm
// const long EXT_MIN_STEPS = 0;
// const long EXT_MAX_STEPS = 5000; // Example: 5000 steps max extension

// --- Servo Configuration ---
const int SERVO_MIN_ANGLE = 0;
const int TOOL_SERVO_MAX_OPERATING_ANGLE = 58; // Max angle for gripper and cutter based on user input

// --- Command Parser ---
CommandParser commandParser;

// Define stepper driver enable states
const int STEPPER_DRIVER_ENABLED = HIGH;
const int STEPPER_DRIVER_DISABLED = LOW;

// --- Helper Function Declaration ---
void resetStepperDrivers();

// --- Arduino Setup Function ---
void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 2000)
    ;
  Serial.println(F("\n--- Tomato Harvesting Robot Initializing ---"));
  // Serial.print(F("Firmware Version: ")); Serial.println(F("3.2_Reduced_Serial"));
  // Serial.print(F("Robot Reference Max Speed (Hardcoded): ")); Serial.print(ROBOT_REFERENCE_MAX_SPEED_MPS, 5); Serial.println(F(" m/s"));
  // Serial.print(F("Left Motor Adjust Factor: ")); Serial.println(LEFT_MOTOR_SPEED_ADJUST_FACTOR, 2);
  // Serial.print(F("Right Motor Adjust Factor: ")); Serial.println(RIGHT_MOTOR_SPEED_ADJUST_FACTOR, 2);

  // Setup DC Motors
  leftMotor.setup();
  rightMotor.setup();
  leftMotor.configureMaxSpeed(ROBOT_REFERENCE_MAX_SPEED_MPS * LEFT_MOTOR_SPEED_ADJUST_FACTOR);
  rightMotor.configureMaxSpeed(ROBOT_REFERENCE_MAX_SPEED_MPS * RIGHT_MOTOR_SPEED_ADJUST_FACTOR);
  // Serial.println(F("DC Motors Initialized."));

  // Setup Stepper Enable Pins
  pinMode(ELEV_ENA_PIN, OUTPUT);
  pinMode(EXT_ENA_PIN, OUTPUT);
  digitalWrite(ELEV_ENA_PIN, STEPPER_DRIVER_ENABLED); // Enable drivers by default
  digitalWrite(EXT_ENA_PIN, STEPPER_DRIVER_ENABLED);
  // Serial.println(F("Stepper Enable Pins Initialized (HIGH = ENABLED)."));

  // Setup Elevation Stepper Motor
  elevStepper.setMaxSpeed(ELEV_MAX_SPEED_SPS);
  elevStepper.setAcceleration(ELEV_ACCEL_SPS2);
  elevStepper.setCurrentPosition(ELEV_MIN_STEPS);
  // elevStepper.setPinsInverted(true, false, false);

  // Setup Extension Stepper Motor
  extStepper.setMaxSpeed(EXT_MAX_SPEED_SPS);
  extStepper.setAcceleration(EXT_ACCEL_SPS2);
  extStepper.setCurrentPosition(0);
  extStepper.setPinsInverted(true, false, false);
  // Serial.println(F("Elevation and Extension Stepper Motors Initialized."));

  // Setup Servo Motors
  gripperServo.attach(GRIPPER_SERVO_PIN);
  cutterServo.attach(CUTTER_SERVO_PIN);
  // Serial.println(F("Servo Motors Initialized."));
  // Set servos to initial MIN_ANGLE position
  gripperServo.write(SERVO_MIN_ANGLE);
  cutterServo.write(SERVO_MIN_ANGLE);
  // Serial.print(F("Gripper and Cutter Servos set to MIN_ANGLE ("));
  // Serial.print(SERVO_MIN_ANGLE); Serial.println(F(" degrees)."));

  Serial.println(F("Initialization Complete. Waiting for commands..."));
  // Serial.println(F("Cmds: MoveTO M V S | ElevatorTO H | ArmTO D | ToolOpen GA CA | GripperClose GA | CutterClose CA | ResetSteppers | STOP"));
  // Serial.println(F("  MoveTO M V S: M=L(linear)/R(rotate), V=dist(mm)/angle(deg), S=speed(m/s or rad/s)"));
}

// --- Arduino Loop Function ---
void loop()
{
  // 1. Process Incoming Serial Commands
  if (commandParser.readSerial())
  {
    if (commandParser.isNewCommandAvailable())
    {
      String cmdName = commandParser.getCommandName();
      int numArgs = commandParser.getNumArgs();

      Serial.print(F("CMD RX: "));
      Serial.print(cmdName);
      // Serial.print(F(" (Args: ")); Serial.print(numArgs); Serial.println(F(")")); // Optional: for debugging arg count

      if (cmdName.equalsIgnoreCase("MoveTO") && numArgs == 3)
      {
        String mode = commandParser.getArgAsString(0);
        float value = commandParser.getArgAsFloat(1);
        float speed = commandParser.getArgAsFloat(2);

        if (speed <= 0)
        {
          Serial.println(F("Error: MoveTO speed must be positive."));
        }
        else
        {
          if (mode.equalsIgnoreCase("L"))
          {                                     // Linear Movement
            float distance_m = value / 1000.0f; // Convert mm to m
            float linear_speed_mps = speed;
            unsigned long duration_ms = 0;
            if (abs(linear_speed_mps) > 0.001f)
            { // Avoid division by zero
              duration_ms = (unsigned long)((abs(distance_m) / linear_speed_mps) * 1000.0f);
            }

            // Serial.print(F("  MoveTO Linear: Dist=")); Serial.print(distance_m * 1000.0f, 0);
            // Serial.print(F("mm, Speed=")); Serial.print(linear_speed_mps, 3);
            // Serial.print(F("m/s, Duration=")); Serial.print(duration_ms); Serial.println(F("ms"));

            if (duration_ms > 0)
            {
              if (distance_m > 0)
              { // Forward
                Serial.println(F("    Moving Forward"));
                leftMotor.setSpeedMPS(-linear_speed_mps);
                rightMotor.setSpeedMPS(linear_speed_mps);
              }
              else
              { // Backward
                Serial.println(F("    Moving Backward"));
                leftMotor.setSpeedMPS(linear_speed_mps);
                rightMotor.setSpeedMPS(-linear_speed_mps);
              }
              delay(duration_ms);
              leftMotor.setSpeedMPS(0);
              rightMotor.setSpeedMPS(0);
              // Serial.println(F("    Linear move complete."));
            }
            else
            {
              // Serial.println(F("    Zero duration, no linear move."));
            }
          }
          else if (mode.equalsIgnoreCase("R"))
          { // Rotational Movement
            float angle_deg = value;
            float angle_rad = radians(angle_deg);
            float angular_speed_rps = speed; // User provides rad/s
            unsigned long duration_ms = 0;
            if (abs(angular_speed_rps) > 0.001f)
            { // Avoid division by zero
              duration_ms = (unsigned long)((abs(angle_rad) / angular_speed_rps) * 1000.0f);
            }

            float wheel_tangential_speed_mps = angular_speed_rps * (TRACK_WIDTH_M / 2.0f);

            // Serial.print(F("  MoveTO Rotate: Angle=")); Serial.print(angle_deg, 1);
            // Serial.print(F("deg, Speed=")); Serial.print(angular_speed_rps, 3);
            // Serial.print(F("rad/s, Duration=")); Serial.print(duration_ms); Serial.println(F("ms"));
            // Serial.print(F("    Wheel tangential speed: "));Serial.print(wheel_tangential_speed_mps,3);Serial.println(F(" m/s"));

            if (duration_ms > 0)
            {
              if (angle_rad > 0)
              { // Spin Left (Robot CCW) - Both motors physical CW
                Serial.println(F("    Spinning Left"));
                leftMotor.setSpeedMPS(wheel_tangential_speed_mps);
                rightMotor.setSpeedMPS(wheel_tangential_speed_mps);
              }
              else
              { // Spin Right (Robot CW) - Both motors physical CCW
                Serial.println(F("    Spinning Right"));
                leftMotor.setSpeedMPS(-wheel_tangential_speed_mps);
                rightMotor.setSpeedMPS(-wheel_tangential_speed_mps);
              }
              delay(duration_ms);
              leftMotor.setSpeedMPS(0);
              rightMotor.setSpeedMPS(0);
              // Serial.println(F("    Rotation complete."));
            }
            else
            {
              // Serial.println(F("    Zero duration, no rotation."));
            }
          }
          else
          {
            Serial.println(F("Error: Invalid MoveTO mode. Use 'L' or 'R'."));
          }
        }
      }
      else if (cmdName.equalsIgnoreCase("ElevatorTO") && numArgs >= 1)
      {
        float targetHeightMM = commandParser.getArgAsFloat(0);
        long targetSteps = (long)(targetHeightMM * ELEV_STEPS_PER_MM_ACTUAL);
        targetSteps = constrain(targetSteps, ELEV_MIN_STEPS, ELEV_MAX_STEPS);
        // Serial.print(F("  ElevatorTO: ")); Serial.print(targetHeightMM);
        // Serial.print(F("mm (Target Steps: ")); Serial.print(targetSteps); Serial.println(F(")"));
        elevStepper.setMaxSpeed(ELEV_MAX_SPEED_SPS);
        elevStepper.setAcceleration(ELEV_ACCEL_SPS2);
        elevStepper.moveTo(targetSteps);
      }
      else if (cmdName.equalsIgnoreCase("ArmTO") && numArgs >= 1)
      {
        float targetDistMM = commandParser.getArgAsFloat(0);
        long targetSteps = (long)(targetDistMM * EXT_STEPS_PER_MM);
        // Serial.print(F("  ArmTO: ")); Serial.print(targetDistMM);
        // Serial.print(F("mm (Target Steps: ")); Serial.print(targetSteps); Serial.println(F(")"));
        extStepper.setMaxSpeed(EXT_MAX_SPEED_SPS);
        extStepper.setAcceleration(EXT_ACCEL_SPS2);
        extStepper.moveTo(targetSteps);
      }
      else if (cmdName.equalsIgnoreCase("ToolOpen") && numArgs == 2)
      {
        int gripperAngle = commandParser.getArgAsLong(0);
        int cutterAngle = commandParser.getArgAsLong(1);
        gripperAngle = constrain(gripperAngle, SERVO_MIN_ANGLE, TOOL_SERVO_MAX_OPERATING_ANGLE);
        cutterAngle = constrain(cutterAngle, SERVO_MIN_ANGLE, TOOL_SERVO_MAX_OPERATING_ANGLE);
        // Serial.print(F("  ToolOpen: Gripper Angle=")); Serial.print(gripperAngle);
        // Serial.print(F(", Cutter Angle=")); Serial.println(cutterAngle);
        gripperServo.write(gripperAngle);
        cutterServo.write(cutterAngle);
      }
      else if (cmdName.equalsIgnoreCase("GripperClose") && numArgs == 1)
      {
        int gripperAngle = commandParser.getArgAsLong(0);
        gripperAngle = constrain(gripperAngle, SERVO_MIN_ANGLE, TOOL_SERVO_MAX_OPERATING_ANGLE);
        // Serial.print(F("  GripperClose: Angle=")); Serial.println(gripperAngle);
        gripperServo.write(gripperAngle);
      }
      else if (cmdName.equalsIgnoreCase("CutterClose") && numArgs == 1)
      {
        int cutterAngle = commandParser.getArgAsLong(0);
        cutterAngle = constrain(cutterAngle, SERVO_MIN_ANGLE, TOOL_SERVO_MAX_OPERATING_ANGLE);
        // Serial.print(F("  CutterClose: Angle=")); Serial.println(cutterAngle);
        cutterServo.write(cutterAngle);
      }
      else if (cmdName.equalsIgnoreCase("ResetSteppers"))
      {
        Serial.println(F("  ResetSteppers Command RX.")); // Keep this for feedback
        resetStepperDrivers();
      }
      else if (cmdName.equalsIgnoreCase("STOP"))
      {
        Serial.println(F("  STOP Command RX. Halting all motion.")); // Keep this
        leftMotor.setSpeedMPS(0.0f);
        rightMotor.setSpeedMPS(0.0f);

        elevStepper.stop();
        elevStepper.setCurrentPosition(elevStepper.currentPosition());
        extStepper.stop();
        extStepper.setCurrentPosition(extStepper.currentPosition());

        gripperServo.write(SERVO_MIN_ANGLE);
        cutterServo.write(SERVO_MIN_ANGLE);

        // Serial.println(F("  All motors halted. Robot Idle."));
      }
      else
      {
        Serial.print(F("Error: Unknown command '"));
        Serial.print(cmdName);
        Serial.println(F("' or wrong arguments.")); // Keep error messages
      }
      commandParser.clearCommand();
    }
  }

  if (digitalRead(ELEV_ENA_PIN) == STEPPER_DRIVER_ENABLED)
  {
    elevStepper.run();
  }
  if (digitalRead(EXT_ENA_PIN) == STEPPER_DRIVER_ENABLED)
  {
    extStepper.run();
  }
}

void resetStepperDrivers()
{
  Serial.println(F("Attempting to reset stepper drivers...")); // Keep

  digitalWrite(ELEV_ENA_PIN, STEPPER_DRIVER_DISABLED);
  digitalWrite(EXT_ENA_PIN, STEPPER_DRIVER_DISABLED);
  // Serial.println(F("  Elevation & Extension Steppers DISabled."));

  delay(100);

  digitalWrite(ELEV_ENA_PIN, STEPPER_DRIVER_ENABLED);
  digitalWrite(EXT_ENA_PIN, STEPPER_DRIVER_ENABLED);
  Serial.println(F("  Steppers RE-enabled.")); // Keep
}
