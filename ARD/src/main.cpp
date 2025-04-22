#include <AccelStepper.h>
#include "CommandParser.h"
#include "DCMotor.h"

// --- Pin Definitions ---
// DC Motors (Base)
const int LEFT_MOTOR_PWM_PIN = 4;
const int LEFT_MOTOR_DIR_PIN = 22;
const int RIGHT_MOTOR_PWM_PIN = 5;
const int RIGHT_MOTOR_DIR_PIN = 23;

// Stepper Motor (Elevation)
const int ELEVATION_STEP_PIN = 6;
const int ELEVATION_DIR_PIN = 24;

// Stepper Motor (Extension/Arm)
const int EXTENSION_STEP_PIN = 7;
const int EXTENSION_DIR_PIN = 25;

// --- Motor Setup ---
DCMotor leftMotor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN);
DCMotor rightMotor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN);

// Interface type for AccelStepper: 1 = Driver (Step/Dir)
AccelStepper elevationStepper(AccelStepper::DRIVER, ELEVATION_STEP_PIN, ELEVATION_DIR_PIN);
AccelStepper extensionStepper(AccelStepper::DRIVER, EXTENSION_STEP_PIN, EXTENSION_DIR_PIN);

// --- Motor Constants ---
// DC Motor Configuration
const float WHEEL_DIAMETER_MM = 75.5; // Using average (74+77)/2
const float WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2.0;
const float TRACK_WIDTH_MM = 300.0;            // Distance between wheel centers
const float MAX_WHEEL_SPEED_MM_PER_SEC = 45.5; // Max speed of the robot (mm/s

// Stepper Configuration
const float STEPS_PER_REVOLUTION = 1600; // 200 * 8 microsteps

// Elevation Stepper (Telescopic Lift)
const float ELEVATION_PULLEY_DIAMETER_MM = 40.0;
const float ELEVATION_PULLEY_CIRCUMFERENCE_MM = PI * ELEVATION_PULLEY_DIAMETER_MM;
const int ELEVATION_STAGES = 3;
// Steps per mm of *linear* pulley movement
const float STEPS_PER_MM_LINEAR_ELEVATION = STEPS_PER_REVOLUTION / ELEVATION_PULLEY_CIRCUMFERENCE_MM;
// Steps per mm of actual *turret* movement (accounts for stages)
const float STEPS_PER_MM_ELEVATION_TURRET = STEPS_PER_MM_LINEAR_ELEVATION / ELEVATION_STAGES;
const float ELEVATION_MAX_SPEED_STEPS_PER_SEC = 1000;    // Tune this! Steps/sec
const float ELEVATION_ACCELERATION_STEPS_PER_SEC2 = 500; // Tune this! Steps/sec^2

// Extension Stepper (Lead Screw)
const float EXTENSION_LEAD_MM = 3.0; // mm per revolution
const float STEPS_PER_MM_EXTENSION = STEPS_PER_REVOLUTION / EXTENSION_LEAD_MM;
const float EXTENSION_MAX_SPEED_STEPS_PER_SEC = 2000;     // Tune this! Steps/sec
const float EXTENSION_ACCELERATION_STEPS_PER_SEC2 = 1000; // Tune this! Steps/sec^2

// --- Command Parser ---
CommandParser commandParser;

void setup()
{
  Serial.begin(115200); // Use a faster baud rate
  while (!Serial)
    ; // Wait for serial connection (especially for native USB like Leonardo/Micro, but good practice)
  Serial.println("Arduino Initialized. Waiting for commands...");

  // Setup DC Motors
  leftMotor.setup();
  rightMotor.setup();
  leftMotor.setSpeed(0);  // Start stopped
  rightMotor.setSpeed(0); // Start stopped
  // Print calculated values for kinematics
  Serial.println("Kinematics enabled for MoveTO (Velocity Mode)");
  Serial.print("Wheel Radius (mm): ");
  Serial.println(WHEEL_RADIUS_MM);
  Serial.print("Track Width (mm): ");
  Serial.println(TRACK_WIDTH_MM);
  Serial.print("Assumed Max Wheel Speed (mm/s): ");
  Serial.println(MAX_WHEEL_SPEED_MM_PER_SEC);

  // Setup Stepper Motors
  elevationStepper.setMaxSpeed(ELEVATION_MAX_SPEED_STEPS_PER_SEC);
  elevationStepper.setAcceleration(ELEVATION_ACCELERATION_STEPS_PER_SEC2);
  elevationStepper.setCurrentPosition(0); // Assume starting at zero position

  extensionStepper.setMaxSpeed(EXTENSION_MAX_SPEED_STEPS_PER_SEC);
  extensionStepper.setAcceleration(EXTENSION_ACCELERATION_STEPS_PER_SEC2);
  extensionStepper.setCurrentPosition(0); // Assume starting at zero position
}

void loop()
{
  // 1. Process Incoming Commands
  if (commandParser.readSerial())
  { // Checks serial and parses if newline received
    if (commandParser.isNewCommandAvailable())
    {
      String cmd = commandParser.getCommandName();
      int numArgs = commandParser.getNumArgs();

      Serial.print("Received command: ");
      Serial.print(cmd);
      Serial.print(" with ");
      Serial.print(numArgs);
      Serial.println(" arguments.");

      // --- Command Handling ---
      if (cmd.equalsIgnoreCase("MoveTO") && numArgs == 2)
      {
        // Interpret X as linear velocity (mm/s), Y as angular velocity (deg/s)
        float linear_velocity_mps = commandParser.getArgAsFloat(0);  // mm/s
        float angular_velocity_dps = commandParser.getArgAsFloat(1); // deg/s

        Serial.print("MoveTO Velocity: Linear=");
        Serial.print(linear_velocity_mps);
        Serial.print(" mm/s, Angular=");
        Serial.print(angular_velocity_dps);
        Serial.println(" deg/s");

        // Convert angular velocity to radians per second
        float angular_velocity_rps = angular_velocity_dps * PI / 180.0; // rad/s

        // Calculate target linear velocities for left and right wheels (mm/s)
        // Inverse Kinematics equations for differential drive:
        // v_right = v + (omega * L / 2)
        // v_left  = v - (omega * L / 2)
        float v_right_target_mps = linear_velocity_mps + (angular_velocity_rps * TRACK_WIDTH_MM / 2.0);
        float v_left_target_mps = linear_velocity_mps - (angular_velocity_rps * TRACK_WIDTH_MM / 2.0);

        // Map target wheel velocities (mm/s) to PWM values (-255 to 255)
        // Assumes linear relationship, scaled by MAX_WHEEL_SPEED_MM_PER_SEC
        int pwm_right = (int)((v_right_target_mps / MAX_WHEEL_SPEED_MM_PER_SEC) * 255.0);
        int pwm_left = (int)((v_left_target_mps / MAX_WHEEL_SPEED_MM_PER_SEC) * 255.0);

        // Clamp PWM values to the valid range [-255, 255]
        pwm_right = constrain(pwm_right, -255, 255);
        pwm_left = constrain(pwm_left, -255, 255);

        Serial.print("  Target Wheel Speeds (mm/s): L=");
        Serial.print(v_left_target_mps);
        Serial.print(", R=");
        Serial.println(v_right_target_mps);
        Serial.print("  Calculated PWM: L=");
        Serial.print(pwm_left);
        Serial.print(", R=");
        Serial.println(pwm_right);

        // Send PWM commands to motors
        leftMotor.setSpeed(pwm_left);
        rightMotor.setSpeed(pwm_right);

        else if (cmd.equalsIgnoreCase("ElevatorTO") && numArgs >= 1)
        {
          // Interpret first arg as target height in mm (absolute position)
          float targetHeightMM = commandParser.getArgAsFloat(0);
          long targetSteps = (long)(targetHeightMM * STEPS_PER_MM_ELEVATION_TURRET);
          elevationStepper.moveTo(targetSteps);
          Serial.print("Moving elevator to ");
          Serial.print(targetHeightMM);
          Serial.print(" mm (");
          Serial.print(targetSteps);
          Serial.println(" steps)");
        }
        else if (cmd.equalsIgnoreCase("ArmTO") && numArgs >= 1)
        { // Assuming "ArmTO" for extension
          // Interpret first arg as target distance in mm (absolute position)
          float targetDistanceMM = commandParser.getArgAsFloat(0);
          long targetSteps = (long)(targetDistanceMM * STEPS_PER_MM_EXTENSION);
          extensionStepper.moveTo(targetSteps);
          Serial.print("Moving arm to ");
          Serial.print(targetDistanceMM);
          Serial.print(" mm (");
          Serial.print(targetSteps);
          Serial.println(" steps)");
        }
        else if (cmd.equalsIgnoreCase("STOP"))
        {
          // Emergency Stop or halt command
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);
          // Stop steppers smoothly (by setting target to current pos)
          elevationStepper.stop(); // Or elevationStepper.moveTo(elevationStepper.currentPosition());
          extensionStepper.stop(); // Or extensionStepper.moveTo(extensionStepper.currentPosition());
          // Or disable steppers if needed (driver dependent)
          Serial.println("STOP command received. Halting all motors.");
        }
        else
        {
          Serial.print("Unknown command or wrong arguments: ");
          Serial.println(cmd);
        }

        // Important: Clear the command after processing
        commandParser.clearCommand();
      }
    }

    // 2. Run Stepper Motors
    // These MUST be called frequently in the loop for AccelStepper to work
    elevationStepper.run();
    extensionStepper.run();

    // 3. Other periodic tasks (if any) can go here
  }