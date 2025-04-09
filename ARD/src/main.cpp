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
        // Interpret args as Left Speed, Right Speed (-255 to 255)
        // This is a basic interpretation. Real MoveTO needs kinematics.
        int leftSpeed = commandParser.getArgAsLong(0);
        int rightSpeed = commandParser.getArgAsLong(1);
        leftMotor.setSpeed(leftSpeed);
        rightMotor.setSpeed(rightSpeed);
        Serial.print("Setting base motors: Left=");
        Serial.print(leftSpeed);
        Serial.print(", Right=");
        Serial.println(rightSpeed);
      }
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