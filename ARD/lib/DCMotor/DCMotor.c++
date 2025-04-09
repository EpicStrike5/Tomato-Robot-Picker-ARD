#include "DCMotor.h"

DCMotor::DCMotor(int pwmPin, int dirPin) : pwmPin(pwmPin), dirPin(dirPin), currentSpeed(0) {}

void DCMotor::setup()
{
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    analogWrite(pwmPin, 0);    // Start stopped
    digitalWrite(dirPin, LOW); // Default direction
}

void DCMotor::setSpeed(int speed)
{
    // Constrain speed to -255 to 255
    speed = constrain(speed, -255, 255);
    currentSpeed = speed;

    if (speed == 0)
    {
        analogWrite(pwmPin, 0); // Stop motor
        // Optional: You might leave direction pin as is or set to a default
    }
    else if (speed > 0)
    {
        // Forward direction (adjust HIGH/LOW based on your driver/wiring)
        digitalWrite(dirPin, HIGH); // Assuming HIGH is one direction
        analogWrite(pwmPin, speed);
    }
    else
    { // speed < 0
        // Reverse direction
        digitalWrite(dirPin, LOW);       // Assuming LOW is the other direction
        analogWrite(pwmPin, abs(speed)); // PWM value is always positive
    }
}