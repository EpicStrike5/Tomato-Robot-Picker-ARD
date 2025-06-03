#include "DCMotor.h"

// Initialize maxSpeedMPS to a small default, should be configured properly via configureMaxSpeed()
DCMotor::DCMotor(int pwmPin, int dirPin) : pwmPin(pwmPin), dirPin(dirPin), maxSpeedMPS(0.01), currentPWM(0) {} // Default maxSpeedMPS to a very small positive value

void DCMotor::setup()
{
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    analogWrite(pwmPin, 0);    // Start stopped
    digitalWrite(dirPin, LOW); // Default direction (adjust if necessary for your wiring)
}

void DCMotor::configureMaxSpeed(float max_speed_mps)
{
    if (max_speed_mps > 0.0001)
    { // Max speed must be meaningfully positive
        this->maxSpeedMPS = max_speed_mps;
    }
    else
    {
        // Handle error or set a sensible default if max_speed_mps is invalid
        if (Serial)
        { // Check if Serial is initialized
            Serial.println(F("Error: DCMotor max_speed_mps must be positive. Using previous/default."));
        }
        // Ensure maxSpeedMPS remains a small positive value to prevent division by zero
        if (this->maxSpeedMPS <= 0.0001)
            this->maxSpeedMPS = 0.01;
    }
}

void DCMotor::setSpeedMPS(float speed_mps)
{
    if (this->maxSpeedMPS <= 0.0001)
    { // Check if maxSpeedMPS is valid
        if (Serial)
        {
            Serial.println(F("Error: DCMotor maxSpeedMPS not configured or invalid. Cannot set speed in m/s. Stopping motor."));
        }
        setSpeedPWM(0); // Stop the motor
        return;
    }

    // Calculate PWM value: (desired_speed / max_speed) * MAX_PWM
    // This gives a value from -MAX_PWM_VALUE to +MAX_PWM_VALUE
    int pwm_value = (int)((speed_mps / this->maxSpeedMPS) * MAX_PWM_VALUE);

    setSpeedPWM(pwm_value);
}

void DCMotor::setSpeedPWM(int pwm_value)
{
    // Constrain pwm_value to -MAX_PWM_VALUE to MAX_PWM_VALUE
    pwm_value = constrain(pwm_value, -MAX_PWM_VALUE, MAX_PWM_VALUE);
    currentPWM = pwm_value;

    if (pwm_value == 0)
    {
        analogWrite(pwmPin, 0); // Stop motor
        // Direction pin can be left as is or set to a default.
    }
    else if (pwm_value > 0)
    {
        // Forward direction (adjust HIGH/LOW based on your driver/wiring for "forward")
        digitalWrite(dirPin, HIGH);
        analogWrite(pwmPin, pwm_value);
    }
    else
    { // pwm_value < 0
        // Reverse direction (adjust HIGH/LOW based on your driver/wiring for "reverse")
        digitalWrite(dirPin, LOW);
        analogWrite(pwmPin, abs(pwm_value)); // PWM value is always positive
    }
}

int DCMotor::getCurrentPWM() const
{
    return currentPWM;
}
