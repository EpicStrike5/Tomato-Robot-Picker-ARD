#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>

class DCMotor
{
public:
    // Constructor: pwmPin is for speed, dirPin is for direction
    DCMotor(int pwmPin, int dirPin);

    // Call this in Arduino's setup()
    void setup();

    // Configure the maximum physical speed of the motor in meters per second (m/s)
    // This is the speed achieved at MAX_PWM_VALUE
    void configureMaxSpeed(float max_speed_mps);

    // Set motor speed based on a physical value: meters per second (m/s)
    // Positive for forward, negative for reverse.
    void setSpeedMPS(float speed_mps);

    // Set motor speed using PWM: -MAX_PWM_VALUE (full reverse) to MAX_PWM_VALUE (full forward), 0 stops
    // This can be kept for testing or lower-level control.
    void setSpeedPWM(int pwm_value);

    // Optional: Getter for current PWM (useful for debugging)
    int getCurrentPWM() const;

private:
    int pwmPin;
    int dirPin;
    float maxSpeedMPS; // Maximum speed in m/s corresponding to MAX_PWM_VALUE
    int currentPWM;    // Stores the current PWM value applied

    // Define MAX_PWM_VALUE internally for this class
    static const int MAX_PWM_VALUE = 255;
};

#endif // DCMOTOR_H
