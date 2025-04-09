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

    // Set motor speed: -255 (full reverse) to 255 (full forward), 0 stops
    void setSpeed(int speed);

private:
    int pwmPin;
    int dirPin;
    int currentSpeed;
};

#endif // DCMOTOR_H