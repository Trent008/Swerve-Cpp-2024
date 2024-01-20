#pragma once
#include "complex"

using namespace std;

// parameters for robot movement and autonomous
struct Parameters
{  
    // to convert to degrees
    float const degrees = (180/M_PI);

    // to convert to radians
    float const radians = (M_PI/180);

    // max amperage for the drive motor PID controllers
    float const driveMotorMaxAmperage = 45;

    // diameter of the drive wheels
    float const wheelDiameter = 3.9;

    // inches of travel per rotation of the motor
    float const driveMotorInPerRot = (M_PI * wheelDiameter / 6.75);

    // percent to change the robot velocity per teleop cycle
    float const slewRate = 0.04;

    // max drive rate for autonomous
    float const autoMaxDriveRate = 0.2;

    // max rotation rate for autonomous
    float const autoMaxTurnRate = 0.2;

    // proportional constant for autonomous position error
    float const autoPositionP = 0.01;

    // proportional constant for autonomous angle error
    float const autoAngleP = 0.005;

    // robot starting position on the field
    complex<float> startingPosition = {0, 0};
    // robot starting angle on the field (degrees)
    float startingAngle = 0;
    
} parameters;