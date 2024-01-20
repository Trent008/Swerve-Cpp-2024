#pragma once
#include "complex"

using namespace std;

struct AutonomousSetpoint
{
    complex<float> robotPosition;
    float robotAngle = 0;
};
