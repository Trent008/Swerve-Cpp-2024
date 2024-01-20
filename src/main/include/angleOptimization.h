#pragma once
#include "math.h"

// return the sum of angle1 and angle2 limited to -M_PI to M_PI radians
float angleSum(float const &angle1, float const &angle2) {
    float res = angle1 + angle2;
    while (res < -180) {
        res += 360;
    }
    while (res > 180) {
        res -= 360;
    }
    return res;
}

// return the difference between angle1 and angle2 limited to -M_PI to M_PI radians
float angleDifference(float const &angle1, float const &angle2) {
    float res = angle1 - angle2;
    while (res < -180) {
        res += 360;
    }
    while (res > 180) {
        res -= 360;
    }
    return res;
}