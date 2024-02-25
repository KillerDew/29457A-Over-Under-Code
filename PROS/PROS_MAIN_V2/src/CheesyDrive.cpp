#include "main.h"
#include <algorithm>
#include <cmath>

extern DriveCommands CurvatureDrive(double throttle, double curvature, double TSpeed, double DSpeed){
    double left;
    double right;
    throttle = std::clamp(throttle, -1.0, 1.0) * DSpeed;
    curvature = std::clamp(curvature, -1.0, 1.0) * TSpeed;
    left = throttle - curvature;
    right = throttle + curvature;

    double maxMag = std::max(abs(left), abs(right));
    if (maxMag > 1.0){
        left /= maxMag;
        right /= maxMag;
    }
    DriveCommands output;
    output.left = left;
    output.right = right;
    return output;
}