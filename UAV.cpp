#define _USE_MATH_DEFINES
#include "UAV.h"
#include <cmath>
#include <fstream>
#include <iomanip>

// Define constants
const double DISTANCE_TOLERANCE = 0.01;
const double DEGREE_TO_RADIAN = M_PI / 180.0;
const double RADIAN_TO_DEGREE = 180.0 / M_PI;
const double ROUND_PRECISION = 100.0;
const double FULL_CIRCLE = 360.0;


// Global function declarations needed for UAV methods
std::pair<double, double> calcNextPointLinear(double x, double y, double v, double az, double t);
std::pair<double, double> calcNextPointCircular(double a, double b, double r, double x, double y, double v, double t);
double calcNewAzimut(double targetX, double targetY, double currentX, double currentY);

UAV::UAV(double startX, double startY, double startZ, double startSpeed, double startAzimuth)
    : x(startX), y(startY), z(startZ), speed(startSpeed), azimuth(startAzimuth) {
}

double UAV::getX() const { return x; }
double UAV::getY() const { return y; }
double UAV::getAzimuth() const { return azimuth; }

void UAV::setAzimuth(double newAzimuth) {
    azimuth = newAzimuth;
}

void UAV::setTarget(double newTargetX, double newTargetY) {
    targetX = newTargetX;
    targetY = newTargetY;
}

bool UAV::hasTarget() const {
    return targetX.has_value() && targetY.has_value();
}

bool UAV::isCloseToTarget(double radius) const {
    if (targetX.has_value() && targetY.has_value()) {
        double distance = std::sqrt(
            (x - targetX.value()) * (x - targetX.value()) +
            (y - targetY.value()) * (y - targetY.value())
        );
        return distance <= radius + DISTANCE_TOLERANCE;
    }
    return false;
}

void UAV::setToLinearMove(double dt) {
    auto [newX, newY] = calcNextPointLinear(x, y, speed, azimuth, dt);
    x = newX;
    y = newY;
}

void UAV::setToCircularMove(double dt, double radius) {
    auto [newX, newY] = calcNextPointCircular(targetX.value(), targetY.value(), radius, x, y, speed, dt);
    x = std::round(newX * ROUND_PRECISION) / ROUND_PRECISION;
    y = std::round(newY * ROUND_PRECISION) / ROUND_PRECISION;
    updateAzimuthDuringCircular(targetX.value(), targetY.value(), radius);
}

void UAV::updateAzimuthDuringCircular(double centerX, double centerY, double radius) {
    double angle = std::atan2(y - centerY, x - centerX) * RADIAN_TO_DEGREE;
    azimuth = angle - 90.0;

    if (azimuth < 0) {
        azimuth += FULL_CIRCLE;
    }
    azimuth = std::round(azimuth);
}

void UAV::writeStateToFile(std::ofstream& outputFile, double time) const {
    outputFile << std::fixed << std::setprecision(2)
        << time << " " << x
        << " " << y << " " << azimuth
        << std::endl;
}

double UAV::getTargetY() {
    return targetY.value();
}

double UAV::getTargetX() {
    return targetX.value();
}
