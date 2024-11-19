#ifndef UAV_H
#define UAV_H
#include <optional>
#include <fstream>

/**
 * @class UAV
 * @brief Represents an Unmanned Aerial Vehicle (UAV).
 *
 * The UAV class models a UAV's position, speed, azimuth (direction), and target tracking. It provides methods to:
 * 1. Move the UAV linearly or circularly.
 * 2. Track a target and check if the UAV is close to it.
 * 3. Write the UAV's state (position and azimuth) to a file for logging purposes.
 */
class UAV {
public:
    /**
     * @brief Constructs a UAV object with initial position, speed, and azimuth.
     *
     * This constructor initializes the UAV's position in 3D space (X, Y, Z), speed, and azimuth.
     * Optionally, the UAV can have a target to move towards.
     *
     * @param startX The starting X-coordinate of the UAV.
     * @param startY The starting Y-coordinate of the UAV.
     * @param startZ The starting Z-coordinate of the UAV.
     * @param startSpeed The speed of the UAV.
     * @param startAzimuth The initial azimuth (direction) of the UAV in degrees.
     */
    UAV(double startX, double startY, double startZ, double startSpeed, double startAzimuth);

    /**
     * @brief Gets the current X-coordinate of the UAV.
     * @return The X-coordinate of the UAV.
     */
    double getX() const;

    /**
     * @brief Gets the current Y-coordinate of the UAV.
     * @return The Y-coordinate of the UAV.
     */
    double getY() const;

    /**
     * @brief Gets the current azimuth (direction) of the UAV.
     * @return The azimuth of the UAV in degrees.
     */
    double getAzimuth() const;

    /**
     * @brief Sets the azimuth (direction) of the UAV.
     *
     * This method updates the UAV's azimuth (direction) to the specified value.
     *
     * @param newAzimuth The new azimuth (direction) of the UAV in degrees.
     */
    void setAzimuth(double newAzimuth);

    /**
     * @brief Sets the target coordinates for the UAV to move towards.
     *
     * This method sets the target coordinates (X, Y) for the UAV to track and move towards.
     *
     * @param newTargetX The new X-coordinate of the target.
     * @param newTargetY The new Y-coordinate of the target.
     */
    void setTarget(double newTargetX, double newTargetY);

    /**
     * @brief Checks if the UAV has a target set.
     * @return True if the UAV has a target; otherwise, false.
     */
    bool hasTarget() const;

    /**
     * @brief Checks if the UAV is close to its target within a specified radius.
     *
     * This method calculates the distance between the UAV and its target, and returns true if
     * the UAV is within the specified radius (with a small tolerance).
     *
     * @param radius The radius (distance) within which the UAV is considered close to the target.
     * @return True if the UAV is close to its target, false otherwise.
     */
    bool isCloseToTarget(double radius) const;

    /**
     * @brief Moves the UAV in a straight line based on its current azimuth and speed.
     *
     * This method updates the UAV's position based on its current direction (azimuth), speed, and
     * the time interval (dt). The new position is calculated and the UAV moves in a straight line.
     *
     * @param dt The time interval (in seconds) used to update the UAV's position.
     */
    void setToLinearMove(double dt);

    /**
     * @brief Moves the UAV in a circular path around its target.
     *
     * This method moves the UAV in a circular trajectory around its target with a specified radius.
     * The new position is calculated and the azimuth is updated accordingly.
     *
     * @param dt The time interval (in seconds) used to update the UAV's position.
     * @param radius The radius of the circular path around the target.
     */
    void setToCircularMove(double dt, double radius);

    /**
     * @brief Writes the current state of the UAV to a file.
     *
     * This method writes the UAV's state (X, Y, azimuth) and the current time to a file in a readable format.
     * The output file is written with two decimal places of precision.
     *
     * @param outputFile The output file stream to write the UAV's state to.
     * @param time The current time at which the state is logged.
     */
    void writeStateToFile(std::ofstream& outputFile, double time) const;

    /**
     * @brief Gets the Y-coordinate of the UAV's target.
     * @return The Y-coordinate of the target.
     */
    double getTargetY();

    /**
     * @brief Gets the X-coordinate of the UAV's target.
     * @return The X-coordinate of the target.
     */
    double getTargetX();

private:
    double x, y, z, speed, azimuth; ///< Position (x, y, z), speed, and azimuth of the UAV.
    std::optional<double> targetX, targetY; ///< Optional target coordinates (X, Y) of the UAV.

    /**
     * @brief Updates the azimuth (direction) of the UAV while moving in a circular path.
     *
     * This method calculates the new azimuth of the UAV during circular movement based on its current position
     * and the center of the circular path. The azimuth is adjusted to keep the UAV moving along the circular trajectory.
     *
     * @param centerX The X-coordinate of the center of the circular path.
     * @param centerY The Y-coordinate of the center of the circular path.
     * @param radius The radius of the circular path.
     */
    void updateAzimuthDuringCircular(double centerX, double centerY, double radius);
};

#endif // UAV_H
#pragma once
