#define _USE_MATH_DEFINES
#include "UAV.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <queue> // Includes Priority Queue
#include <cmath>
#include <optional>
#include <utility> // For std::pair


struct CommandCompare;
class UAV;

// Constants
const double EPSILON = 1e-6; // Small error threshold for calculations
const double DEG_TO_RAD = M_PI / 180.0; // Conversion factor from degrees to radians
const double RAD_TO_DEG = 180.0 / M_PI; // Conversion factor from radians to degrees
const double FULL_CIRCLE = 360.0;

// Error messages
const std::string ERROR_FILE_OPEN = "Failed to open file: ";
const std::string ERROR_FORMAT = "Invalid format in line: ";

// Constants for keys in the parameter file
const std::string KEY_DT = "Dt";
const std::string KEY_N_UAV = "N_uav";
const std::string KEY_RADIUS = "R";
const std::string KEY_X0 = "X0";
const std::string KEY_Y0 = "Y0";
const std::string KEY_Z0 = "Z0";
const std::string KEY_V0 = "V0";
const std::string KEY_AZ = "Az";
const std::string KEY_TIMELIM = "TimeLim";

/**
 * Structure to hold simulation parameters loaded from a file.
 */
struct SimulationParams {
    double dt;
    int numUAVs;
    double radius;
    double startX;
    double startY;
    double startZ;
    double startSpeed;
    double startAzimuth;
    double timeLimit;
};

/**
 * Structure to define a command for a UAV.
 */
struct Command {
    double time;
    int num;
    double x;
    double y;
};

/**
 * Comparator structure for prioritizing commands in a priority queue based on time.
 */
struct CommandCompare {
    bool operator()(const Command& a, const Command& b) const {
        return a.time > b.time; // Min-heap based on time
    }
};

// Function declarations
std::pair<double, double> calculate_next_point_linear(double x, double y, double v, double az, double t);
std::pair<double, double> calculate_next_point_circular(double a, double b, double r, double x, double y, double v, double t);
bool loadSimulationParams(const std::string& filename, SimulationParams& params);
bool loadCommands(const std::string& filename, std::priority_queue<Command, std::vector<Command>, CommandCompare>& commands);
double calcNewAzimut(double targetX, double targetY, double currentX, double currentY);
void handleUAV(UAV& uav, std::priority_queue<Command, std::vector<Command>, CommandCompare>& commands,
    std::ofstream& outputFile, double time, const SimulationParams& params, int uavIndex);

/**
 * Load simulation parameters from a file.
 * @param filename Path to the parameter file.
 * @param params Reference to the SimulationParams structure to populate.
 * @return True if loading succeeded, false otherwise.
 */
bool loadSimulationParams(const std::string& filename, SimulationParams& params) {
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        std::cerr << ERROR_FILE_OPEN << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        std::string key;
        char equals;
        if (iss >> key >> equals) {
            if (equals != '=') {
                std::cerr << ERROR_FORMAT << line << std::endl;
                continue;
            }

            if (key == KEY_DT) {
                iss >> params.dt;
            }
            else if (key == KEY_N_UAV) {
                iss >> params.numUAVs;
            }
            else if (key == KEY_RADIUS) {
                iss >> params.radius;
            }
            else if (key == KEY_X0) {
                iss >> params.startX;
            }
            else if (key == KEY_Y0) {
                iss >> params.startY;
            }
            else if (key == KEY_Z0) {
                iss >> params.startZ;
            }
            else if (key == KEY_V0) {
                iss >> params.startSpeed;
            }
            else if (key == KEY_AZ) {
                iss >> params.startAzimuth;
            }
            else if (key == KEY_TIMELIM) {
                iss >> params.timeLimit;
            }
        }
    }

    inputFile.close();
    return true;
}

/**
 * Load commands for UAVs from a file into a priority queue.
 * @param filename Path to the commands file.
 * @param commands Priority queue to store the commands.
 * @return True if loading succeeded, false otherwise.
 */
bool loadCommands(const std::string& filename, std::priority_queue<Command, std::vector<Command>, CommandCompare>& commands) {
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        std::cerr << ERROR_FILE_OPEN << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        Command command;
        if (iss >> command.time >> command.num >> command.x >> command.y) {
            commands.push(command);
        }
        else {
            std::cerr << ERROR_FORMAT << line << std::endl;
        }
    }

    inputFile.close();
    return true;
}

/**
 * Handle the state and commands of a UAV at a specific simulation time.
 * @param uav Reference to the UAV object.
 * @param commands Priority queue of commands.
 * @param outputFile Output file to log UAV state.
 * @param time Current simulation time.
 * @param params Simulation parameters.
 * @param uavIndex Index of the UAV.
 */
void handleUAV(UAV& uav, std::priority_queue<Command, std::vector<Command>, CommandCompare>& commands,
    std::ofstream& outputFile, double time, const SimulationParams& params, int uavIndex) {

    uav.writeStateToFile(outputFile, time);

    if (!commands.empty() && commands.top().time <= time && commands.top().num == uavIndex) {
        const Command& cmd = commands.top();
        auto azimuth = calcNewAzimut(cmd.x, cmd.y, uav.getX(), uav.getY());
        uav.setAzimuth(azimuth);
        uav.setTarget(cmd.x, cmd.y);
        commands.pop();
    }

    if (uav.hasTarget() && uav.isCloseToTarget(params.radius)) {
        uav.setToCircularMove(params.dt, params.radius);
    }
    else {
        uav.setToLinearMove(params.dt);
    }
}

/**
 * Calculate the next point for circular movement.
 */
std::pair<double, double> calcNextPointCircular(double centerX, double centerY, double
    radius,
    double currentX, double currentY, double speed, double dt) {
    double currentAngle = std::atan2(currentY - centerY, currentX - centerX);
    double angularSpeed = speed / radius;
    double deltaAngle = angularSpeed * dt;
    double newAngle = currentAngle - deltaAngle;

    double newX = centerX + radius * std::cos(newAngle);
    double newY = centerY + radius * std::sin(newAngle);

    return { newX, newY };
}

/**
 * Calculate the next point for linear movement.
 */
std::pair<double, double> calcNextPointLinear(double x, double y, double v, double az,
    double
    t) {
    double az_radians = az * DEG_TO_RAD;

    double dx = v * std::cos(az_radians) * t;
    double dy = v * std::sin(az_radians) * t;

    if (std::fabs(dx) < EPSILON) dx = 0.0;
    if (std::fabs(dy) < EPSILON) dy = 0.0;

    double x_next = x + dx;
    double y_next = y + dy;

    return { x_next, y_next };
}

/**
 * Calculate the azimuth to a target point.
 */
double calcNewAzimut(double targetX, double targetY, double currentX, double currentY) {
    double deltaX = targetX - currentX;
    double deltaY = targetY - currentY;

    double azimut = std::atan2(deltaY, deltaX) * RAD_TO_DEG;
    if (azimut < 0) {
        azimut += FULL_CIRCLE;
    }

    return azimut;
}

/**
 * Main function that manages the logic of the UAV simulation.
 * The function loads parameters and commands from input files, creates UAV objects,
 * and performs the steps for each UAV in real-time.
 *
 * @return 0 if the simulation completes successfully.
 */
int main() {
    SimulationParams params;
    const std::string fileParams = "SimParams.ini";
    loadSimulationParams(fileParams, params);

    std::priority_queue<Command, std::vector<Command>, CommandCompare> commands;
    const std::string fileCommands = "SimCmds.txt";
    loadCommands(fileCommands, commands);

    std::vector<UAV> uavs;
    std::vector<std::ofstream> outputFiles;
    for (int i = 0; i < params.numUAVs; ++i) {
        UAV uav(params.startX, params.startY, params.startZ, params.startSpeed, params.startAzimuth);
        uavs.push_back(uav);

        std::string filename = "UAV" + std::to_string(i) + ".txt";
        outputFiles.emplace_back(filename);
    }

    double time = 0;
    while (time <= params.timeLimit) {
        for (int i = 0; i < params.numUAVs; ++i) {
            handleUAV(uavs[i], commands, outputFiles[i], time, params, i);
        }

        time += params.dt;
    }

    return 0;
}
