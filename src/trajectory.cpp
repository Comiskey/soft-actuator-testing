#include "trajectory.h"
/*
    * Trajectory class implementation
    This class is used to store user-generated trajectory data and interpolate 
    betweent the time and pressure values to create a smooth transition between
    desired pressure setpoints.

    times: array of time values
    pressures: array of pressure values
    maxSize: maximum size of the arrays as defined by the user in main.cpp
    currentSetPoint: counter to keep track of the current setpoint in the trajectory
*/

// Constructor to initialize the size of the arrays and the counter
Trajectory::Trajectory(int size) : maxSize(size), currentSetPoint(0) {
    times = new float[size];
    pressures = new double[size];
}

// Destructor to free allocated memory
Trajectory::~Trajectory() {
    delete[] times;
    delete[] pressures;
}

// Function to reset the currentSetPoint counting variable
// once a cycle is completed
void Trajectory::reset() {
    currentSetPoint = 1;
}

// Function to print the trajectory data
// for debugging purposes
void Trajectory::printTrajectory() const {
    Serial.println("Trajectory Data:");
    for (int i = 0; i < currentSetPoint; ++i) {
        Serial.print("Time: ");
        Serial.print(times[i]);
        Serial.print(", Pressure: ");
        Serial.println(pressures[i]);
    }
}

// Function to fill trajectory class instance with new data
bool Trajectory::setTrajectoryPoints(const float* newTimes, const double* newPressures, int size) {
    if (size > maxSize) {
        Serial.println("Error: The size of the lists is greater than maxSize.");
        return false;
    }
    for (int i = 0; i < size; ++i) {
        times[i] = newTimes[i];
        pressures[i] = newPressures[i];
    }
    return true;
}

// Function to linearly interpolate between two points
// in the trajectlory based on the current time
float Trajectory::interp(float deltaT) {
    if (maxSize < 2) {
        // Not enough points to interpolate
        return pressures[0];
    }

    // Find correct interval for interpolation
    // cannot simply increment currentSetPoint by 1
    // due to time variabilities in control actions
    for (int i = currentSetPoint; i < maxSize; ++i) {
        if (deltaT < times[i]) {
            currentSetPoint = i;
            break;
        }
    }
    // Perform linear interpolation
    float t1 = times[currentSetPoint - 1];
    float t2 = times[currentSetPoint];
    double p1 = pressures[currentSetPoint - 1];
    double p2 = pressures[currentSetPoint];
    float factor = (deltaT - t1) / (t2 - t1);
    return p1 + factor * (p2 - p1);  
}

// Initialize the trajectory object
bool InitializeTrajectory(Trajectory* traj, const float* times, const double* pressures, int size) {
    // Ensure the size of the arrays is less than the maxSize of the trajectory object
    if (!traj->setTrajectoryPoints(times, pressures, size)) {
        Serial.println("Error: The size of the lists is greater than maxSize of trajectory object");
        return false;
    }
    Serial.println("Trajectory init complete");
    return true;
}
