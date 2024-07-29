#include "trajectory.h"
#include "adjustableSettings.h"
/*
    * Trajectory class implementation
    This class is used to store user-generated trajectory data and interpolate 
    betweent the time and pressure values to create a smooth transition between
    desired pressure setpoints.

    times: array of time values
    pressures: array of pressure values
    maxSize: maximum size of the arrays as defined by the user in main.cpp
    currentSetPoint: counter to keep track of the current setpoint in the trajectory
    timeOut: time out value to prevent the system from hanging if the trajectory is not completed
*/

// Constructor to initialize the size of the arrays and the counter
Trajectory::Trajectory(int size) : maxSize(size), currentSetPoint(0) {
    times = new float[size];
    pressures = new double[size];
    timeoutDuration = 2000; // 2 seconds
    lastWithinThresholdTime = millis();
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
    lastWithinThresholdTime = millis();
}

// Function to fill trajectory class instance with new data
// and to set timeOut value
bool Trajectory::setTrajectoryPoints(const float* newTimes, const double* newPressures, int size) {
    if (size > maxSize) {
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
float Trajectory::interp(unsigned long deltaT) {
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

// Function to check if the system failing to follow the trajectory
bool Trajectory::failingToFollow(double currentPressure, float deltaT, double threshold) {
    double expectedPressure = pressures[currentSetPoint];
    // Check if current pressure is too far from the non-zero setpoint
    if (abs(expectedPressure - currentPressure) > threshold && expectedPressure != 0.0) {
        // If it is, check if how long since the last time pressure was within threshold
        if ((millis() - lastWithinThresholdTime) > timeoutDuration){
            // if it has been too long, declare failure to follow trajectory
            return true;
        }
    } else { // if pressure is within threshold, update lastWithinThresholdTime
        lastWithinThresholdTime = millis();
    }
    // if both failure criterion aren't met, return false
    return false;
}

// Function to print the trajectory data
// for debugging purposes
bool Trajectory::isFinished(unsigned long deltaT) const {
    return deltaT > times[maxSize - 1];
}

// Initialize the trajectory object
bool InitializeTrajectory(Trajectory* traj, const float* times, const double* pressures, int size) {
    // Ensure the size of the arrays is less than the maxSize of the trajectory object
    if (!traj->setTrajectoryPoints(times, pressures, size)) {
        return false;
    }
    return true;
}
