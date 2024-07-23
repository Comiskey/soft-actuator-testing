#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>

class Trajectory {
public:
    Trajectory(int size);
    ~Trajectory();
    void reset();
    void printTrajectory() const;
    bool setTrajectoryPoints(const float* newTimes, const double* newPressures, int size);
    float interp(float deltaT); 
private:
    int maxSize;
    float* times;
    double* pressures;
    int currentSetPoint;
};

bool InitializeTrajectory(Trajectory* traj, const float* times, const double* pressures, int size);

#endif
