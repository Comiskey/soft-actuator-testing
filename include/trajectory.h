#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>

class Trajectory {
public:
    Trajectory(int size);
    ~Trajectory();
    void reset();
    bool isFinished(unsigned long deltaT) const;
    bool setTrajectoryPoints(const float* newTimes, const double* newPressures, int size);
    float interp(unsigned long deltaT); 
    bool failingToFollow(double actualPressure, float deltaT, double threshold);
private:
    int maxSize;
    float* times;
    double* pressures;
    int currentSetPoint;
    unsigned long lastWithinThresholdTime;
    unsigned long timeoutDuration;
};

bool InitializeTrajectory(Trajectory* traj, const float* times, const double* pressures, int size);

#endif
