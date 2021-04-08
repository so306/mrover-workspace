#ifndef GIMBAL_AVOIDANCE_HPP
#define GIMBAL_AVOIDANCE_HPP

#include "obstacleAvoidanceStateMachine.hpp"

class GimbalAvoidance : public ObstacleAvoidanceStateMachine 
{

enum GimbalAvoidanceState {
    InitialDrive,
    IntialGimbalTurnedDrive,
    DriveGimbalStraight,
    DriveGimbalTurned
}
public:
    GimbalAvoidance( StateMachine *roverStateMachine, double thresholdDistance );

    ~GimbalAvoidance();

    virtual NavState executeDriveAroundObs( Rover* phoebe );
};

#endif