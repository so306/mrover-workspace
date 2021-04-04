#ifndef GIMBAL_AVOIDANCE_HPP
#define GIMBAL_AVOIDANCE_HPP

#include "obstacleAvoidanceStateMachine.hpp"

class GimbalAvoidance : public ObstacleAvoidanceStateMachine 
{
public:
    GimbalAvoidance( StateMachine *roverStateMachine, double thresholdDistance );

    ~GimbalAvoidance();

    virtual NavState executeTurnAroundObs( Rover* phoebe, const rapidjson::Document& roverConfig );

    virtual NavState executeDriveAroundObs( Rover* phoebe );
};

#endif