#ifndef GIMBALL_AVOIDANCE_HPP
#define GIMBALL_AVOIDANCE_HPP

#include "obstacleAvoidanceStateMachine.hpp"

class GimballAvoidance : public ObstacleAvoidanceStateMachine 
{
public:
    GimballAvoidance( StateMachine *roverStateMachine );

    ~GimballAvoidance();

    virtual Odometry createAvoidancePoint( Rover* phoebe, const double distance );

    virtual NavState executeTurnAroundObs( Rover* phoebe, const rapidjson::Document& roverConfig );

    virtual NavState executeDriveAroundObs( Rover* phoebe );
};

#endif