#ifndef SIMPLE_AVOIDANCE_HPP
#define SIMPLE_AVOIDANCE_HPP

#include "obstacleAvoidanceStateMachine.hpp"

// This class implements the logic for the simple obstacle avoidance algorithm.
// If an obstacle is seen, create an avoidance point using trigonometry with the angle turned and
// distance from obstacle.
class SimpleAvoidance : public ObstacleAvoidanceStateMachine
{
public:
    SimpleAvoidance( StateMachine* roverStateMachine, double thresholdDistance );

    ~SimpleAvoidance();

    NavState executeDriveAroundObs( Rover* phoebe, const rapidjson::Document& roverConfig );
};

#endif //SIMPLE_AVOIDANCE_HPP
