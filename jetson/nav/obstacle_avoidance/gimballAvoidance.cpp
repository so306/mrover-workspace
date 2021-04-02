#include "gimballAvoidance.hpp"

GimballAvoidance::GimballAvoidance( StateMachine *roverStateMachine )
    : ObstacleAvoidanceStateMachine(roverStateMachine) {

}

GimballAvoidance::~GimballAvoidance() {

}

Odometry GimballAvoidance::createAvoidancePoint( Rover* phoebe, const double distance ) {
    return phoebe->roverStatus().odometry();
}

NavState GimballAvoidance::executeTurnAroundObs( Rover* phoebe, const rapidjson::Document& roverConfig ) {
    return NavState::TurnAroundObs;
}

NavState GimballAvoidance::executeDriveAroundObs( Rover* phoebe ) {
    return NavState::DriveAroundObs;
}