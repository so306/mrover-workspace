#include "gimbalAvoidance.hpp"

GimbalAvoidance::GimbalAvoidance( StateMachine *roverStateMachine, double thresholdDistance )
    : ObstacleAvoidanceStateMachine( roverStateMachine, thresholdDistance ) {
}

GimbalAvoidance::~GimbalAvoidance() {
    
}

NavState GimbalAvoidance::executeTurnAroundObs( Rover* phoebe, const rapidjson::Document& roverConfig ) {
    return NavState::TurnAroundObs;
}

NavState GimbalAvoidance::executeDriveAroundObs( Rover* phoebe ) {
    return NavState::DriveAroundObs;
}