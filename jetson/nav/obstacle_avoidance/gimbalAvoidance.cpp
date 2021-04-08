#include "gimbalAvoidance.hpp"

GimbalAvoidance::GimbalAvoidance( StateMachine *roverStateMachine, double thresholdDistance )
    : ObstacleAvoidanceStateMachine( roverStateMachine, thresholdDistance ) {
}

GimbalAvoidance::~GimbalAvoidance() {
    
}

NavState GimbalAvoidance::executeDriveAroundObs( Rover* phoebe ) {
    
    return NavState::DriveAroundObs;
}