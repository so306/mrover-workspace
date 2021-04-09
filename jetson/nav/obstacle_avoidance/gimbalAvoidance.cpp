#include "gimbalAvoidance.hpp"
#include "utilities.hpp"
#include "gimbal.hpp"

GimbalAvoidance::GimbalAvoidance( StateMachine *roverStateMachine, double thresholdDistance )
    : ObstacleAvoidanceStateMachine( roverStateMachine, thresholdDistance ) {
}

GimbalAvoidance::~GimbalAvoidance() {
    
}

NavState GimbalAvoidance::executeDriveAroundObs( Rover* phoebe, const rapidjson::Document& roverConfig ) {
    DriveStatus driveStatus = phoebe->drive( mObstacleAvoidancePoint );
    if(driveStatus == DriveStatus::Arrived) return NavState::DriveAroundObs; // TODO progress states
    double distanceRemaining = calcDistance(phoebe->roverStatus().odometry(), mObstacleAvoidancePoint);
    if(distanceRemaining < roverConfig["computerVision"]["visionDistance"].GetDouble()) {
        phoebe->gimbal().setDesiredGimbalYaw(90); // TODO put on correct side of rover
    }
    
    return NavState::DriveAroundObs;
}