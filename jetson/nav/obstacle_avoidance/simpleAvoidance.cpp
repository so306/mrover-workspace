#include "simpleAvoidance.hpp"

#include "stateMachine.hpp"
#include "utilities.hpp"

#include <iostream>
#include <cmath>

// Constructs a SimpleAvoidance object with the input roverStateMachine.
// SimpleAvoidance is abstacted from ObstacleAvoidanceStateMachine object so it creates an
// ObstacleAvoidanceStateMachine object with the roverStateMachine. The SimpleAvoidance object will
// execute the logic for the simple avoidance algorithm
SimpleAvoidance::SimpleAvoidance( StateMachine* roverStateMachine, double thresholdDistance )
    : ObstacleAvoidanceStateMachine( roverStateMachine, thresholdDistance ) {}

// Destructs the SimpleAvoidance object.
SimpleAvoidance::~SimpleAvoidance() {}

// Drives to dummy waypoint. Once arrived, rover will drive to original waypoint
// ( original waypoint is the waypoint before obstacle avoidance was triggered )
NavState SimpleAvoidance::executeDriveAroundObs( Rover* phoebe )
{
    if( isObstacleDetected( phoebe ) )
    {
        if( phoebe->roverStatus().currentState() == NavState::DriveAroundObs )
        {
            return NavState::TurnAroundObs;
        }
        return NavState::SearchTurnAroundObs;
    }

    DriveStatus driveStatus = phoebe->drive( mObstacleAvoidancePoint );
    if( driveStatus == DriveStatus::Arrived )
    {
        if( phoebe->roverStatus().currentState() == NavState::DriveAroundObs )
        {
            return NavState::Turn;
        }
        return NavState::SearchTurn;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return phoebe->roverStatus().currentState();
    }
    if( phoebe->roverStatus().currentState() == NavState::DriveAroundObs )
    {
        return NavState::TurnAroundObs;
    }
    return NavState::SearchTurnAroundObs;
} // executeDriveAroundObs()