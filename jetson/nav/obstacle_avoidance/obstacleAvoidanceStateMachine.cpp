#include "obstacleAvoidanceStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "simpleAvoidance.hpp"
#include "gimbalAvoidance.hpp"
#include <cmath>
#include <iostream>

// Constructs an ObstacleAvoidanceStateMachine object with roverStateMachine
ObstacleAvoidanceStateMachine::ObstacleAvoidanceStateMachine( StateMachine* stateMachine_, double thresholdDistance)
    : roverStateMachine( stateMachine_ ),
      mThresholdDistance(thresholdDistance),
      mJustDetectedObstacle( false ) {}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleAngle( double bearing )
{
    mOriginalObstacleAngle = bearing;
}

// Allows outside objects to set the original obstacle distance
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleDistance( double distance )
{
    mOriginalObstacleDistance = distance + mThresholdDistance;
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleElements( double bearing, double distance )
{
    updateObstacleAngle( bearing );
    updateObstacleDistance( distance );
}

// Create the odometry point used to drive around an obstacle
Odometry ObstacleAvoidanceStateMachine::createAvoidancePoint( Rover* phoebe, const double distance )
{
    Odometry avoidancePoint = phoebe->roverStatus().odometry();
    double totalLatitudeMinutes = avoidancePoint.latitude_min +
        cos( degreeToRadian( avoidancePoint.bearing_deg ) ) * distance * LAT_METER_IN_MINUTES;
    double totalLongitudeMinutes = avoidancePoint.longitude_min +
        sin( degreeToRadian( avoidancePoint.bearing_deg ) ) * distance * phoebe->longMeterInMinutes();
    avoidancePoint.latitude_deg += totalLatitudeMinutes / 60;
    avoidancePoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes) / 60 ) * 60 );
    avoidancePoint.longitude_deg += totalLongitudeMinutes / 60;
    avoidancePoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

    return avoidancePoint;

} // createAvoidancePoint()

// Runs the avoidance state machine through one iteration. This will be called by StateMachine
// when NavState is in an obstacle avoidance state. This will call the corresponding function based
// on the current state and return the next NavState
NavState ObstacleAvoidanceStateMachine::run( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    switch ( phoebe->roverStatus().currentState() )
    {
        case NavState::TurnAroundObs:
        case NavState::SearchTurnAroundObs:
        {
            return executeTurnAroundObs( phoebe, roverConfig );
        }

        case NavState::DriveAroundObs:
        case NavState::SearchDriveAroundObs:
        {
            return executeDriveAroundObs( phoebe );
        }

        default:
        {
            cerr << "Entered unknown NavState in obstacleAvoidanceStateMachine" << endl;
            return NavState::Unknown;
        }
    } // switch
}

// Checks that both rover is in search state and that target is detected
bool ObstacleAvoidanceStateMachine::isTargetDetected ( Rover* phoebe )
{
    return ( phoebe->roverStatus().currentState() == NavState::SearchTurnAroundObs &&
             phoebe->roverStatus().leftTarget().distance >= 0 );
}

// Turn away from obstacle until it is no longer detected.
// If in search state and target is both detected and reachable, return NavState TurnToTarget.
// ASSUMPTION: There is no rock that is more than 8 meters (pathWidth * 2) in diameter
NavState SimpleAvoidance::executeTurnAroundObs( Rover* phoebe,
                                                const rapidjson::Document& roverConfig )
{
    if( isTargetDetected ( phoebe ) && isTargetReachable( phoebe, roverConfig ) )
    {
        return NavState::TurnToTarget;
    }
    if( !isObstacleDetected( phoebe ) )
    {
        double distanceAroundObs = mOriginalObstacleDistance /
                                   cos( fabs( degreeToRadian( mOriginalObstacleAngle ) ) );
        mObstacleAvoidancePoint = createAvoidancePoint( phoebe, distanceAroundObs );
        if( phoebe->roverStatus().currentState() == NavState::TurnAroundObs )
        {
            return NavState::DriveAroundObs;
        }
        mJustDetectedObstacle = false;
        return NavState::SearchDriveAroundObs;
    }

    double obstacleBearing = phoebe->roverStatus().obstacle().bearing;
    if( mJustDetectedObstacle &&
        ( obstacleBearing < 0 ? mLastObstacleAngle >= 0 : mLastObstacleAngle < 0 ) ) {
        obstacleBearing *= -1;
    }

    double desiredBearing = mod( phoebe->roverStatus().odometry().bearing_deg + obstacleBearing, 360 );
    mJustDetectedObstacle = true;
    mLastObstacleAngle = obstacleBearing;
    phoebe->turn( desiredBearing );
    return phoebe->roverStatus().currentState();
} // executeTurnAroundObs()

// The obstacle avoidance factory allows for the creation of obstacle avoidance objects and
// an ease of transition between obstacle avoidance algorithms
ObstacleAvoidanceStateMachine* ObstacleAvoiderFactory ( StateMachine* roverStateMachine,
                                                        ObstacleAvoidanceAlgorithm algorithm,
                                                        double thresholdDistance )
{
    ObstacleAvoidanceStateMachine* avoid = nullptr;
    switch ( algorithm )
    {
        case ObstacleAvoidanceAlgorithm::SimpleAvoidance:
            avoid = new SimpleAvoidance( roverStateMachine, thresholdDistance );
            break;
        case ObstacleAvoidanceAlgorithm::GimbalAvoidance:
            avoid = new GimbalAvoidance( roverStateMachine, thresholdDistance );
        default:
            std::cerr << "Unkown Search Type. Defaulting to original\n";
            avoid = new SimpleAvoidance( roverStateMachine, thresholdDistance );
            break;
    } // switch
    return avoid;
} // ObstacleAvoiderFactory

