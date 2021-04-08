#ifndef OBSTACLE_AVOIDANCE_STATE_MACHINE_HPP
#define OBSTACLE_AVOIDANCE_STATE_MACHINE_HPP

#include "rover.hpp"

class StateMachine;

// This class is the representation of different 
// obstacle avoidance algorithms
enum class ObstacleAvoidanceAlgorithm
{
    SimpleAvoidance,
    GimbalAvoidance
};

// This class is the base class for the logic of the obstacle avoidance state machine
class ObstacleAvoidanceStateMachine 
{
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    ObstacleAvoidanceStateMachine( StateMachine* stateMachine_, double thresholdDistance );

    virtual ~ObstacleAvoidanceStateMachine() {}

    void updateObstacleAngle( double bearing );

    void updateObstacleDistance( double distance );

    void updateObstacleElements( double bearing, double distance );  

    Odometry createAvoidancePoint( Rover* phoebe, const double distance );

    NavState run( Rover* phoebe, const rapidjson::Document& roverConfig );

    bool isTargetDetected( Rover* phoebe );

    virtual NavState executeTurnAroundObs( Rover* phoebe, const rapidjson::Document& roverConfig );

    virtual NavState executeDriveAroundObs( Rover* phoebe ) = 0;

protected:
    /*************************************************************************/
    /* Protected Member Variables */
    /*************************************************************************/
    
    // Pointer to rover State Machine to access member functions
    StateMachine* roverStateMachine;

    // Odometry point used when avoiding obstacles.
    Odometry mObstacleAvoidancePoint;

    // Initial angle to go around obstacle upon detection.
    double mOriginalObstacleAngle;

    // Initial distance to go around obstacle upon detection.
    double mOriginalObstacleDistance;

    // Threshold distance to add to obstacle distance
    double mThresholdDistance;

    // bool for consecutive obstacle detections
    bool mJustDetectedObstacle;

    // Last obstacle angle for consecutive angles
    double mLastObstacleAngle;
};

// Creates an ObstacleAvoidanceStateMachine object based on the inputted obstacle 
// avoidance algorithm. This allows for an an ease of transition between obstacle 
// avoidance algorithms
ObstacleAvoidanceStateMachine* ObstacleAvoiderFactory( StateMachine* roverStateMachine,
                                                       ObstacleAvoidanceAlgorithm algorithm,
                                                       double thresholdDistance );

#endif //OBSTACLE_AVOIDANCE_STATE_MACHINE_HPP
