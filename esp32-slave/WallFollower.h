/**
 * WallFollower.h
 * Header for wall follower class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#ifndef WALLFOLLOWER_h
#define WALLFOLLOWER_h

#include <Arduino.h>  // arduino commands
#include <Adafruit_VL53L0X.h>  // time of flight sensor
#include "MecanumBase.h"  // mecanum base
#include "TripleTOF.h"  // multiplexed time of flight sensors

struct WallPose {
    unsigned int front;
    unsigned int right;
    float theta;
};

struct FollowerGains {
    float kpWall;
    float kpRot;
};

// struct Twist2D {
//     float vx;
//     float vy;
//     float vtheta;
// };

enum FollowerState { notFollowing, findingWall, wallFollowing, turningCorner };

/**
 * Wall follower class for use with one front-facing and two right-facing time 
 * of flight sensors on a mecanum base.
 */
class WallFollower 
{
    public:
        WallPose _currentPose;
        FollowerState _state;
        TripleRange _ranges;

        /**
         * Constructor for wall follower class.
         * 
         * @param base          Mecanum base.
         * @param ttof          Multiplexed time of flight sensors.
         * @param setDistance   Desired wall distance.
         * @param turnDistance  Threshold for turning at corners.
         * @param kpWall        Wall distance proportional gain.
         * @param kpRotation    Base orientation proportional gain.
         * @param findSpeed     Speed while finding wall in range [0.0, 1.0].
         * @param followSpeed   Speed while following wall in range [0.0, 1.0].
         */
        WallFollower(MecanumBase& base,
                     TripleTOF& ttof,
                     unsigned int rightSensorDistance = 237,
                     unsigned int setDistance = 100,
                     unsigned int turnDistance = 200,
                     unsigned int rotDistance = 400,
                     float kpWall = 1.0,
                     float kpRotation = 1.0,
                     float findSpeed = 0.7,
                     float followSpeed = 0.5,
                     float rotationSpeed = 0.5);

        /**
         * Enable wall following.
         */
        void enable();

        /**
         * Disable wall following.
         */
        void disable();

        /**
         * Update control input and pass to base.
         */
        void update();

    private:
        MecanumBase* _base;
        TripleTOF* _ttof;
        FollowerGains _gains;
        unsigned int _sensorDistance;
        unsigned int _setDistance;
        unsigned int _rotDistance;
        unsigned int _turnDistance;
        float _findSpeed;
        float _followSpeed;
        float _rotationSpeed;

        /**
         * Finds a wall.
         */
        void findWall();

        /**
         * Follows a wall.
         */
        void followWall();

        /**
         * Turns a corner.
         */
        void turnCorner();

        /**
         * Compute control input.
         */
        Twist2D wallControl(WallPose pose, 
                            unsigned int setDistance,
                            float followSpeed,
                            FollowerGains gains);

        /**
         * Parse range inputs.
         */
        WallPose parseRanges(TripleRange ranges);
};

#endif  // WALLFOLLOWER_h
