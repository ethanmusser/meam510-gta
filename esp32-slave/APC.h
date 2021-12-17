/**
 * APC.h
 * Header for absolute position control (APC) class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#ifndef APC_h
#define APC_h

#include <Arduino.h>  // arduino commands
#include "vive510.h"  // vive position detection
#include "MecanumBase.h"  // mecanum base

struct Pose2D {
    float x;
    float y;
    float theta;
};

struct Twist2D {
    float vx;
    float vy;
    float vtheta;
};

struct Gains {
    float kp;
    float ki;
    float kd;
};

/**
 * Absolute positioning controller (APC) class.
 */
class APC 
{
    public:
        Gains _gains;
        float _epsilon;
        Pose2D _currentPose;
        Twist2D _currentTwist;
        Pose2D _desiredPose;
        Pose2D _previousPose;
        unsigned long _previousUpdateTime;

        /**
         * Constructor for APC with only position sensing.
         * 
         * @param base      Mobile base.
         * @param vive      Vive sensor mounted to mobile base.
         * @param xOffset   Origin offset from vive x-position in meters.
         * @param yOffset   Origin offset from vive y-position in meters.
         * @param kp        Proportional gain.
         * @param ki        Integral gain.
         * @param kd        Derivative gain.
         * @param epsilon   Allowable norm state error.
         */
        APC(MecanumBase& base,
            Vive510& vive,
            float xOffset = 0.0,
            float yOffset = 0.0,
            float kp = 1.0,
            float ki = 0.0,
            float kd = 0.2,
            float epsilon = 0.02);
        
        /**
         * Constructor for APC with position and orientation.
         * 
         * @param base      Mobile base.
         * @param frontVive Front vive sensor mounted to mobile base.
         * @param rearVive  Rear vive sensor mounted to mobile base.
         * @param xOffset   Origin offset from front vive x-position in meters.
         * @param yOffset   Origin offset from front vive y-position in meters.
         * @param qOffset   Angular offset from vive-to-vive axis in radians.
         * @param kp        Proportional gain.
         * @param ki        Integral gain.
         * @param kd        Derivative gain.
         * @param epsilon   Allowable norm state error.
         */
        APC(MecanumBase& base,
            Vive510& frontVive, 
            Vive510& rearVive,
            float xOffset = 0.0,
            float yOffset = 0.0,
            float qOffset = 0.0,
            float kp = 1.0,
            float ki = 0.0,
            float kd = 0.2,
            float epsilon = 0.02);

        /**
         * Sets base destination pose..
         * 
         * @param pose  Desired base pose.
         */
        void setDestination(Pose2D pose);
        
        /**
         * Sets base destination pose.
         * 
         * @param x     Desired base x-position.
         * @param y     Desired base y-position.
         * @param theta Desired base orientation.
         */
        void setDestination(float x = 0.0,
                            float y = 0.0,
                            float theta = 0.0);

        /**
         * Sets base offsets.
         * 
         * @param xOffset   X offset in meters.
         * @param yOffset   Y offset in meters.
         * @param qOffset   Theta offset in meters.
         */
        void setOffsets(float xOffset,
                        float yOffset,
                        float qOffset);

        /**
         * Sets APC control gains.
         * 
         * @param kp    Proportional gain.
         * @param ki    Integral gain.
         * @param kd    Derivative gain.
         */
        void setGains(float kp = 1.0,
                      float ki = 0.0,
                      float kd = 0.2);

        /**
         * Set positional error tolerance.
         * 
         * @param epsilon   Error tolerance in meters.
         */
        void setEpsilon(float epsilon = 0.02);

        /**
         * Updates controller 
         */
        void update();
        
        /**
         * Disables absolute position control.
         */
        void disable();
        
    protected:

    private:
        MecanumBase* _base;
        Vive510* _frontVive;
        Vive510* _rearVive;
        Pose2D _offsets;
        bool _hasRearVive;
        bool _isEnabled;

        /**
         * Computes current pose from vive sensor(s).
         */
        void computePose();

        /**
         * Compute control input to base.
         * 
         * @param currentPose   Current pose.
         * @param currentTwist  Current twist.
         * @param desiredPose   Desired pose.
         * @param gains         Contorller gains.
         */
        Twist2D computeControl(Pose2D currentPose,
                               Twist2D currentTwist,
                               Pose2D desiredPose,
                               Gains gains);

        /**
         * Returns the error between two poses.
         * 
         * @param current   Current pose.
         * @param desired   Desired pose.
         * @return Pose2D of error between current and desired poses.
         */
        Pose2D computeError(Pose2D current,
                            Pose2D desired);
        
        /**
         * Returns the Euclidean norm of a set of pose states.
         * 
         * @param pose  Pose.
         * @return Euclidean norm of pose states.
         */
        float computeNorm(Pose2D pose);
        
        /**
         * Wraps angle to specified range.  
         * 
         * @param angle Angle to be wrapped.
         * @param lower Lower constraint (default -pi).
         * @param upper Upper constraint (default pi).
         * @return Angle wrapped to range [lower, upper].
         */
        float wrapAngle(float angle,
                        float lower = -M_PI,
                        float upper = M_PI);

};

#endif // APC_h
