#line 1 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\APC.h"
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

// struct Pose2D {
//     float x;
//     float y;
//     float theta;
// };

// struct Twist2D {
//     float vx;
//     float vy;
//     float vtheta;
// };

struct Gains {
    float kp;
    float ki;
    float kd;
};

struct Epsilons {
    float epsilonT;
    float epsilonR;
};

/**
 * Absolute positioning controller (APC) class.
 */
class APC 
{
    public:
        Gains _transGains;
        Gains _rotGains;
        Epsilons _epsilons;
        Pose2D _currentPose;
        Twist2D _currentTwist;
        Pose2D _desiredPose;
        Pose2D _previousPose;
        Pose2D _frontVivePose;
        Pose2D _rearVivePose;
        unsigned long _previousUpdateTime;

        /**
         * Constructor for APC with only position sensing.
         * 
         * @param base      Mobile base.
         * @param vive      Vive sensor mounted to mobile base.
         * @param xOffset   Origin offset from vive x-position in meters.
         * @param yOffset   Origin offset from vive y-position in meters.
         * @param kpTrans   Translational proportional gain.
         * @param kdTrans   Translational derivative gain.
         * @param epsilonT  Allowable norm translational state error.
         */
        APC(MecanumBase& base,
            Vive510& vive,
            float xOffset = 0.0,
            float yOffset = 0.0,
            float kpTrans = 3.0,
            float kdTrans = 1.0,
            float epsilonT = 0.05);
        
        /**
         * Constructor for APC with position and orientation.
         * 
         * @param base      Mobile base.
         * @param frontVive Front vive sensor mounted to mobile base.
         * @param rearVive  Rear vive sensor mounted to mobile base.
         * @param xOffset   Origin offset from front vive x-position in meters.
         * @param yOffset   Origin offset from front vive y-position in meters.
         * @param qOffset   Angular offset from vive-to-vive axis in radians.
         * @param kpTrans   Translational proportional gain.
         * @param kdTrans   Translational derivative gain.
         * @param kpRot     Rotational proportional gain.
         * @param kdRot     Rotational derivative gain.
         * @param epsilonT  Allowable norm translational state error.
         * @param epsilonR  Allowable rotational state error.
         */
        APC(MecanumBase& base,
            Vive510& frontVive, 
            Vive510& rearVive,
            float xOffset = 0.0,
            float yOffset = 0.0,
            float qOffset = 0.0,
            float kpTrans = 3.0,
            float kdTrans = 1.0,
            float kpRot = 0.6,
            float kdRot = 0.2,
            float epsilonT = 0.05,
            float epsilonR = 0.2);

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
         * @param kpTrans   Translational proportional gain.
         * @param kdTrans   Translational derivative gain.
         * @param kpRot     Rotational proportional gain.
         * @param kdRot     Rotational derivative gain.
         */
        void setGains(float kpTrans = 1.0,
                      float kdTrans = 0.2,
                      float kpRot = 0.7,
                      float kdRot = 0.15);
        /**
         * Sets APC control gains for translational motion.
         * 
         * @param kp    Proportional gain.
         * @param kd    Derivative gain.
         */
        void setTranslationalGains(float kp = 1.0,
                                   float kd = 0.2);

        /**
         * Sets APC control gains for rotational motion.
         * 
         * @param kp    Proportional gain.
         * @param kd    Derivative gain.
         */
        void setRotationalGains(float kp = 1.0,
                                float kd = 0.2);

        /**
         * Set error tolerances.
         * 
         * @param epsilonT   Error tolerance in meters.
         */
        void setEpsilons(float epsilonT = 0.02,
                         float epsilonR = 0.17);

        /**
         * Updates controller 
         */
        void update();
        
        /**
         * Enables absolute position control.
         */
        void enable();
        
        /**
         * Enables translational absolute position control.
         */
        void enableTranslation();
        
        /**
         * Enables rotational absolute position control.
         */
        void enableRotation();
        
        /**
         * Disables absolute position control.
         */
        void disable();

        /**
         * Disables translational absolute position control.
         */
        void disableTranslation();
        
        /**
         * Disables rotational absolute position control.
         */
        void disableRotation();
        
        /**
         * Computes and returns the current base pose.
         * 
         * @return Base pose as a Pose2D.
         */
        Pose2D getPose();
        
        /**
         * Computes and returns the current base twist.
         * 
         * @return Base pose as a Twist2D.
         */
        Twist2D getTwist();
        
    protected:

    private:
        MecanumBase* _base;
        Vive510* _frontVive;
        Vive510* _rearVive;
        Pose2D _offsets;
        bool _hasRearVive;
        bool _isEnabled;
        bool _transControlEnabled;
        bool _rotControlEnabled;

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
         * @param transGains    Translational contorller gains.
         * @param rotGains      Rotational contorller gains.
         * @return              Control twist.
         */
        Twist2D computeBaseControl(Pose2D currentPose,
                                   Twist2D currentTwist,
                                   Pose2D desiredPose,
                                   Gains transGains,
                                   Gains rotGains);

        /**
         * Compute control input to base.
         * 
         * @param currentPosition   Current position.
         * @param currentVelocity   Current velocity.
         * @param desiredPosition   Desired position.
         * @param gains             Controller gains.
         * @return                  Control velocity.
         */
        float computeControl(float currentPosition,
                             float currentVelocity,
                             float desiredPosition,
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
        float computeTotalNorm(Pose2D pose);
        
        /**
         * Returns the Euclidean norm of a set of pose translational states.
         * 
         * @param pose  Pose.
         * @return Euclidean norm of pose translational states.
         */
        float computeTranslationalNorm(Pose2D pose);
        
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
