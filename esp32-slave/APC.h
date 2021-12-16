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
#include "vive510.h"  // motor controllers

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

/**
 * Absolute positioning controller (APC) class.
 */
class APC 
{
    public:
        Pose2D _currentPose = { 0.0, 0.0, 0.0 };
        Pose2D _desiredPose = { 0.0, 0.0, 0.0 };

        /**
         * Constructor for APC with only position sensing.
         * 
         * @param base      Mobile base.
         * @param vive      Vive sensor mounted to mobile base.
         * @param xOffset   Origin offset from vive x-position.
         * @param yOffset   Origin offset from vive y-position.
         */
        APC(MecanumBase& base,
            Vive510& vive,
            float xOffset = 0.0,
            float yOffset = 0.0);
        
        /**
         * Constructor for APC with position and orientation.
         * 
         * @param base      Mobile base.
         * @param frontVive Front vive sensor mounted to mobile base.
         * @param rearVive  Rear vive sensor mounted to mobile base.
         * @param xOffset   Origin offset from front vive x-position.
         * @param yOffset   Origin offset from front vive y-position.
         */
        APC(MecanumBase& base,
            Vive510& frontVive, 
            Vive510& rearVive,
            float xOffset = 0.0,
            float yOffset = 0.0);

        /**
         * Enables absolute position control.
         */
        void enable();
        
        /**
         * Disables absolute position control.
         */
        void disable();
        
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
         * Updates controller 
         */
        void update();
        
    protected:

    private:
        Vive510* _frontVive;
        Vive510* _rearVive;

        /**
         * Computes current pose from vive sensor(s).
         */
        void computePose();

        /**
         * Returns the error between two poses.
         * 
         * @param currentPose   Current pose.
         * @param desiredPose   Desired pose.
         * @return Pose2D of error between current and desired poses.
         */
        Pose2D computeError(Pose2D currentPose,
                            Pose2D desiredPose);
        
        /**
         * Compute control input to base.
         */
        Twist2D computeControl();

};

#endif // APC_h
