/**
 * MecanumBase.h
 * Header for MecanumBase class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#ifndef MECANUMBASE_h
#define MECANUMBASE_h

#include <Arduino.h>  // arduino commands
#include "MotorController.h"  // motor controllers

#define FRONTLEFT 0
#define FRONTRIGHT 1
#define REARLEFT 2
#define REARRIGHT 3

struct WheelSpeeds {
    float frontLeftSpeed;
    float frontRightSpeed;
    float rearLeftSpeed;
    float rearRightSpeed;
};

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
 * Mecanum X-drive mobile base.
 * 
 *  \ ______ /
 *  \  |  |  /
 *     |  |  
 *  / _|__|_ \
 *  /        \
 */
class MecanumBase 
{
    public:
        WheelSpeeds _wheelSpeeds = { 0.0, 0.0, 0.0, 0.0 };

        /**
         * Constructor for MecanumBase.
         * 
         * @param frontLeftMotor    Controller for front-left wheel motor.
         * @param frontRightMotor   Controller for front-right wheel motor.
         * @param rearLeftMotor     Controller for rear-left wheel motor.
         * @param rearRightMotor    Controller for rear-right wheel motor.
         */
        MecanumBase(MotorController& frontLeftMotor,
                    MotorController& frontRightMotor,
                    MotorController& rearLeftMotor,
                    MotorController& rearRightMotor);
        
        /**
         * Drive base using cartesion directions.
         * 
         * @param xSpeed    Desired fore-aft speed in range [-1.0, 1.0].  
         *                  Forward is positive.
         * @param ySpeed    Desired strafe speed in range [-1.0, 1.0]. Right is
         *                  positive.
         * @param zRotation Desired rotation speed in range [-1.0, 1.0]. CCW is
         *                  positive.
         */
        void driveCartesian(float xSpeed, 
                            float ySpeed,
                            float zRotation);

        /**
         * Drive base using polar directions.
         * 
         * @param magnitude Desired speed in range [-1.0, 1.0]. Direction angle
         *                  is positive.
         * @param angle     Desired direction angle in range [0, 2*pi]. CCW 
         *                  is positive.  Zero corresponds to forward.
         * @param zRotation Desired rotation speed in range [-1.0, 1.0]. CCW is 
         *                  positive.
         */
        void drivePolar(float magnitude, 
                        float angle,
                        float zRotation);

        /**
         * Stops base without braking.
         */
        void stop();

        /**
         * Stops base with braking.
         */
        void brake();

    protected:

    private:
        MotorController* _frontLeftMotor;
        MotorController* _frontRightMotor;
        MotorController* _rearLeftMotor;
        MotorController* _rearRightMotor;

        /**
         * Set base wheel speeds.
         * 
         * @param speeds    WheelSpeeds struct containing desired wheel speeds.
         */
        void setWheelSpeeds(WheelSpeeds speeds);

};

#endif // MECANUMBASE_h
