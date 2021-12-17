/**
 * APC.cpp
 * Source for absolute position control (APC) class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#include "APC.h"
#include <math.h>

APC::APC(MecanumBase& base,
         Vive510& vive,
         float xOffset,
         float yOffset,
         float kp,
         float ki,
         float kd,
         float epsilon)
    : _base(&base),
      _frontVive(&vive),
      _offsets{xOffset, yOffset, 0.0},
      _gains{kp, ki, kd},
      _epsilon(epsilon),
      _hasRearVive(false)
{}

APC::APC(MecanumBase& base,
         Vive510& frontVive, 
         Vive510& rearVive,
         float xOffset,
         float yOffset,
         float qOffset,
         float kp,
         float ki,
         float kd,
         float epsilon)
    : _base(&base),
      _frontVive(&frontVive),
      _rearVive(&rearVive),
      _offsets{xOffset, yOffset, qOffset},
      _gains{kp, ki, kd},
      _epsilon(epsilon),
      _hasRearVive(true)
{}

void APC::setDestination(Pose2D pose)
{
    _isEnabled = true;
    _desiredPose = pose;
}

void APC::setDestination(float x,
                         float y,
                         float theta)
{
    _isEnabled = true;
    _desiredPose.x = x;
    _desiredPose.y = y;
    _desiredPose.theta = theta;
}

void APC::setGains(float kp,
                   float ki,
                   float kd)
{
    _gains.kp = kp;
    _gains.ki = ki;
    _gains.kd = kd;
}

void APC::setEpsilon(float epsilon)
{
    _epsilon = epsilon;
}

void APC::update()
{
    computePose();
    if(_isEnabled){
        if( computeNorm(_currentPose) <= _epsilon ) {
            disable();
        } else {
            // Compute Control Input
            Twist2D control = computeControl(_currentPose, _currentTwist, 
                    _desiredPose, _gains);

            // Drive Base
            _base->driveCartesian(control.vx, control.vy, control.vtheta);
        }
    }
}

void APC::disable()
{
    _isEnabled = false;
    _base->brake();
}

void APC::computePose()
{
    // Get Vive Pose(s)
    Pose2D frontVivePose, rearVivePose;
    frontVivePose.x = ((float) _frontVive->xCoord()) / 1.0e3;
    frontVivePose.y = ((float) _frontVive->yCoord()) / 1.0e3;
    if(_hasRearVive) {
        rearVivePose.x = ((float) _rearVive->xCoord()) / 1.0e3;
        rearVivePose.y = ((float) _rearVive->yCoord()) / 1.0e3;
    }

    // Compute Base Pose
    _previousPose = _currentPose;
    _currentPose.x = frontVivePose.x - _offsets.x;
    _currentPose.y = frontVivePose.y - _offsets.y;
    if(_hasRearVive) {
        Pose2D viveDiff = computeError(frontVivePose, rearVivePose);
        float angle = atan2(viveDiff.y, viveDiff.x);
        _currentPose.theta = angle >= 0.0 ? angle : angle + 2.0 * M_PI;
    }

    // Compute Base Twist
    long currentTime = millis();
    float dt = (float) (currentTime - _previousUpdateTime);
    _previousUpdateTime = currentTime;
    _currentTwist.vx = (_currentPose.x - _previousPose.x) / dt;
    _currentTwist.vy = (_currentPose.y - _previousPose.y) / dt;
    _currentTwist.vtheta = (_currentPose.theta - _previousPose.theta) / dt;
}

Twist2D APC::computeControl(Pose2D currentPose, 
                            Twist2D currentTwist,
                            Pose2D desiredPose, 
                            Gains gains)
{
    // Compute Error
    Pose2D error = computeError(currentPose, desiredPose);

    // Compute Control Input
    Twist2D control;
    control.vx = gains.kp * (currentPose.x - desiredPose.x) 
            + gains.kd * currentTwist.vx;
    control.vy = gains.kp * (currentPose.y - desiredPose.y)
            + gains.kd * currentTwist.vy;
    control.vtheta = gains.kp * (currentPose.theta - desiredPose.theta)
            + gains.kd * currentTwist.vtheta;
    return control;
}

Pose2D APC::computeError(Pose2D current,
                         Pose2D desired)
{
    // Constrain Inputs
    current.theta = wrapAngle(current.theta);
    desired.theta = wrapAngle(desired.theta);

    // Compue Error
    Pose2D error;
    error.x = current.x - desired.x;
    error.y = current.y - desired.y;
    error.theta = current.theta - desired.theta;
    return error;
}

float APC::computeNorm(Pose2D pose)
{
    return sqrt(pow(pose.x, 2) + pow(pose.y, 2) + pow(pose.theta, 2));
}

float APC::wrapAngle(float angle,
                     float lower,
                     float upper)
{
    float range = upper - lower; // https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
    angle = fmod(angle - lower, range);
    if(angle < 0.0) {
        angle += range;
    }
    return angle - lower;
}

