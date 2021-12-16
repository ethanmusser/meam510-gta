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
         float yOffset)
    : _base(&base),
      _frontVive(&vive),
      _offsets{xOffset, yOffset, 0.0},
      _hasRearVive(false)
{}

APC::APC(MecanumBase& base,
         Vive510& frontVive, 
         Vive510& rearVive,
         float xOffset,
         float yOffset,
         float qOffset)
    : _base(&base),
      _frontVive(&frontVive),
      _rearVive(&rearVive),
      _offsets{xOffset, yOffset, qOffset},
      _hasRearVive(true)
{}

void APC::enable()
{
    // TODO: Implement
}

void APC::disable()
{
    // TODO: Implement
}

void APC::setDestination(Pose2D pose)
{
    // TODO: Implement
}

void APC::setDestination(float x,
                         float y,
                         float theta)
{
    // TODO: Implement
}

void APC::update()
{
    // TODO: Implement
}

void APC::computePose()
{
    // Get Vive Pose(s)
    Pose2D frontVivePose;
    frontVivePose.x = ((float) frontVive.xCoord()) / 1.0e3;
    frontVivePose.y = ((float) frontVive.yCoord()) / 1.0e3;
    if(_hasRearVive) {
        Pose2D rearVivePose;
        frontVivePose.x = ((float) frontVive.xCoord()) / 1.0e3;
        frontVivePose.y = ((float) frontVive.yCoord()) / 1.0e3;
    }

    // Compute Base Pose
    _currentPose.x = frontVivePose.x - _offsets.x;
    _currentPose.y = frontVivePose.y - _offsets.y;
    if(_hasRearVive) {
        Pose2D viveDiff = computeError(frontVivePose, rearVivePose);
        float angle = atan2(viveDiff.y, viveDiff.x);
        _currentPose.theta = angle >= 0.0 ? angle : angle + 2.0 * M_PI;
    }
}

Pose2D APC::computeError(Pose2D currentPose,
                         Pose2D desiredPose)
{
    // Constrain Inputs
    currentPose.theta = wrapAngle(currentPose.theta);
    desiredPose.theta = wrapAngle(desiredPose.theta);

    // Compue Error
    Pose2D error;
    error.x = currentPose.x - desiredPose.x;
    error.y = currentPose.y - desiredPose.y;
    error.theta = currentPose.theta - desiredPose.theta;
}

Twist2D APC::computeControl()
{
    // TODO: Implement
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

