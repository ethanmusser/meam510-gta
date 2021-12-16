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

APC::APC(MecanumBase& base,
         Vive510& vive,
         float xOffset,
         float yOffset)
{
    // TODO: Implement
}

APC::APC(MecanumBase& base,
         Vive510& frontVive, 
         Vive510& rearVive,
         float xOffset,
         float yOffset)
{
    // TODO: Implement
}

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

void APC::computePose()
{
    // TODO: Implement
}

Pose2D APC::computeError(Pose2D currentPose,
                         Pose2D desiredPose)
{
    // TODO: Implement
}

Twist2D APC::computeControl()
{
    // TODO: Implement
}

