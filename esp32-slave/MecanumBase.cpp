/**
 * MecanumBase.cpp
 * Source for MecanumBase class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#include "MecanumBase.h"

MecanumBase::MecanumBase(MotorController& frontLeftMotor,
                         MotorController& frontRightMotor,
                         MotorController& rearLeftMotor,
                         MotorController& rearRightMotor)
    : _frontLeftMotor(&frontLeftMotor),
      _frontRightMotor(&frontRightMotor),
      _rearLeftMotor(&rearLeftMotor),
      _rearRightMotor(&rearRightMotor)
{}

void MecanumBase::driveCartesian(float xSpeed, 
                                 float ySpeed,
                                 float zRotation)
{
    // Clamp Inputs
    xSpeed = constrain(xSpeed, -1.0, 1.0);
    ySpeed = constrain(ySpeed, -1.0, 1.0);
    zRotation = constrain(zRotation, -1.0, 1.0);

    // Call Polar Drive to Set Wheel Speeds
    float magnitude = sqrt(pow(xSpeed, 2) + pow(ySpeed, 2));
    float angle = atan2(ySpeed, xSpeed);
    drivePolar(magnitude, angle, zRotation);
}

void MecanumBase::drivePolar(float magnitude, 
                             float angle,
                             float zRotation)
{
    // Clamp Inputs
    magnitude = constrain(magnitude, -1.0, 1.0);
    // angle = constrain(angle, 0.0, 2.0*M_PI);
    zRotation = constrain(zRotation, -1.0, 1.0);
    
    // Compute Wheel Speeds
    float trigEntry = angle + M_PI_4;
    WheelSpeeds wheelSpeeds;
    wheelSpeeds.frontLeftSpeed  = magnitude * sin(trigEntry) + zRotation;
    wheelSpeeds.frontRightSpeed = magnitude * cos(trigEntry) - zRotation;
    wheelSpeeds.rearLeftSpeed   = magnitude * cos(trigEntry) + zRotation;
    wheelSpeeds.rearRightSpeed  = magnitude * sin(trigEntry) - zRotation;

    // Set Wheel Speeds
    setWheelSpeeds(wheelSpeeds);
}

void MecanumBase::stop()
{
    _frontLeftMotor->stop();
    _frontRightMotor->stop();
    _rearLeftMotor->stop();
    _rearRightMotor->stop();
}

void MecanumBase::brake()
{
    _frontLeftMotor->brake();
    _frontRightMotor->brake();
    _rearLeftMotor->brake();
    _rearRightMotor->brake();
}

void MecanumBase::setWheelSpeeds(WheelSpeeds speeds)
{
    _wheelSpeeds = speeds;
    _frontLeftMotor->set(_wheelSpeeds.frontLeftSpeed);
    _frontRightMotor->set(_wheelSpeeds.frontRightSpeed);
    _rearLeftMotor->set(_wheelSpeeds.rearLeftSpeed);
    _rearRightMotor->set(_wheelSpeeds.rearRightSpeed);
}
