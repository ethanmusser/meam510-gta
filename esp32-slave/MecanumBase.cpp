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

