#line 1 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\MotorController.cpp"
/**
 * MotorController.cpp
 * Source for MotorController class.
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#include "MotorController.h"
#include <analogWrite.h>  // analogWrite via LEDC

MotorController::MotorController(int enablingPin, 
                                 unsigned int directionPin)
    : _enablingPin(enablingPin),
      _directionPin{directionPin, 0},
      _isEnablingDrive(enablingPin >= 0 ? true : false),
      _isDualDirection(false),
      _hasEncoder(false)
{}

MotorController::MotorController(int enablingPin, 
                                 unsigned int directionPin,
                                 Encoder& encoder)
    : _enablingPin(enablingPin),
      _directionPin{directionPin, 0},
      _encoder(&encoder),
      _isEnablingDrive(enablingPin >= 0 ? true : false),
      _isDualDirection(false),
      _hasEncoder(true)
{}

MotorController::MotorController(int enablingPin, 
                                 unsigned int forwardDirectionPin,
                                 unsigned int reverseDirectionPin)
    : _enablingPin(enablingPin),
      _directionPin{forwardDirectionPin, reverseDirectionPin},
      _isEnablingDrive(enablingPin >= 0 ? true : false),
      _isDualDirection(true),
      _hasEncoder(false)
{}

MotorController::MotorController(int enablingPin, 
                                 unsigned int forwardDirectionPin,
                                 unsigned int reverseDirectionPin,
                                 Encoder& encoder)
    : _enablingPin(enablingPin),
      _directionPin{forwardDirectionPin, reverseDirectionPin},
      _encoder(&encoder),
      _isEnablingDrive(enablingPin >= 0 ? true : false),
      _isDualDirection(true),
      _hasEncoder(true)
{}

void MotorController::setup()
{
    // Pin Modes
    pinMode(_enablingPin, OUTPUT);
    pinMode(_directionPin[0], OUTPUT);
    if(_isDualDirection) {
        pinMode(_directionPin[1], OUTPUT);
    }

    // Initialization
    set(0.0);
}

void MotorController::set(float speed)
{
    // Constrain Inputs
    _speed = constrain(speed, -1.0, 1.0);

    // Set Duty Cycle & Direction
    if(_speed > 0) {
        setDuty(abs(_speed), forward);
    } else if(_speed < 0) {
        setDuty(abs(_speed), backward);
    } else {
        setDuty(abs(_speed), braking);
    } 
}

void MotorController::adjust(float increment)
{
    if(increment >= 0) {
        setDuty(_dutyCycle + increment, _direction);
    } else {
        setDuty(_dutyCycle - increment, _direction);
    }
}

void MotorController::stop()
{
    setDuty(0.0, freeSpin);
}

void MotorController::brake()
{
    setDuty(0.0, braking);
}

void MotorController::setDuty(float duty, 
                              MotorDirection direction)
{
    switch(direction) {
        case forward:
            setDutyForward(duty);
            break;
        case backward:
            setDutyBackward(duty);
            break;
        case freeSpin:
            setDutyFree(duty);
            break;
        case braking:
            setDutyBrake(duty);
            break;
    }
}

void MotorController::setDutyForward(float duty)
{
    // Constrain Inputs
    _dutyCycle = constrain(duty, 0.0, 1.0);

    // Set Pins
    if(_isEnablingDrive && _isDualDirection) {
        analogWrite(_enablingPin, (unsigned int) 255 * _dutyCycle);
        digitalWrite(_directionPin[0], HIGH);
        digitalWrite(_directionPin[1], LOW);
    } else if(!_isEnablingDrive && _isDualDirection) {
        analogWrite(_directionPin[0], (unsigned int) 255 * _dutyCycle);
        analogWrite(_directionPin[1], 0);
    } else if(_isEnablingDrive && !_isDualDirection) {
        analogWrite(_enablingPin, (unsigned int) 255 * _dutyCycle);
        digitalWrite(_directionPin[0], HIGH);
    } else { // if(!_isEnablingDrive && !_isDualDirection)
        analogWrite(_directionPin[0], (unsigned int) 255 * _dutyCycle);
    }
    _direction = forward;
}

void MotorController::setDutyBackward(float duty)
{
    // Constrain Inputs
    _dutyCycle = constrain(duty, 0.0, 1.0);

    // Set Pins
    if(_isEnablingDrive && _isDualDirection) {
        analogWrite(_enablingPin, (unsigned int) 255 * _dutyCycle);
        digitalWrite(_directionPin[0], LOW);
        digitalWrite(_directionPin[1], HIGH);
    } else if(!_isEnablingDrive && _isDualDirection) {
        analogWrite(_directionPin[0], 0);
        analogWrite(_directionPin[1], (unsigned int) 255 * _dutyCycle);
    } else if(_isEnablingDrive && !_isDualDirection) {
        analogWrite(_enablingPin, (unsigned int) 255 * _dutyCycle);
        digitalWrite(_directionPin[0], LOW);
    } else { // if(!_isEnablingDrive && !_isDualDirection)
        analogWrite(_directionPin[0], (unsigned int) 255 * _dutyCycle);
    }
    _direction = backward;
}

void MotorController::setDutyFree(float duty) 
{
    // Constrain Inputs
    _dutyCycle = constrain(duty, 0.0, 1.0);

    // Set Pins
    if(_isEnablingDrive && _isDualDirection) {
        analogWrite(_enablingPin, 0);
        digitalWrite(_directionPin[0], LOW);
        digitalWrite(_directionPin[1], LOW);
    } else if(!_isEnablingDrive && _isDualDirection) {
        analogWrite(_directionPin[0], 0);
        analogWrite(_directionPin[1], 0);
    } else if(_isEnablingDrive && !_isDualDirection) {
        setDuty(0.0, _direction);
    } else { // if(!_isEnablingDrive && !_isDualDirection)
        analogWrite(_directionPin[0], 0);
    }
    _direction = freeSpin;
}

void MotorController::setDutyBrake(float duty) 
{
    // Constrain Inputs
    _dutyCycle = constrain(duty, 0.0, 1.0);

    // Set Pins
    if(_isEnablingDrive && _isDualDirection) {
        analogWrite(_enablingPin, 255);
        digitalWrite(_directionPin[0], HIGH);
        digitalWrite(_directionPin[1], HIGH);
    } else if(!_isEnablingDrive && _isDualDirection) {
        analogWrite(_directionPin[0], 255);
        analogWrite(_directionPin[1], 255);
    } else if(_isEnablingDrive && !_isDualDirection) {
        if(_direction == forward) {
            setDuty(0.0, backward);
        } else if(_direction == backward) {
            setDuty(0.0, forward);
        } else {
            analogWrite(_enablingPin, 0);
        }
    } else { // if(!_isEnablingDrive && !_isDualDirection)
        analogWrite(_directionPin[0], 0);
    }
    _direction = braking;
}
