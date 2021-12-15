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

MotorController::MotorController(unsigned int enablingPin, 
                                 unsigned int directionPin)
    : _enablingPin(enablingPin),
      _directionPin{directionPin, 0},
      _isEnablingDrive(enablingPin >= 0 ? true : false),
      _isDualDirection(false),
      _hasEncoder(false)
{}

MotorController::MotorController(unsigned int enablingPin, 
                                 unsigned int directionPin,
                                 Encoder& encoder)
    : _enablingPin(enablingPin),
      _directionPin{directionPin, 0},
      _encoder(&encoder),
      _isEnablingDrive(enablingPin >= 0 ? true : false),
      _isDualDirection(false),
      _hasEncoder(true)
{}

MotorController::MotorController(unsigned int enablingPin, 
                                 unsigned int forwardDirectionPin,
                                 unsigned int reverseDirectionPin)
    : _enablingPin(enablingPin),
      _directionPin{forwardDirectionPin, reverseDirectionPin},
      _isEnablingDrive(enablingPin >= 0 ? true : false),
      _isDualDirection(true),
      _hasEncoder(false)
{}

MotorController::MotorController(unsigned int enablingPin, 
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
    if(speed > 0) {
        setDutyCycleAndDirection(abs(speed), forward);
    } else if(speed < 0) {
        setDutyCycleAndDirection(abs(speed), backward);
    } else {
        setDutyCycleAndDirection(abs(speed), braking);
    } 
}

void MotorController::adjust(float increment)
{
    if(increment >= 0) {
        setDutyCycleAndDirection(_dutyCycle + increment, _direction);
    } else {
        setDutyCycleAndDirection(_dutyCycle - increment, _direction);
    }
}

void MotorController::stop()
{
    setDutyCycleAndDirection(0.0, freeSpin);
}

void MotorController::brake()
{
    setDutyCycleAndDirection(0.0, braking);
}

void MotorController::setDutyCycleAndDirection(float duty, 
                                               MotorDirection direction)
{
    // Constrain Inputs
    _dutyCycle = constrain(duty, 0.0, 1.0);

    // Drive Pins
    switch(direction) {
        // Forward Case
        case forward:
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
            break;
        // Backward Case
        case backward:
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
            break;
        // Free-Spinning Case
        case freeSpin:
            if(_isEnablingDrive && _isDualDirection) {
                analogWrite(_enablingPin, 0);
                digitalWrite(_directionPin[0], LOW);
                digitalWrite(_directionPin[1], LOW);
            } else if(!_isEnablingDrive && _isDualDirection) {
                analogWrite(_directionPin[0], 0);
                analogWrite(_directionPin[1], 0);
            } else if(_isEnablingDrive && !_isDualDirection) {
                setDutyCycleAndDirection(0.0, _direction);
            } else { // if(!_isEnablingDrive && !_isDualDirection)
                analogWrite(_directionPin[0], 0);
            }
            _direction = freeSpin;
            break;
        // Braking Case
        case braking:
            if(_isEnablingDrive && _isDualDirection) {
                analogWrite(_enablingPin, 255);
                digitalWrite(_directionPin[0], HIGH);
                digitalWrite(_directionPin[1], HIGH);
            } else if(!_isEnablingDrive && _isDualDirection) {
                analogWrite(_directionPin[0], 255);
                analogWrite(_directionPin[1], 255);
            } else if(_isEnablingDrive && !_isDualDirection) {
                if(_direction == forward) {
                    setDutyCycleAndDirection(0.0, backward);
                } else if(direction == backward) {
                    setDutyCycleAndDirection(0.0, forward);
                } else {
                    analogWrite(_enablingPin, 0);
                }
            } else { // if(!_isEnablingDrive && !_isDualDirection)
                analogWrite(_directionPin[0], 0);
            }
            _direction = braking;
            break;
    }
}

