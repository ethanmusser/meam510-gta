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
      _isDualDirection(false),
      _hasEncoder(false)
{}

MotorController::MotorController(unsigned int enablingPin, 
                                 unsigned int directionPin,
                                 Encoder& encoder)
    : _enablingPin(enablingPin),
      _directionPin{directionPin, 0},
      _encoder(&encoder),
      _isDualDirection(false),
      _hasEncoder(true)
{}

MotorController::MotorController(unsigned int enablingPin, 
                                 unsigned int forwardDirectionPin,
                                 unsigned int reverseDirectionPin)
    : _enablingPin(enablingPin),
      _directionPin{forwardDirectionPin, reverseDirectionPin},
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
      _isDualDirection(true),
      _hasEncoder(true)
{}

void MotorController::setup()
{
    // Pin Modes
    pinMode(_enablingPin, OUTPUT);
    pinMode(_directionPin[0], OUTPUT);
    if(_isDualDirection) pinMode(_directionPin[1], OUTPUT);

    // Initialization
    set(0.0);
}

void MotorController::setDutyCycle(float duty)
{
    _dutyCycle = constrain(duty, 0.0, 1.0);
    analogWrite(_enablingPin, (unsigned int) 255 * _dutyCycle);
}

void MotorController::setDirection(MotorDirection direction)
{
    if(direction == forward) {
        if(_isDualDirection) {
            digitalWrite(_directionPin[0], HIGH);
            digitalWrite(_directionPin[1], LOW);
        } else {
            digitalWrite(_directionPin[0], HIGH);
        }
        _direction = forward;
    } else if(direction == backward) {
        if(_isDualDirection) {
            digitalWrite(_directionPin[0], HIGH);
            digitalWrite(_directionPin[1], LOW);
        } else {
            digitalWrite(_directionPin[0], LOW);
        }
        _direction = backward;
    } else if(direction == freeSpin) {
        if(_isDualDirection) {
            digitalWrite(_directionPin[0], LOW);
            digitalWrite(_directionPin[1], LOW);
        } else {
            setDirection(forward);
        }
        _direction = freeSpin;
    } else {// if(direction == braking) 
        if(_isDualDirection) {
            digitalWrite(_directionPin[0], HIGH);
            digitalWrite(_directionPin[1], HIGH);
        } else {
            if(_direction == forward) {
                setDirection(backward);
            } else {
                setDirection(forward);
            }
        }
        _direction = braking;
    }
}
