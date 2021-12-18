/**
 * BeaconTracker.h
 * Header for beacon tracker class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#include "BeaconTracker.h"

BeaconTracker::BeaconTracker(MecanumBase& base,
                             unsigned int leftIrPin,
                             unsigned int rightIrPin,
                             unsigned int lowFrequency,
                             unsigned int highFrequency,
                             unsigned int lowNoise,
                             unsigned int highNoise,
                             float driveSpeed,
                             float rotateSpeed)
    : _base(&base),
      _irPins{leftIrPin, rightIrPin},
      _frequencies{lowFrequency, highFrequency},
      _desPeriod{1000000/lowFrequency, 1000000/highFrequency},
      _noise{lowNoise, highNoise},
      _driveSpeed(driveSpeed),
      _rotateSpeed(rotateSpeed)
{}

void BeaconTracker::begin()
{
    for(int i = 0; i < NUM_BEACONS; i++) {
        pinMode(_irPins[i], INPUT);
    }
}

void BeaconTracker::enable()
{
    _state = searching;
}

void BeaconTracker::disable()
{
    _state = notTracking;
    _base->stop();
}

void BeaconTracker::update()
{
    if(_state != notTracking) {
        microseconds = micros();
        _period[0] = checkPeriod(0);
        _period[1] = checkPeriod(1);
        findBeacon();
    }
}

void BeaconTracker::setTarget(unsigned int target)
{
    _target = target;
}

void BeaconTracker::findBeacon()
{
    if(_period[0] == 1 && _period[1] == 1) {
        _state = approaching;
        _base->driveCartesian(_driveSpeed, 0.0, 0.0);
    } else if((_period[0] == 1 && _period[1] == 0) 
               || (_period[0] == 0 && _period[1] == 0)) {
        _state = searching;
        _base->driveCartesian(0.0, 0.0, -_rotateSpeed);
    } else if(_period[0] == 0 && _period[1] == 1) {
        _state = searching;
        _base->driveCartesian(0.0, 0.0, _rotateSpeed);
    }
}

int BeaconTracker::checkPeriod(unsigned int ch)
{
    static unsigned int prevRead[NUM_BEACONS];
    static unsigned int prevTime[NUM_BEACONS];

    int on_off;
    unsigned int newRead = digitalRead(_irPins[ch]);
    if(newRead != prevRead[ch]) {
        int period = 2 * (microseconds - prevTime[ch]);
        if(period > _desPeriod[_target] - _noise[_target] 
           && period < _desPeriod[_target] + _noise[_target]) {
            on_off = 1;
        } else {
            on_off = 0;
        }
    } else {
        on_off = 0;
    }
    prevRead[ch] = newRead;
    prevTime[ch] = microseconds;
    return on_off;
}


