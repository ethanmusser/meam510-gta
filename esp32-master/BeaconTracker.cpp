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
      _setFrequencies{lowFrequency, highFrequency},
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
        findBeacon();
    }
}

void BeaconTracker::setTarget(float target)
{
    _target = target;
    // if(isClose(target, 23.0, 0.5)) {
    //     digitalWrite(4, LOW);
    // } else {
    //     digitalWrite(4, HIGH);
    // }
}

void BeaconTracker::findBeacon()
{
    // Serial.print("freqs = "); Serial.print(frequency[0]); Serial.print(", "); Serial.println(frequency[1]);
    // Serial.print("Left = "); Serial.print(frequency[0]);
    // Serial.print(", Right = "); Serial.println(frequency[1]);
    // Serial.print(", Target = "); Serial.print(_target);
    // Serial.print(", Response = "); Serial.println(digitalRead(30));
    bool f0 = isClose(frequency[0], _target, 20.0);
    // bool f1 = isClose(frequency[1], _target, 20.0);
    // if(f0 && f1) {
    //     _state = approaching;
    //     _base->driveCartesian(_driveSpeed, 0.0, 0.0);
    // } else if((f0 && !f1) || (!f0 && !f1)) {
    //     _state = searching;
    //     _base->driveCartesian(0.0, 0.0, _rotateSpeed);
    // } else if(!f0 && f1) {
    //     _state = searching;
    //     _base->driveCartesian(0.0, 0.0, -_rotateSpeed);
    // }
    if (f0){
        _base->driveCartesian(_driveSpeed, 0.0, 0.0);
    } else {
        _base->driveCartesian(0.0, 0.0, _rotateSpeed);
    }
}

bool BeaconTracker::isClose(float a, float b, float epsilon) 
{
    return abs(a - b) < abs(epsilon);
}

void BeaconTracker::setFrequency(float freq1, float freq2)
{
    frequency[0] = freq1;
    frequency[1] = freq2;
    // frequency[1] = digitalRead(30) == HIGH ? _target : 9999.0;
}


// int BeaconTracker::checkPeriod(unsigned int ch)
// {
//     static unsigned int prevRead[NUM_BEACONS];
//     static unsigned int prevTime[NUM_BEACONS];

//     int on_off;
//     unsigned int newRead = digitalRead(_irPins[ch]);
//     Serial.print("newRead = "); Serial.println(newRead);
//     if(newRead != prevRead[ch]) {
//         int period = 2 * (microseconds - prevTime[ch]);
//         Serial.print("period = "); Serial.println(period);
//         Serial.print("microseconds = "); Serial.println(microseconds);
//         Serial.print("prevTime = "); Serial.println(prevTime[ch]);
//         if(period > _desPeriod[_target] - _noise[_target] 
//            && period < _desPeriod[_target] + _noise[_target]) {
//             on_off = 1;
//         } else {
//             on_off = 0;
//         }
//     } else {
//         on_off = 0;
//     }
//     prevRead[ch] = newRead;
//     prevTime[ch] = microseconds;
//     return on_off;
// }



