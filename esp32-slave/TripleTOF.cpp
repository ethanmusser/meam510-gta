/**
 * TripleTOF.cpp
 * Source for triple time of flight sensor class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#include "TripleTOF.h"

TripleTOF::TripleTOF(Adafruit_VL53L0X& lox1,
                     unsigned int lox1Address,
                     unsigned int lox1Shutdown,
                     Adafruit_VL53L0X& lox2,
                     unsigned int lox2Address,
                     unsigned int lox2Shutdown,
                     Adafruit_VL53L0X& lox3,
                     unsigned int lox3Address,
                     unsigned int lox3Shutdown)
    : _lox{&lox1, &lox2, &lox3},
      _loxAddresses{lox1Address, lox2Address, lox3Address},
      _loxShutdowns{lox1Shutdown, lox2Shutdown, lox3Shutdown}
{}

void TripleTOF::begin()
{
    // Pin Initialization
    for (int i = 0; i < NUM_LOX; i++) {
        pinMode(_loxShutdowns[i], OUTPUT);
        digitalWrite(_loxShutdowns[i], LOW);
    }

    // Setup IDs
    setID();
}

void TripleTOF::readSensors()
{
    for (int i = 0; i < NUM_LOX; i++) {
        _lox[i]->rangingTest(&_measures[i], false);
        if(_measures[i].RangeStatus != 4) {
            _rangeMillis[i] = _measures[i].RangeMilliMeter;
        } else { // Out of Range
            _rangeMillis[i] = OUT_OF_RANGE;
        }
    }
}

TripleRange TripleTOF::getRanges()
{
    readSensors();
    TripleRange ranges = {_rangeMillis[0], _rangeMillis[1], _rangeMillis[2]};
    return ranges;
}

void TripleTOF::setID()
{
    // All Reset
    for (int i = 0; i < NUM_LOX; i++) {
        digitalWrite(_loxShutdowns[i], LOW);
    }
    delay(10);

    // Unreset and Activate Individually
    for (int i = 0; i < NUM_LOX; i++) {
        // Activate
        digitalWrite(_loxShutdowns[i], HIGH);
        delay(10);
        // Initialize
        if(!_lox[i]->begin(_loxAddresses[i])) {
            Serial.printf("Failed to boot VL53L0X %d\n", i+1);
            // while(1) ;
        }
        delay(10);
    }
}

