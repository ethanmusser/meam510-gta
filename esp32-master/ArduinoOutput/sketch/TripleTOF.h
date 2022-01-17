#line 1 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\TripleTOF.h"
/**
 * TripleTOF.h
 * Header for triple time of flight sensor class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#ifndef TRIPLETOF_h
#define TRIPLETOF_h

#include <Arduino.h>  // arduino commands
#include <Adafruit_VL53L0X.h>  // time of flight sensor

#define NUM_LOX 3
#define OUT_OF_RANGE 99999

struct TripleRange {
    unsigned int r1;
    unsigned int r2;
    unsigned int r3;
};

/**
 * Triple time of flight sensor class.
 */
class TripleTOF 
{
    public:
        unsigned int _rangeMillis[NUM_LOX];

        /**
         * Constructor for time of flight class.
         * 
         * @param lox1          VL53L0X sensor.
         * @param lox1Address   Address to be assigned if multiple sensors.
         * @param lox1Shutdown  Shutdown pin.
         * @param lox2          VL53L0X sensor.
         * @param lox2Address   Address to be assigned if multiple sensors.
         * @param lox2Shutdown  Shutdown pin.
         * @param lox3          VL53L0X sensor.
         * @param lox3Address   Address to be assigned if multiple sensors.
         * @param lox3Shutdown  Shutdown pin.
         */
        TripleTOF(Adafruit_VL53L0X& lox1,
                  unsigned int lox1Address,
                  unsigned int lox1Shutdown,
                  Adafruit_VL53L0X& lox2,
                  unsigned int lox2Address,
                  unsigned int lox2Shutdown,
                  Adafruit_VL53L0X& lox3,
                  unsigned int lox3Address,
                  unsigned int lox3Shutdown);

        /**
         * Startup function.
         */
        void begin();

        /**
         * Reads sensors.
         */
        void readSensors();

        /**
         * Reads and returns sensor ranges.
         * 
         * @return Structure of range values.
         */
        TripleRange getRanges();

    private:
        Adafruit_VL53L0X* _lox[NUM_LOX];
        VL53L0X_RangingMeasurementData_t _measures[NUM_LOX];
        unsigned int _loxAddresses[NUM_LOX];
        unsigned int _loxShutdowns[NUM_LOX];

        /**
         * Sets TOF sensor IDs.
         */
        void setID();

};

#endif  // TRIPLETOF_h
