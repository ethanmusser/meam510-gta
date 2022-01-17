/**
 * BeaconTracker.h
 * Header for beacon tracker class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan C. Donlon
 * @version 0.1
 */

#ifndef BEACONTRACKER_h
#define BEACONTRACKER_h

#include <Arduino.h>  // arduino commands
#include "MecanumBase.h"  // mecanum base

#define NUM_BEACONS 2

struct BeaconPose {
    unsigned int front;
    unsigned int right;
    float theta;
};

struct TrackerGains {
    float kpWall;
    float kpRot;
};

enum TrackerState { notTracking, searching, approaching };

/**
 * Beacon tracker class for use with split IR sensors on a mecanum base.
 */
class BeaconTracker 
{
    public:
        unsigned int _periods[NUM_BEACONS];
        TrackerState _state;
        unsigned long microseconds;
        volatile float frequency[2];

        /**
         * Constructor for beacon tracker class.
         * 
         * @param base          Mecanum base.
         * @param leftIrPin     Pin for left-hand IR detector.
         * @param rightIrPin    Pin for right-hand IR detector.
         * @param lowFrequency  Low beacon signal frequency.
         * @param highFrequency High beacon signal frequency.
         * @param lowNoise      Low beacon signal noise.
         * @param highNoise     High beacon signal noise.
         * @param driveSpeed    Beacon approach speed in range [0.0, 1.0].
         * @param rotateSpeed   Search rotation speed in range [0.0, 1.0].
         */
        BeaconTracker(MecanumBase& base,
                      unsigned int leftIrPin,
                      unsigned int rightIrPin,
                      unsigned int lowFrequency = 23,
                      unsigned int highFrequency = 700,
                      unsigned int lowNoise = 3000,
                      unsigned int highNoise = 1000,
                      float driveSpeed = 0.7,
                      float rotateSpeed = 0.6);

        /**
         * Initialization.
         */
        void begin();

        /**
         * Enable beacon tracking.
         */
        void enable();

        /**
         * Disable beacon tracking.
         */
        void disable();

        /**
         * Update control input and pass to base.
         */
        void update();

        /**
         * Sets target.
         */
        void setTarget(float target);

        /**
         * Sets target.
         */
        void setFrequency(float freq1, float freq2);

    private:
        MecanumBase* _base;
        unsigned int _irPins[NUM_BEACONS];
        unsigned int _setFrequencies[NUM_BEACONS];
        unsigned int _desPeriod[NUM_BEACONS];
        unsigned int _noise[NUM_BEACONS];
        volatile unsigned int _period[NUM_BEACONS];
        volatile unsigned int _freq[NUM_BEACONS];
        volatile float _target;
        float _driveSpeed;
        float _rotateSpeed;

        volatile int currentRisingEdgeTime[2];
        volatile int lastRisingEdgeTime[2];
        volatile float period[2];

        /**
         * Search for beacon.
         */
        void findBeacon();

        /**
         * Check period on IR detector.
         */
        bool isClose(float a, float b, float epsilon);
};

#endif  // BEACONTRACKER_h
