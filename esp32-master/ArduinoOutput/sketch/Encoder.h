#line 1 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\Encoder.h"
/**
 * Encoder.h
 * Header for Encoder class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>  // arduino commands

/**
 * Single-signal encoder.
 */
class Encoder 
{
    public:
        float _speed;

        /**
         * Constructor for Encoder.
         * 
         * @param inputPin      Digital signal input pin.
         * @param countsPerRev  Disc counts per revolution.
         */
        Encoder(unsigned int inputPin,
                unsigned int countsPerRev);

        /**
         * Compute average angular velocity since last reset.
         */
        void compute();

        /**
         * Reset encoder count.
         */
        void reset();

    protected:

    private:
        unsigned int _inputPin;
        unsigned int _countsPerRev;
        volatile int _count;

        /**
         * Callback for trigger interrupt.
         */
        void IRAM_ATTR onTrigger();
};

#endif // ENCODER_h
