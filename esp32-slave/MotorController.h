/**
 * MotorController.h
 * Header for MotorController class.
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#ifndef MOTORCONTROLLER_h
#define MOTORCONTROLLER_h

#include <Arduino.h>  // arduino commands
#include "Encoder.h"  // Encoder object

enum MotorDirection { forward, backward, freeSpin, braking };

/**
 * PWM-driven brushed motor controller.
 */
class MotorController 
{
    public:
        float _speed = 0.0;
        MotorDirection _direction;

        /**
         * Constructor for MotorController with single direction pin.
         * 
         * @param enablingPin   Enabling signal output pin.  Negative if PWM 
         *                      drive through direction pin.
         * @param directionPin  Direction output pin.
         */
        MotorController(int enablingPin, 
                        unsigned int directionPin);

        /**
         * Constructor for MotorController with single direction pin and an 
         * encoder.
         * 
         * @param enablingPin   Enabling signal output pin.  Negative if PWM 
         *                      drive through direction pin.
         * @param directionPin  Direction output pin.
         * @param encoder       Motor encoder object.
         */
        MotorController(int enablingPin, 
                        unsigned int directionPin,
                        Encoder& encoder);

        /**
         * Constructor for MotorController with two direction pins.
         * 
         * @param enablingPin           Enabling signal output pin.  Negative if
         *                              PWM drive through direction pins.
         * @param forwardDirectionPin   Forward direction output pin.
         * @param reverseDirectionPin   Reverse direction output pin.
         */
        MotorController(int enablingPin, 
                        unsigned int forwardDirectionPin,
                        unsigned int reverseDirectionPin);

        /**
         * Constructor for MotorController with two direction pins and an 
         * encoder.
         * 
         * @param enablingPin           Enabling signal output pin.  Negative if
         *                              PWM drive through direction pins.
         * @param forwardDirectionPin   Forward direction output pin.
         * @param reverseDirectionPin   Reverse direction output pin.
         * @param encoder               Motor encoder Encoder object.
         */
        MotorController(int enablingPin, 
                        unsigned int forwardDirectionPin,
                        unsigned int reverseDirectionPin,
                        Encoder& encoder);

        /**
         * Setup and initialize pins.
         */
        void setup();

        /**
         * Sets the motor speed.
         * 
         * @param speed Speed in range [-1.0, 1.0]
         */
        void set(float speed);

        /**
         * Adjusts motor speed.
         * 
         * @param increment Amount to increment or decrement speed by.  In range
         *                  [0.0, 1.0] to increment.  In range [-1.0, 0.0] to 
         *                  decrement.
         */
        void adjust(float increment);

        /**
         * Stops motors without braking.
         */
        void stop();

        /**
         * Stops motors with braking.
         */
        void brake();

    protected:

    private:
        unsigned int _enablingPin;
        unsigned int _directionPin[2];
        bool _isEnablingDrive;
        bool _isDualDirection;
        bool _hasEncoder;
        Encoder* _encoder;
        float _dutyCycle = 0.0;

        /**
         * Set motor duty cycle and direction.
         * 
         * @param duty Duty cycle in range[0.0, 1.0].
         * @param direction Motor direction (forward, backward, freeSpin, 
         *                  braking).
         */
        void setDuty(float duty, MotorDirection direction);

        /**
         * Set motor duty cycle driving forward.
         * 
         * @param duty Duty cycle in range[0.0, 1.0].
         */
        void setDutyForward(float duty);

        /**
         * Set motor duty cycle driving backward.
         * 
         * @param duty Duty cycle in range[0.0, 1.0].
         */
        void setDutyBackward(float duty);

        /**
         * Set motor duty cycle free spinning.
         * 
         * @param duty Duty cycle in range[0.0, 1.0].
         */
        void setDutyFree(float duty);

        /**
         * Set motor duty cycle braking.
         * 
         * @param duty Duty cycle in range[0.0, 1.0].
         */
        void setDutyBrake(float duty);
};

#endif // MOTORCONTROLLER_h
