/**
 * esp32-master.ino
 * Code for the master ESP32 Pico Kit used for sensing and coordination.
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#define IR_PIN 21
#define TARGET_PIN 19
#define RESP_PIN 22

volatile int currentRisingEdgeTime = 0;
volatile int lastRisingEdgeTime = 0;
volatile double period = 0.0;
volatile double frequency = 0.0;
volatile double target = 0.0;

void IRAM_ATTR detectRisingEdgeHandler() {
    currentRisingEdgeTime = micros();
    period = (currentRisingEdgeTime - lastRisingEdgeTime) / 1.0e6;
    frequency = 1.0 / period;
    lastRisingEdgeTime = currentRisingEdgeTime;
}

bool isClose(double a, double b, double epsilon) {
    return abs(a - b) < abs(epsilon);
}

void setup() {
    Serial.begin(115200);
    pinMode(IR_PIN, INPUT);
    pinMode(TARGET_PIN, OUTPUT);
    pinMode(RESP_PIN, OUTPUT);
    digitalWrite(RESP_PIN, LOW);
    attachInterrupt(digitalPinToInterrupt(IR_PIN), detectRisingEdgeHandler, RISING);
}

void loop() {
    noInterrupts();
    if(digitalRead(TARGET_PIN) == LOW) {
        if(isClose(frequency, 23.0, 15.0)) {
            digitalWrite(RESP_PIN, HIGH);
        } else {
            digitalWrite(RESP_PIN, LOW);
        }
    } else {
        if(isClose(frequency, 700.0, 100.0)) {
            digitalWrite(RESP_PIN, HIGH);
        } else {
            digitalWrite(RESP_PIN, LOW);
        }
    }
    interrupts();
}