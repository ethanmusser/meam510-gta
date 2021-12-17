/**
 * esp32-slave.ino
 * Code for the slave ESP32 Pico Kit used for actuation and mobility.
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#include "indexJS.h"
#include "html510.h"
#include "MecanumBase.h"
#include "APC.h"

/**
 * Pin Definitions
 */
// LV8401V Driver Pins
#define M1_IN1_PIN 33
#define M1_IN2_PIN 32
#define M2_IN1_PIN 23
#define M2_IN2_PIN 19
#define M3_IN1_PIN 22
#define M3_IN2_PIN 21
#define M4_IN1_PIN 25
#define M4_IN2_PIN 26
// Vive Pins
#define VIVE_F_PIN 36
#define VIVE_R_PIN 39

/**
 * Miscellaneous Definitions
 */
#define DEBUGMODE 0
#define SPEEDINC 25

/**
 * Constants
 */
const char* ssid = "Walrus";
const char* password = "password";
const float speedAdjustmentFactor = 0.1;
const float rotationalSpeedFactor = 0.7;

/**
 * Global Variables
 */
float baseSpeed = 0.8;

/**
 * Global Objects
 */
HTML510Server h(80);
// LV8401V Driver MotorController
MotorController frontLeftMotor(-1, M1_IN1_PIN, M1_IN2_PIN);
MotorController frontRightMotor(-1, M2_IN1_PIN, M2_IN2_PIN);
MotorController rearLeftMotor(-1, M3_IN1_PIN, M3_IN2_PIN);
MotorController rearRightMotor(-1, M4_IN1_PIN, M4_IN2_PIN);
// Mobile Base
MecanumBase base(frontLeftMotor, frontRightMotor, 
                 rearLeftMotor, rearRightMotor);
// Vive Sensors
Vive510 frontVive(VIVE_F_PIN);
Vive510 rearVive(VIVE_R_PIN);
// Absolute Position Control
APC apc(base, frontVive, rearVive, 0.0, 0.0, M_PI_2, 1.0, 0.0, 0.2);


/* -------------------------------------------------------------------------- */

/**
 * Web handler.
 */
void handleRoot() {
    h.sendhtml(body);
}

/**
 * Move forward button press handler.
 */
void handleMoveForwardButtonHit() {
    base.driveCartesian(baseSpeed * 1.0, 0.0, 0.0);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleMoveForwardButtonHit");
    h.sendplain("");
}

/**
 * Move backward button press handler.
 */
void handleMoveBackwardButtonHit() {
    base.driveCartesian(baseSpeed * -1.0, 0.0, 0.0);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleMoveBackwardButtonHit");
    h.sendplain("");
}

/**
 * Move left button press handler.
 */
void handleMoveLeftButtonHit() {
    base.driveCartesian(0.0, baseSpeed * -1.0, 0.0);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleMoveLeftButtonHit");
    h.sendplain("");
}

/**
 * Move right button press handler.
 */
void handleMoveRightButtonHit() {
    base.driveCartesian(0.0, baseSpeed * 1.0, 0.0);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleMoveRightButtonHit");
    h.sendplain("");
}

/**
 * Rotate left button press handler.
 */
void handleRotateLeftButtonHit() {
    base.driveCartesian(0.0, 0.0, baseSpeed * rotationalSpeedFactor * 1.0);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleRotateLeftButtonHit");
    h.sendplain("");
}

/**
 * Rotate right button press handler.
 */
void handleRotateRightButtonHit() {
    base.driveCartesian(0.0, 0.0, baseSpeed * rotationalSpeedFactor * -1.0);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleRotateRightButtonHit");
    h.sendplain("");
}

/**
 * Movement button release handler.
 */
void handleMovementButtonRelease() {
    base.stop();
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleMovementButtonRelease");
    h.sendplain("");
}

/**
 * Speed up button press handler.
 */
void handleSpeedUpButtonHit() {
    baseSpeed = constrain(baseSpeed + speedAdjustmentFactor, 0.0, 1.0);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleSpeedUpButtonHit");
    h.sendplain("");
}

/**
 * Slow down button press handler.
 */
void handleSpeedDownButtonHit() {
    baseSpeed = constrain(baseSpeed - speedAdjustmentFactor, 0.0, 1.0);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleSpeedDownButtonHit");
    h.sendplain("");
}

/**
 * Full speed button press handler.
 */
void handleSpeedFullButtonHit() {
    baseSpeed = 1.0;
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleSpeedFullButtonHit");
    h.sendplain("");
}

/**
 * Zero speed button press handler.
 */
void handleSpeedZeroButtonHit() {
    baseSpeed = 0.0;
    base.brake();
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleSpeedZeroButtonHit");
    h.sendplain("");
}

/**
 * Set destination button press handler.
 */
void handleSetDestinationButtonHit() {
    String str = h.getText();
    float x = str.substring(0, 4).toFloat() / 1.0e3;
    float y = str.substring(5, 9).toFloat() / 1.0e3;
    float q = str.substring(10).toFloat() * M_PI / 180.0;
    apc.setDestination(x, y, q);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleSetDestinationButtonHit");
    h.sendplain("");
}

/**
 * Set destination button press handler.
 */
void handleSetGainsButtonHit() {
    String str = h.getText();
    float kp = str.substring(0, 4).toFloat() / 1.0e3;
    float ki = str.substring(5, 9).toFloat() / 1.0e3;
    float kd = str.substring(10).toFloat() * M_PI / 180.0;
    apc.setGains(kp, ki, kd);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleSetGainsButtonHit");
    h.sendplain("");
}

/**
 * Direction state update handler.
 */
void handle_direction_state() {
    // TODO: Implement
    // String s = direction ? "Forward" : "Reverse";
    String s = "";
    h.sendplain(s);
}

/* -------------------------------------------------------------------------- */

/**
 * Main setup.
 */
void setup() {
    // General
    Serial.begin(115200);

    // WiFi
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(IPAddress(192, 168, 1, 153),  IPAddress(192, 168, 1, 1), 
            IPAddress(255, 255, 255, 0)); 
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");  Serial.println(myIP);      

    // Web Buttons
    h.begin();
    h.attachHandler("/ ", handleRoot);
    h.attachHandler("/move_fwd_btn_hit", handleMoveForwardButtonHit);
    h.attachHandler("/move_bkwd_btn_hit", handleMoveBackwardButtonHit);
    h.attachHandler("/move_left_btn_hit", handleMoveLeftButtonHit);
    h.attachHandler("/move_right_btn_hit", handleMoveRightButtonHit);
    h.attachHandler("/rot_left_btn_hit", handleRotateLeftButtonHit);
    h.attachHandler("/rot_right_btn_hit", handleRotateRightButtonHit);
    h.attachHandler("/mvmt_btn_release", handleMovementButtonRelease);
    h.attachHandler("/spd_up_btn_hit", handleSpeedUpButtonHit);
    h.attachHandler("/spd_dwn_btn_hit", handleSpeedDownButtonHit);
    h.attachHandler("/spd_full_btn_hit", handleSpeedFullButtonHit);
    h.attachHandler("/spd_zero_btn_hit", handleSpeedZeroButtonHit);
    h.attachHandler("/destination?val=", handleSetDestinationButtonHit);
    h.attachHandler("/gains?val=", handleSetGainsButtonHit);
    
    // Motors
    frontLeftMotor.setup();
    frontRightMotor.setup();
    rearLeftMotor.setup();
    rearRightMotor.setup();

    // Vive Sensors
    frontVive.begin();
    rearVive.begin();
}

/**
 * Main loop.
 */
void loop() {
    // Serve Web Requests
    h.serve();

    // Update Base Control
    apc.update();

    // Wait
    delay(10);
}
