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

#include <WiFi.h>
#include <WiFiUdp.h>
#include "indexJS.h"
#include "html510.h"
#include "MecanumBase.h"
#include "APC.h"

/**
 * Pin Definitions
 */
// LV8401V Driver Pins
#define M1_IN1_PIN 19
#define M1_IN2_PIN 23
#define M2_IN1_PIN 25
#define M2_IN2_PIN 26
#define M3_IN1_PIN 32
#define M3_IN2_PIN 33
#define M4_IN1_PIN 5 
#define M4_IN2_PIN 18
// Vive Pins
#define VIVE_F_PIN 36
#define VIVE_R_PIN 39

/**
 * Miscellaneous Definitions
 */
#define APMODE 0
#define DEBUGMODE 0
#define SPEEDINC 25

/**
 * Constants
 */
const char* ssid = "TP-Link_05AF";
const char* password = "47543454";
// const char* ssid = "Walrus";
// const char* password = "password";
const float speedAdjustmentFactor = 0.1;
const float rotationalSpeedFactor = 0.7;

/**
 * Global Variables
 */
unsigned int robotNumber = 1;
float baseSpeed = 0.8;

/**
 * Global Objects
 */
// UDP
WiFiUDP locationUDPServer;
WiFiUDP canUDPServer;
WiFiUDP robotUDPServer;
IPAddress ipTarget(192, 168, 1, 255); // broadcast address to everyone at 192.168.1.xxx
IPAddress ipLocal(192, 168, 0, 160);   // our IP address
// Web Interface
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
APC apc(base, frontVive, rearVive, 
        0.0, 0.0, 3.0*M_PI_2,
        1.0, 0.2, 0.6, 0.15,
        0.02, 0.17);


/* -------------------------------------------------------------------------- */

void fncUdpSend(char *datastr, int len) {
    locationUDPServer.beginPacket(ipTarget, 2510);
    locationUDPServer.write((uint8_t *)datastr, len);
    locationUDPServer.endPacket();
}

void handleCanMsg() {
    const int UDP_PACKET_SIZE = 14; // can be up to 65535
    uint8_t packetBuffer[UDP_PACKET_SIZE];

    int cb = canUDPServer.parsePacket();
    if (cb)
    {
        int x, y;
        packetBuffer[cb] = 0; // null terminate string
        canUDPServer.read(packetBuffer, UDP_PACKET_SIZE);

        x = atoi((char *)packetBuffer + 2);
        y = atoi((char *)packetBuffer + 7);
    }
}

void handleRobotMsg() {
    const int UDP_PACKET_SIZE = 14; // can be up to 65535
    uint8_t packetBuffer[UDP_PACKET_SIZE];

    int cb = robotUDPServer.parsePacket();
    if (cb)
    {
        int x, y;
        packetBuffer[cb] = 0; // null terminate string
        robotUDPServer.read(packetBuffer, UDP_PACKET_SIZE);

        x = atoi((char *)packetBuffer + 2);
        y = atoi((char *)packetBuffer + 7);
    }
}

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
 * Stop autonomous button press handler.
 */
void handleStopAutonomousButtonHit() {
    apc.disable();
    base.brake();
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleSpeedZeroButtonHit");
    h.sendplain("");
}

/**
 * Set offsets button press handler.
 */
void handleSetOffsetsButtonHit() {
    String str = h.getText();
    float x = str.substring(0, 4).toFloat() / 1.0e3;
    float y = str.substring(5, 9).toFloat() / 1.0e3;
    float q = (str.substring(10).toFloat() - 180) * M_PI / 180.0;
    apc.setOffsets(x, y, q);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleSetOffsetsButtonHit");
    h.sendplain("");
}


/**
 * Set destination button press handler.
 */
void handleSetDestinationButtonHit() {
    String str = h.getText();
    float x = str.substring(0, 4).toFloat() / 1.0e3;
    float y = str.substring(5, 9).toFloat() / 1.0e3;
    float q = (str.substring(10).toFloat() - 180) * M_PI / 180.0;
    apc.setDestination(x, y, q);
    apc.enable();
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleSetDestinationButtonHit");
    h.sendplain("");
}

/**
 * Set destination button press handler.
 */
void handleSetGainsButtonHit() {
    String str = h.getText();
    float kpTrans = str.substring(0, 4).toFloat();
    float kdTrans = str.substring(5, 9).toFloat();
    float kpRot = str.substring(10, 14).toFloat();
    float kdRot = str.substring(15).toFloat();
    apc.setGains(kpTrans, kdTrans, kpRot, kdRot);
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleSetGainsButtonHit");
    h.sendplain("");
}

/**
 * Set destination button press handler.
 */
void handleSetRobotNumber() {
    robotNumber = h.getVal();
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleSetRobotNumber");
    h.sendplain("");
}

/**
 * Current position update handler.
 */
void handleGetCurrentPosition() {
    Pose2D pose = apc.getPose();
    int x = (int) (pose.x * 1.0e3);
    int y = (int) (pose.y * 1.0e3);
    int theta = (int) (pose.theta * 180.0 / M_PI) + 180;
    String str = "x = " + String(x) + "mm, \ny = " + String(y) + " mm, \nq = " + String(theta) + " deg\n";
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleGetCurrentPosition");
    h.sendplain(str);
}

/**
 * Desired position update handler.
 */
void handleGetDesiredPosition() {
    int x = (int) (apc._desiredPose.x * 1.0e3);
    int y = (int) (apc._desiredPose.y * 1.0e3);
    int theta = (int) (apc._desiredPose.theta * 180.0 / M_PI) + 180;
    String str = "x = " + String(x) + "mm, \ny = " + String(y) + " mm, \nq = " + String(theta) + " deg\n";
    if (DEBUGMODE) Serial.println("[DEBUG][esp32-slave.ino] handleGetDesiredPosition");
    h.sendplain(str);
}

/* -------------------------------------------------------------------------- */

/**
 * Main setup.
 */
void setup() {
    // General
    Serial.begin(115200);

    // WiFi
    if(APMODE) {
        WiFi.softAP(ssid, password);
        WiFi.softAPConfig(IPAddress(192, 168, 1, 153),  IPAddress(192, 168, 1, 1), 
                IPAddress(255, 255, 255, 0)); 
        IPAddress myIP = WiFi.softAPIP();
        Serial.print("AP IP address: ");  Serial.println(myIP);   
    } else {   
        Serial.print("Connecting to "); Serial.println(ssid);
        WiFi.config(ipLocal, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
        WiFi.begin(ssid, password);
    }

    // UDP
    canUDPServer.begin(1510);     // can port 1510
    robotUDPServer.begin(2510);   // robot port 2510
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");

    // Web Interface
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
    h.attachHandler("/stop_auto_btn_hit", handleStopAutonomousButtonHit);
    h.attachHandler("/offsets?val=", handleSetOffsetsButtonHit);
    h.attachHandler("/destination?val=", handleSetDestinationButtonHit);
    h.attachHandler("/gains?val=", handleSetGainsButtonHit);
    h.attachHandler("/robot_num?val=", handleSetRobotNumber);
    h.attachHandler("/cur_pos?val=", handleGetCurrentPosition);
    h.attachHandler("/des_pos?val=", handleGetDesiredPosition);
    
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
    // UDP
    handleCanMsg();
    handleRobotMsg();

    // Serve Web Requests
    h.serve();

    // Update Base Control
    static unsigned int loopCount = 0;
    if(loopCount >= 20) {
        apc.update();
        loopCount = 0;
    }
    loopCount++;

    // Broadcast Location
    static unsigned int lastLocationTxTime = 0;
    if(millis() - lastLocationTxTime >= 1000) {
        char loc[13];
        sprintf(loc, "%1d:%4d,%4d", robotNumber, frontVive.xCoord(), 
                rearVive.yCoord());
        fncUdpSend(loc, 13);
    }
    
    // Wait
    delay(10);
}
