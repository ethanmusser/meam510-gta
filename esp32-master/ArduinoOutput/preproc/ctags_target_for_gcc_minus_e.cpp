# 1 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
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
# 12 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
# 13 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 2
# 14 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 2
# 15 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 2
# 16 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 2
# 17 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 2
# 18 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 2
# 19 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 2
# 20 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 2
# 21 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 2
# 22 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 2

/**

 * Pin Definitions

 */
# 26 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
// LV8401V Driver Pins
# 35 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
// Vive Pins


// Time of Flight Pins



// IR Sensor Pins


// Autonomous LED


/**

 * Miscellaneous Definitions

 */


/**

 * Constants

 */
# 58 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
// const char* ssid = "TP-Link_E0C8";
// const char* password = "52665134";
const char* ssid = "TP-Link_05AF";
const char* password = "47543454";
// const char* ssid = "Walrus";
// const char* password = "password";
const float speedAdjustmentFactor = 0.1;
const float rotationalSpeedFactor = 0.7;

/**

 * Global Variables

 */
# 70 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
unsigned int robotNumber = 0;
float baseSpeed = 0.8;
enum AutonomousMode { noAutonomous, apcAutonomous, wallFollowingAutonomous, beaconTrackingAutonomous } autoMode;

/**

 * Global Objects

 */
# 77 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
// UDP
WiFiUDP locationUDPServer;
WiFiUDP canUDPServer;
WiFiUDP robotUDPServer;
IPAddress ipTarget(192, 168, 1, 255); // broadcast address to everyone at 192.168.1.xxx
IPAddress ipLocal(192, 168, 0, 160); // our IP address
// Web Interface
HTML510Server h(80);
// LV8401V Driver MotorController
MotorController frontLeftMotor(-1, 19, 23);
MotorController frontRightMotor(-1, 25, 26);
MotorController rearLeftMotor(-1, 32, 33);
MotorController rearRightMotor(-1, 5, 18);
// Mobile Base
MecanumBase base(frontLeftMotor, frontRightMotor,
                 rearLeftMotor, rearRightMotor);
// Vive Sensors
Vive510 frontVive(36);
Vive510 rearVive(39);
// Absolute Position Control
APC apc(base, frontVive, rearVive,
        0.0, 0.0, 3.0*
# 98 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 3 4
                     1.57079632679489661923
# 98 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
                           ,
        1.0, 0.2, 0.6, 0.15,
        0.02, 0.17);
// Time of Flight Sensors
Adafruit_VL53L0X frontLox = Adafruit_VL53L0X();
Adafruit_VL53L0X rightFrontLox = Adafruit_VL53L0X();
Adafruit_VL53L0X rightRearLox = Adafruit_VL53L0X();
TripleTOF lox(frontLox, 0x30, 15,
              rightFrontLox, 0x31, 14,
              rightRearLox, 0x32, 27);
// Wall Following
WallFollower wf(base, lox, 237,
                300, 200, 180,
                1.0, 1.0,
                0.7, 0.7, 0.5);
// Beacon Tracking
BeaconTracker bt(base, 10, 4,
                 23, 700, 5000, 5000,
                 0.7, 0.50);


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
# 164 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleRoot() {
    h.sendhtml(body);
}

/**

 * Move forward button press handler.

 */
# 171 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleMoveForwardButtonHit() {
    base.driveCartesian(baseSpeed * 1.0, 0.0, 0.0);
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleMoveForwardButtonHit");
    h.sendplain("");
}

/**

 * Move backward button press handler.

 */
# 180 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleMoveBackwardButtonHit() {
    base.driveCartesian(baseSpeed * -1.0, 0.0, 0.0);
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleMoveBackwardButtonHit");
    h.sendplain("");
}

/**

 * Move left button press handler.

 */
# 189 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleMoveLeftButtonHit() {
    base.driveCartesian(0.0, baseSpeed * -1.0, 0.0);
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleMoveLeftButtonHit");
    h.sendplain("");
}

/**

 * Move right button press handler.

 */
# 198 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleMoveRightButtonHit() {
    base.driveCartesian(0.0, baseSpeed * 1.0, 0.0);
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleMoveRightButtonHit");
    h.sendplain("");
}

/**

 * Rotate left button press handler.

 */
# 207 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleRotateLeftButtonHit() {
    base.driveCartesian(0.0, 0.0, baseSpeed * rotationalSpeedFactor * 1.0);
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleRotateLeftButtonHit");
    h.sendplain("");
}

/**

 * Rotate right button press handler.

 */
# 216 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleRotateRightButtonHit() {
    base.driveCartesian(0.0, 0.0, baseSpeed * rotationalSpeedFactor * -1.0);
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleRotateRightButtonHit");
    h.sendplain("");
}

/**

 * Movement button release handler.

 */
# 225 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleMovementButtonRelease() {
    base.stop();
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleMovementButtonRelease");
    h.sendplain("");
}

/**

 * Speed up button press handler.

 */
# 234 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleSpeedUpButtonHit() {
    baseSpeed = ((baseSpeed + speedAdjustmentFactor)<(0.0)?(0.0):((baseSpeed + speedAdjustmentFactor)>(1.0)?(1.0):(baseSpeed + speedAdjustmentFactor)));
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleSpeedUpButtonHit");
    h.sendplain("");
}

/**

 * Slow down button press handler.

 */
# 243 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleSpeedDownButtonHit() {
    baseSpeed = ((baseSpeed - speedAdjustmentFactor)<(0.0)?(0.0):((baseSpeed - speedAdjustmentFactor)>(1.0)?(1.0):(baseSpeed - speedAdjustmentFactor)));
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleSpeedDownButtonHit");
    h.sendplain("");
}

/**

 * Full speed button press handler.

 */
# 252 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleSpeedFullButtonHit() {
    baseSpeed = 1.0;
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleSpeedFullButtonHit");
    h.sendplain("");
}

/**

 * Zero speed button press handler.

 */
# 261 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleSpeedZeroButtonHit() {
    baseSpeed = 0.0;
    base.brake();
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleSpeedZeroButtonHit");
    h.sendplain("");
}

/**

 * Start wall following button press handler.

 */
# 271 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleStartWallFollowingButtonHit() {
    apc.disable();
    wf.enable();
    autoMode = wallFollowingAutonomous;
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleStartWallFollowingButtonHit");
    h.sendplain("");
}

/**

 * Stop autonomous button press handler.

 */
# 282 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleStopAutonomousButtonHit() {
    apc.disable();
    wf.disable();
    bt.disable();
    base.brake();
    autoMode = noAutonomous;
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleSpeedZeroButtonHit");
    h.sendplain("");
}

/**

 * Set offsets button press handler.

 */
# 295 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleSetOffsetsButtonHit() {
    String str = h.getText();
    float x = str.substring(0, 4).toFloat() / 1.0e3;
    float y = str.substring(5, 9).toFloat() / 1.0e3;
    float q = (str.substring(10).toFloat() - 180) * 
# 299 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 3 4
                                                   3.14159265358979323846 
# 299 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
                                                        / 180.0;
    apc.setOffsets(x, y, q);
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleSetOffsetsButtonHit");
    h.sendplain("");
}


/**

 * Set destination button press handler.

 */
# 309 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleSetDestinationButtonHit() {
    String str = h.getText();
    float x = str.substring(0, 4).toFloat() / 1.0e3;
    float y = str.substring(5, 9).toFloat() / 1.0e3;
    float q = (str.substring(10).toFloat() - 180) * 
# 313 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 3 4
                                                   3.14159265358979323846 
# 313 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
                                                        / 180.0;
    apc.setDestination(x, y, q);
    apc.enable();
    autoMode = apcAutonomous;
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleSetDestinationButtonHit");
    h.sendplain("");
}

/**

 * Set destination button press handler.

 */
# 324 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleSetGainsButtonHit() {
    String str = h.getText();
    float kpTrans = str.substring(0, 4).toFloat();
    float kdTrans = str.substring(5, 9).toFloat();
    float kpRot = str.substring(10, 14).toFloat();
    float kdRot = str.substring(15).toFloat();
    apc.setGains(kpTrans, kdTrans, kpRot, kdRot);
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleSetGainsButtonHit");
    h.sendplain("");
}

/**

 * Set destination button press handler.

 */
# 338 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleSetBeaconButtonHit() {
    bt.setTarget(h.getVal() == 0 ? 23.0 : 700.0);
    bt.enable();
    autoMode = beaconTrackingAutonomous;
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleSetBeaconButtonHit");
    h.sendplain("");
}

/**

 * Set destination button press handler.

 */
# 349 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleSetRobotNumber() {
    robotNumber = h.getVal();
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleSetRobotNumber");
    h.sendplain("");
}

/**

 * Current position update handler.

 */
# 358 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleGetCurrentPosition() {
    Pose2D pose = apc.getPose();
    int x = (int) (pose.x * 1.0e3);
    int y = (int) (pose.y * 1.0e3);
    int theta = (int) (pose.theta * 180.0 / 
# 362 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 3 4
                                           3.14159265358979323846
# 362 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
                                               ) + 180;
    String str = "x = " + String(x) + "mm, \ny = " + String(y) + " mm, \nq = " + String(theta) + " deg\n";
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleGetCurrentPosition");
    h.sendplain(str);
}

/**

 * Desired position update handler.

 */
# 371 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleGetDesiredPosition() {
    int x = (int) (apc._desiredPose.x * 1.0e3);
    int y = (int) (apc._desiredPose.y * 1.0e3);
    int theta = (int) (apc._desiredPose.theta * 180.0 / 
# 374 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 3 4
                                                       3.14159265358979323846
# 374 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
                                                           ) + 180;
    String str = "x = " + String(x) + "mm, \ny = " + String(y) + " mm, \nq = " + String(theta) + " deg\n";
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleGetDesiredPosition");
    h.sendplain(str);
}

/**

 * ToF values update handler.

 */
# 383 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void handleGetLoxVals() {
    int front = wf._currentPose.front;
    int right = wf._currentPose.right;
    int theta = (int) (wf._currentPose.theta * 180.0 / 
# 386 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino" 3 4
                                                      3.14159265358979323846
# 386 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
                                                          ) + 180;
    int mode = wf._state;
    String str = "f = " + String(front) + "mm, \nr = " + String(right) + " mm, \nq = " + String(theta) + " deg\n, mode = " + String(mode) + "\n";
    if (0) Serial.println("[DEBUG][esp32-slave.ino] handleGetLoxVals");
    h.sendplain(str);
}

/* -------------------------------------------------------------------------- */

static volatile int currentRisingEdgeTime[2];
static volatile int lastRisingEdgeTime[2];
static volatile double period[2];
static volatile double frequency[2];

void __attribute__((section(".iram1" "." "27"))) computeFrequency(unsigned int ch) {
    currentRisingEdgeTime[ch] = micros();
    period[ch] = (currentRisingEdgeTime[ch] - lastRisingEdgeTime[ch]) / 1.0e6;
    frequency[ch] = 1.0 / period[ch];
    lastRisingEdgeTime[ch] = currentRisingEdgeTime[ch];
}

void __attribute__((section(".iram1" "." "28"))) detectRisingEdgeLeft() {
    computeFrequency(0);
}

void __attribute__((section(".iram1" "." "29"))) detectRisingEdgeRight() {
    computeFrequency(1);
}

/* -------------------------------------------------------------------------- */

/**

 * Main setup.

 */
# 420 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void setup() {
    // General
    Serial.begin(115200);

    // WiFi
    if(0) {
        WiFi.softAP(ssid, password);
        WiFi.softAPConfig(IPAddress(192, 168, 1, 153), IPAddress(192, 168, 1, 1),
                IPAddress(255, 255, 255, 0));
        IPAddress myIP = WiFi.softAPIP();
        Serial.print("AP IP address: "); Serial.println(myIP);
    } else {
        Serial.print("Connecting to "); Serial.println(ssid);
        WiFi.config(ipLocal, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
        WiFi.begin(ssid, password);
    }

    // UDP
    canUDPServer.begin(1510); // can port 1510
    robotUDPServer.begin(2510); // robot port 2510
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
    h.attachHandler("/start_wall_follow_btn_hit", handleStartWallFollowingButtonHit);
    h.attachHandler("/offsets?val=", handleSetOffsetsButtonHit);
    h.attachHandler("/bacon?val=", handleSetBeaconButtonHit);
    h.attachHandler("/destination?val=", handleSetDestinationButtonHit);
    h.attachHandler("/gains?val=", handleSetGainsButtonHit);
    h.attachHandler("/robot_num?val=", handleSetRobotNumber);
    h.attachHandler("/cur_pos?val=", handleGetCurrentPosition);
    h.attachHandler("/des_pos?val=", handleGetDesiredPosition);
    h.attachHandler("/lox?val=", handleGetLoxVals);

    // Motors
    frontLeftMotor.setup();
    frontRightMotor.setup();
    rearLeftMotor.setup();
    rearRightMotor.setup();

    // Vive Sensors
    frontVive.begin();
    rearVive.begin();

    // LOX Sensors
    lox.begin();

    // Beacon Tracker
    bt.begin();
    attachInterrupt((((10)<40)?(10):-1), detectRisingEdgeLeft, 0x01);
    attachInterrupt((((4)<40)?(4):-1), detectRisingEdgeRight, 0x01);
    // Serial.println("Setup Complete.");

    // Autonomous Light
    pinMode(12, 0x02);
}

/**

 * Main loop.

 */
# 497 "c:\\Users\\ethan\\Google Drive\\School\\11. Masters III - Fall 2021\\Mechatronics (MEAM 510)\\Project\\code\\meam510-gta\\esp32-slave\\esp32-slave.ino"
void loop() {
    // UDP
    handleCanMsg();
    handleRobotMsg();

    // Serve Web Requests
    h.serve();

    // Update Base Control
    static long autoStartTime = millis();
    static bool lastAuto = false;
    if(autoMode != noAutonomous && !lastAuto) {
        autoStartTime = millis();
    }
    if(millis() - autoStartTime >= 5000 && lastAuto) {
        digitalWrite(12, 0x1);
    } else {
        digitalWrite(12, 0x0);
    }
    ;
    switch(autoMode) {
        case noAutonomous:
            lastAuto = false;
            break;
        case apcAutonomous:
            lastAuto = true;
            apc.update();
            break;
        case wallFollowingAutonomous:
            lastAuto = true;
            wf.update();
            break;
        case beaconTrackingAutonomous:
            lastAuto = true;
            bt.setFrequency(frequency[0], frequency[1]);
            bt.update();
            break;
    }
    ;

    // Broadcast Location
    static unsigned long lastLocationTxTime = 0;
    if(millis() - lastLocationTxTime >= 1000) {
        char loc[13];
        sprintf(loc, "%1d:%4d,%4d", robotNumber, frontVive.xCoord(),
                rearVive.yCoord());
        fncUdpSend(loc, 13);
        lastLocationTxTime = millis();
    }
}
