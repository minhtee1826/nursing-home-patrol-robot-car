/*
  ESP32 Robot Control with PID Self-Driving
  - WiFi remote control via WebSocket
  - PID-based obstacle avoidance with 90-degree turns
  - Compass via Arduino Nano serial
  - Ultrasonic distance sensing
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HardwareSerial.h>
#include <math.h>

const char* ssid = "wifi32";
const char* password = "123456789";

// ====================================================
// PIN DEFINITIONS (matching MEDCARNINETY.ino)
// ====================================================
#define ENA 14
#define IN1 27
#define IN2 26
#define ENB 32
#define IN3 25
#define IN4 33
#define LED_PIN 2

// Ultrasonic sensor pins
#define TRIG_PIN 5
#define ECHO_PIN 18

// Nano serial communication (for compass data)
#define RX_PIN 16
#define TX_PIN 17

// ====================================================
// SPEED & PID SETTINGS
// ====================================================

// Manual control speeds
int DEFAULT_SPEED = 200;
int TURN_SPEED = 180;
int REVERSE_SPEED = 150;

// Self-driving speeds
const int FWD_SPEED = 100;
const int BACK_SPEED = 105;

// PID settings for 90-degree turns
const int MIN_PWM = 100;
const int MAX_PID_SPEED = 160;
float turnKp = 0.67;
float turnKi = 0;
float turnKd = 0.36;
const float TOLERANCE = 4.0;  // degrees
const int LOOP_DELAY = 30;    // ms

// Timing for obstacle avoidance
const int BACK_TIME = 400;
const int OBSTACLE_DIST = 25;

// ====================================================
// GLOBAL VARIABLES
// ====================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
HardwareSerial NanoSerial(2);

// WiFi and command management
unsigned long lastWiFiCheck = 0;
unsigned long lastCommandTime = 0;
unsigned long lastHeartbeat = 0;
const unsigned long WIFI_CHECK_INTERVAL = 3000;
const unsigned long COMMAND_TIMEOUT = 1500;
const unsigned long HEARTBEAT_TIMEOUT = 5000;
const unsigned long MIN_COMMAND_INTERVAL = 80;

String currentCommand = "STOP";
String lastReceivedCommand = "";
bool motorRunning = false;
bool clientConnected = false;

// Self-driving mode
bool selfDrivingActive = false;

// Compass variables
float headingRawX = 0, headingRawY = 0, headingRawZ = 0;
float offX = 0, offY = 0;
float currentHeading = 0;

// ====================================================
// COMPASS HELPER FUNCTIONS
// ====================================================

float calculateHeading(float x, float y) {
    float headingRad = atan2(y, x);
    float headingDeg = headingRad * 180.0 / PI;
    if (headingDeg < 0) {
        headingDeg += 360;
    }
    return headingDeg;
}

void parseData(String data, float &x, float &y, float &z) {
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
   
    if (firstComma > 0 && secondComma > 0) {
        x = data.substring(0, firstComma).toFloat();
        y = data.substring(firstComma + 1, secondComma).toFloat();
        z = data.substring(secondComma + 1).toFloat();
    }
}

float getRelativeHeading(float startAngle, float currentAngle) {
    float diff = currentAngle - startAngle;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    return diff;
}

void readCompass() {
    if (NanoSerial.available()) {
        String data = NanoSerial.readStringUntil('\n');
        parseData(data, headingRawX, headingRawY, headingRawZ);
        float calX = headingRawX - offX;
        float calY = headingRawY - offY;
        currentHeading = calculateHeading(calX, calY);
    }
}

// ====================================================
// ULTRASONIC SENSOR
// ====================================================

int getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 25000);
    if (duration == 0) return 999;
    return duration * 0.034 / 2;
}

// ====================================================
// MOTOR CONTROL FUNCTIONS
// ====================================================

void motorStop() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    motorRunning = false;
    currentCommand = "STOP";
}

void motorForward() {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, DEFAULT_SPEED);
    analogWrite(ENB, DEFAULT_SPEED);
    motorRunning = true;
    currentCommand = "FORWARD";
}

void motorBackward() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, DEFAULT_SPEED);
    analogWrite(ENB, DEFAULT_SPEED);
    motorRunning = true;
    currentCommand = "BACKWARD";
}

void motorLeftSharp() {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, REVERSE_SPEED);
    analogWrite(ENB, TURN_SPEED);
    motorRunning = true;
    currentCommand = "LEFT";
}

void motorRightSharp() {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    analogWrite(ENA, TURN_SPEED);
    analogWrite(ENB, REVERSE_SPEED);
    motorRunning = true;
    currentCommand = "RIGHT";
}

// Self-driving motor functions (lower speed)
void moveForwardAuto() {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, FWD_SPEED);
    analogWrite(ENB, FWD_SPEED);
    motorRunning = true;
}

void moveBackwardAuto() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, BACK_SPEED);
    analogWrite(ENB, BACK_SPEED);
    motorRunning = true;
}

void spinLeft(int speed) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void spinRight(int speed) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

// ====================================================
// PID TURN FUNCTION
// ====================================================

void turnRelativePID(float targetRelativeAngle) {
    bool aligned = false;
    float previousError = 0;
    unsigned long lastTime = millis();
    unsigned long timeoutStart = millis();

    // Flush buffer and read current heading
    while(NanoSerial.available()) readCompass();
    readCompass();
    float startHeading = currentHeading;

    Serial.print("PID Turn - Start: "); Serial.print(startHeading);
    Serial.print(" Target: "); Serial.println(targetRelativeAngle);
    ws.textAll("NAV:TURNING");

    // Timeout after 4 seconds
    while (!aligned && (millis() - timeoutStart < 4000)) {
        readCompass();
       
        unsigned long currentTime = millis();
        float dt = (currentTime - lastTime) / 1000.0;
        if (dt == 0) dt = 0.001;

        float currentRelative = getRelativeHeading(startHeading, currentHeading);
        float error = targetRelativeAngle - currentRelative;

        // Handle wrap-around
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // Check alignment
        if (abs(error) < TOLERANCE) {
            aligned = true;
            motorStop();
            Serial.println("PID Turn - Aligned!");
            break;
        }

        // PID calculation
        float derivative = (error - previousError) / dt;
        float output = (turnKp * error) + (turnKd * derivative);
       
        int speed = abs(output);
       
        // Anti-stall
        if (abs(error) > 15.0 && speed < MIN_PWM) {
            speed = MIN_PWM;
        }
        if (speed > MAX_PID_SPEED) speed = MAX_PID_SPEED;

        // Execute turn
        if (output > 0) {
            spinLeft(speed);
        } else {
            spinRight(speed);
        }

        previousError = error;
        lastTime = currentTime;
        delay(LOOP_DELAY);
    }
    motorStop();
}

// ====================================================
// COMPASS CALIBRATION
// ====================================================

void performCalibration() {
    Serial.println("Calibrating Compass...");
    ws.textAll("NAV:CALIBRATING");
   
    float minX = 1000, maxX = -1000, minY = 1000, maxY = -1000;
    unsigned long start = millis();
   
    spinRight(180);

    while (millis() - start < 3000) {
        readCompass();
        if (headingRawX < minX) minX = headingRawX;
        if (headingRawX > maxX) maxX = headingRawX;
        if (headingRawY < minY) minY = headingRawY;
        if (headingRawY > maxY) maxY = headingRawY;
        delay(10);
    }
    motorStop();
   
    offX = (maxX + minX) / 2;
    offY = (maxY + minY) / 2;
   
    Serial.println("Calibration Complete.");
    ws.textAll("NAV:CALIBRATED");
}

// ====================================================
// OBSTACLE AVOIDANCE
// ====================================================

void avoidObstacle() {
    Serial.println(">> Obstacle Detected!");
    ws.textAll("NAV:OBSTACLE");
    motorStop();
    delay(100);

    // Reverse
    moveBackwardAuto();
    delay(BACK_TIME);
    motorStop();
    delay(200);

    // Turn 90 degrees right
    Serial.println("Decision: 90 Degree Turn Right");
    ws.textAll("NAV:TURN_90");
    turnRelativePID(-90.0);
   
    motorStop();
    delay(500);
}

// ====================================================
// SELF-DRIVING LOOP
// ====================================================

void selfDrivingLoop() {
    if (!selfDrivingActive) return;
   
    readCompass();
    int distance = getDistance();

    // Send distance to client
    ws.textAll("DIST:" + String(distance));

    if (distance > 0 && distance < OBSTACLE_DIST) {
        avoidObstacle();
    } else {
        moveForwardAuto();
    }
}

// ====================================================
// WIFI & WEBSOCKET
// ====================================================

void optimizeWiFi() {
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.setAutoReconnect(true);
    WiFi.mode(WIFI_STA);
    Serial.println("WiFi optimized");
}

void monitorWiFiSignal() {
    if (millis() - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
        lastWiFiCheck = millis();
       
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
            return;
        }
       
        int rssi = WiFi.RSSI();
        String signalLevel = (rssi > -50) ? "EXCELLENT" : (rssi > -70) ? "GOOD" : "WEAK";
        ws.textAll("SIGNAL:" + signalLevel + ":" + String(rssi));
    }
}

bool shouldProcessCommand(const String& command) {
    unsigned long now = millis();
    if (command == lastReceivedCommand && (now - lastCommandTime) < MIN_COMMAND_INTERVAL) {
        return false;
    }
    lastReceivedCommand = command;
    lastCommandTime = now;
    return true;
}

void executeCommand(const String& command) {
    if (command == "FORWARD") {
        selfDrivingActive = false;
        motorForward();
    } else if (command == "BACKWARD") {
        selfDrivingActive = false;
        motorBackward();
    } else if (command == "LEFT") {
        selfDrivingActive = false;
        motorLeftSharp();
    } else if (command == "RIGHT") {
        selfDrivingActive = false;
        motorRightSharp();
    } else if (command == "STOP") {
        selfDrivingActive = false;
        motorStop();
    } else if (command == "NAV_START") {
        Serial.println("Self-Driving Mode ACTIVATED");
        performCalibration();
        selfDrivingActive = true;
        ws.textAll("NAV:ACTIVE");
    } else if (command == "NAV_STOP") {
        selfDrivingActive = false;
        motorStop();
        ws.textAll("NAV:STOPPED");
        Serial.println("Self-Driving Mode STOPPED");
    }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
       
        String command = "";
        for (size_t i = 0; i < len; i++) {
            command += (char)data[i];
        }
       
        lastHeartbeat = millis();
       
        if (command == "PING") {
            ws.textAll("PONG");
            return;
        }
       
        if (shouldProcessCommand(command)) {
            Serial.println("CMD: " + command);
            executeCommand(command);
            ws.textAll("ACK:" + command);
        }
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("Client #%u connected\n", client->id());
            clientConnected = true;
            lastHeartbeat = millis();
            client->text("STATUS:" + currentCommand);
            client->text("SIGNAL:GOOD:" + String(WiFi.RSSI()));
            break;
           
        case WS_EVT_DISCONNECT:
            Serial.printf("Client #%u disconnected\n", client->id());
            clientConnected = false;
            // Keep selfDrivingActive running - don't stop auto mode on disconnect
            // Only stop manual control motors
            if (!selfDrivingActive) {
                motorStop();
            }
            break;
           
        case WS_EVT_DATA:
            handleWebSocketMessage(arg, data, len);
            break;
           
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

void checkSafetyConditions() {
    unsigned long now = millis();
   
    // Only apply timeout in manual mode
    if (!selfDrivingActive && motorRunning && (now - lastCommandTime > COMMAND_TIMEOUT)) {
        motorStop();
        ws.textAll("TIMEOUT:COMMAND");
    }
   
    if (clientConnected && (now - lastHeartbeat > HEARTBEAT_TIMEOUT)) {
        // Only stop manual control on heartbeat timeout, keep auto mode running
        if (!selfDrivingActive) {
            motorStop();
        }
        clientConnected = false;
        ws.textAll("TIMEOUT:HEARTBEAT");
    }
}

// ====================================================
// SETUP & LOOP
// ====================================================

void setup() {
    Serial.begin(115200);
    NanoSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    NanoSerial.setTimeout(10);
    delay(1000);
   
    Serial.println("\n=====================================================");
    Serial.println("ESP32 ROBOT - MANUAL + PID SELF-DRIVING");
    Serial.println("=====================================================");
   
    // Initialize GPIO
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
   
    digitalWrite(LED_PIN, HIGH);
    motorStop();
   
    // WiFi setup
    optimizeWiFi();
    Serial.println("Connecting to WiFi: " + String(ssid));
   
    WiFi.begin(ssid, password);
   
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(1000);
        Serial.print(".");
        attempts++;
    }
   
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n=====================================================");
        Serial.println("WiFi CONNECTED!");
        Serial.println("IP: " + WiFi.localIP().toString());
        Serial.println("WebSocket: ws://" + WiFi.localIP().toString() + "/ws");
        Serial.println("=====================================================");
    } else {
        Serial.println("\nWiFi connection failed!");
        return;
    }
   
    // WebSocket setup
    ws.onEvent(onEvent);
    server.addHandler(&ws);
   
    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "404");
    });
   
    server.begin();
   
    Serial.println("Features:");
    Serial.println("  - Manual control (FORWARD/BACKWARD/LEFT/RIGHT/STOP)");
    Serial.println("  - Self-driving (NAV_START/NAV_STOP)");
    Serial.println("  - PID 90-degree turns");
    Serial.println("  - Ultrasonic obstacle avoidance");
    Serial.println("=====================================================");
    Serial.println("ROBOT READY!");
}

void loop() {
    ws.cleanupClients();
    checkSafetyConditions();
    monitorWiFiSignal();
   
    // Run self-driving if active
    if (selfDrivingActive) {
        selfDrivingLoop();
        delay(20);
    } else {
        delay(5);
    }
}


