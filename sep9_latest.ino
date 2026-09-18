#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>

// --- PIN DEFINITIONS ---
#define M1_STEP_PIN 18  // Motor 1 (Steering)
#define M1_DIR_PIN  19

#define M2_STEP_PIN 22  // Motor 2 (Drive)
#define M2_DIR_PIN  23

// --- WI-FI CREDENTIALS ---
const char* ssid = "ESP32_Motor_Controller";
const char* password = "password123";

WebServer server(80);
WebSocketsServer webSocket(81);
Preferences preferences;

// --- STATE TRACKING & LOCKS ---
int currentAngle = 0;                  // Tracks Motor 1 position (0, 270, -270 deg)
char activeKey = 0;                    // Tracks button holding control (0 = NONE)
bool isMovingMotor1 = false;           // Lock while Motor 1 is positioning
uint8_t m2DriveDirection = 0;          // 0 = STOP, 1 = CW, 2 = CCW

// --- NON-BLOCKING MOTOR 2 TIMING (1/16 SPEED) ---
unsigned long lastM2StepMicros = 0;
bool m2StepPinState = LOW;
const unsigned long M2_STEP_HALF_PERIOD_US = 12800; // 1/16 speed setting

// --- WATCHDOG TIMER FOR HOLDING BUTTONS ---
unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_TIMEOUT_MS = 1000; // Watchdog timeout threshold

// --- STATE LOCK FLAGS ---
volatile bool emergencyStopFlag = false; 
bool isRecording = false;               
bool isExecutingPath = false;           

// --- PATH RECORDING DATA STRUCTURES ---
struct PathStep {
    char key;
    unsigned long durationMs;
};

#define MAX_PATH_STEPS 50
PathStep pathBuffer[MAX_PATH_STEPS];
int recordedStepCount = 0;
unsigned long pressStartTime = 0;

// --- NVS FLASH STORAGE ---
void savePathToNVS() {
    preferences.begin("motorPath", false);
    preferences.putInt("count", recordedStepCount);
    if (recordedStepCount > 0) {
        preferences.putBytes("steps", pathBuffer, sizeof(PathStep) * recordedStepCount);
    }
    preferences.end();
}

void loadPathFromNVS() {
    preferences.begin("motorPath", true);
    recordedStepCount = preferences.getInt("count", 0);
    
    if (recordedStepCount > 0 && recordedStepCount <= MAX_PATH_STEPS) {
        preferences.getBytes("steps", pathBuffer, sizeof(PathStep) * recordedStepCount);
    } else {
        recordedStepCount = 0;
    }
    preferences.end();
}

// --- NON-BLOCKING MILLISECOND DELAY WITH E-STOP CHECK ---
bool interruptibleDelay(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        server.handleClient(); 
        webSocket.loop();
        if (emergencyStopFlag) return false;
        yield();
    }
    return true;
}

// --- MOTOR 1 POSITIONING FUNCTION ---
void moveMotor1ToAngle(int targetAngle) {
    if (emergencyStopFlag) return;
    
    int diff = targetAngle - currentAngle;
    if (diff == 0) return;

    digitalWrite(M2_STEP_PIN, LOW);
    m2StepPinState = LOW;

    digitalWrite(M1_DIR_PIN, (diff > 0) ? HIGH : LOW);
    
    int steps = (abs(diff) * 200) / 360; // 1:3 Gear ratio -> 150 steps for 270 deg
    
    unsigned long lastNetCheck = millis();

    for (int i = 0; i < steps; i++) {
        if (millis() - lastNetCheck > 20) {
            server.handleClient();
            webSocket.loop();
            lastNetCheck = millis();
        }

        if (emergencyStopFlag) {
            digitalWrite(M1_STEP_PIN, LOW);
            return;
        }

        digitalWrite(M1_STEP_PIN, HIGH);
        delayMicroseconds(4000); // 1/4 speed to prevent gear slipping
        digitalWrite(M1_STEP_PIN, LOW);
        delayMicroseconds(4000);
    }

    if (!emergencyStopFlag) {
        currentAngle = targetAngle;
    }
}

// --- BUTTON EVENT HANDLERS ---
void onKeyPress(char key) {
    if (emergencyStopFlag) {
        emergencyStopFlag = false;
    }

    if (isExecutingPath) return;
    if (key != '1' && key != '2' && key != '3' && key != '4') return;

    lastHeartbeatTime = millis();

    if (activeKey == 0) {
        activeKey = key;
        isMovingMotor1 = true;
        pressStartTime = millis();

        m2DriveDirection = 0; 

        int targetAngle = 0;
        uint8_t pendingDriveDir = 0;

        switch (key) {
            case '1': targetAngle = 0;    pendingDriveDir = 1; break; // Front
            case '2': targetAngle = -270;  pendingDriveDir = 2; break; // Left
            case '3': targetAngle = 0;    pendingDriveDir = 2; break; // Back
            case '4': targetAngle = 270; pendingDriveDir = 2; break; // Right
        }

        moveMotor1ToAngle(targetAngle);
        isMovingMotor1 = false;

        m2DriveDirection = pendingDriveDir;
        lastM2StepMicros = micros();
    }
}

void stopDriveMotor() {
    if (activeKey != 0) {
        unsigned long holdDuration = millis() - pressStartTime;
        m2DriveDirection = 0; 
        
        digitalWrite(M2_STEP_PIN, LOW);
        m2StepPinState = LOW;

        if (isRecording && !isExecutingPath && recordedStepCount < MAX_PATH_STEPS) {
            pathBuffer[recordedStepCount].key = activeKey;
            pathBuffer[recordedStepCount].durationMs = holdDuration;
            recordedStepCount++;
        }

        activeKey = 0;
    }
}

// --- WEBSOCKET EVENT LISTENER ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        String msg = String((char*)payload);
        
        if (msg.startsWith("P:")) { // Press or Hold
            char key = msg.charAt(2);
            onKeyPress(key);
        } 
        else if (msg.startsWith("R:")) { // Release
            stopDriveMotor();
        }
        else if (msg == "ESTOP") {
            emergencyStopFlag = true;
            stopDriveMotor();
        }
    }
}

// --- MOTOR DRIVE HELPER FOR PATH EXECUTION ---
void driveMotor2ForDuration(uint8_t direction, unsigned long durationMs) {
    if (direction == 0 || durationMs == 0 || emergencyStopFlag) return;

    digitalWrite(M2_DIR_PIN, (direction == 1) ? HIGH : LOW);
    unsigned long startTime = millis();

    while (millis() - startTime < durationMs) {
        server.handleClient();
        webSocket.loop();

        if (emergencyStopFlag) {
            digitalWrite(M2_STEP_PIN, LOW);
            return;
        }

        if (micros() - lastM2StepMicros >= M2_STEP_HALF_PERIOD_US) {
            lastM2StepMicros = micros();
            m2StepPinState = !m2StepPinState;
            digitalWrite(M2_STEP_PIN, m2StepPinState);
        }
        yield();
    }
    
    digitalWrite(M2_STEP_PIN, LOW);
    m2StepPinState = LOW;
}

// --- RUN FORWARD PATH 1 ---
bool runPath1() {
    if (recordedStepCount == 0 || isExecutingPath || isRecording) return false;
    
    emergencyStopFlag = false;
    isExecutingPath = true;

    for (int i = 0; i < recordedStepCount; i++) {
        if (emergencyStopFlag) break;

        char k = pathBuffer[i].key;
        unsigned long dur = pathBuffer[i].durationMs;

        int targetAngle = 0;
        uint8_t driveDir = 0;

        switch (k) {
            case '1': targetAngle = 0;    driveDir = 1; break;
            case '2': targetAngle = -270;  driveDir = 2; break;
            case '3': targetAngle = 0;    driveDir = 2; break;
            case '4': targetAngle = 270; driveDir = 2; break;
        }

        moveMotor1ToAngle(targetAngle);
        if (emergencyStopFlag) break;

        driveMotor2ForDuration(driveDir, dur);
        if (emergencyStopFlag) break;

        if (!interruptibleDelay(100)) break;
    }

    if (!emergencyStopFlag) {
        moveMotor1ToAngle(0);
    }
    
    bool completedSuccessfully = !emergencyStopFlag;
    isExecutingPath = false;
    return completedSuccessfully;
}

// --- REVERSE PATH 1 ---
bool reversePath1() {
    if (recordedStepCount == 0 || isExecutingPath || isRecording) return false;

    emergencyStopFlag = false;
    isExecutingPath = true;

    for (int i = recordedStepCount - 1; i >= 0; i--) {
        if (emergencyStopFlag) break;

        char originalKey = pathBuffer[i].key;
        unsigned long dur = pathBuffer[i].durationMs;

        char reverseKey = '1';
        int targetAngle = 0;
        uint8_t driveDir = 0;

        switch (originalKey) {
            case '1': reverseKey = '3'; break;
            case '2': reverseKey = '4'; break;
            case '3': reverseKey = '1'; break;
            case '4': reverseKey = '2'; break;
        }

        switch (reverseKey) {
            case '1': targetAngle = 0;    driveDir = 1; break;
            case '2': targetAngle = -270;  driveDir = 2; break;
            case '3': targetAngle = 0;    driveDir = 2; break;
            case '4': targetAngle = 270; driveDir = 2; break;
        }

        moveMotor1ToAngle(targetAngle);
        if (emergencyStopFlag) break;

        driveMotor2ForDuration(driveDir, dur);
        if (emergencyStopFlag) break;

        if (!interruptibleDelay(100)) break;
    }

    if (!emergencyStopFlag) {
        moveMotor1ToAngle(0);
    }

    bool completedSuccessfully = !emergencyStopFlag;
    isExecutingPath = false;
    return completedSuccessfully;
}

// --- HTML PAGE UI (WEBSOCKET INTEGRATED) ---
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>ESP32 WebSocket Controller</title>
  <style>
    body { font-family: Arial; text-align: center; background: #121212; color: white; touch-action: manipulation; user-select: none; }
    h2 { margin-top: 15px; margin-bottom: 5px; }
    #status { font-size: 14px; color: #ffca28; margin-bottom: 15px; }
    
    .estop-btn { 
      background: #ff0000; color: white; font-size: 20px; font-weight: bold; 
      border: 3px solid #ffffff; padding: 15px 30px; border-radius: 12px; 
      width: 80%; margin-bottom: 15px; box-shadow: 0 0 15px rgba(255,0,0,0.8);
    }
    .estop-btn:active { background: #b30000; }

    .grid { display: grid; grid-template-columns: 90px 90px 90px; grid-gap: 12px; justify-content: center; margin-top: 10px; }
    .btn { background: #008CBA; color: white; font-size: 18px; font-weight: bold; border: none; padding: 20px 0; border-radius: 12px; -webkit-touch-callout: none; }
    .btn:active { background: #005f73; }
    .empty { visibility: hidden; }
    
    .path-section { margin-top: 20px; border-top: 1px solid #333; padding-top: 15px; }
    .rec-btn { background: #e63946; color: white; padding: 12px 18px; font-weight: bold; border: none; border-radius: 8px; margin: 4px; }
    .stop-rec-btn { background: #6c757d; color: white; padding: 12px 18px; font-weight: bold; border: none; border-radius: 8px; margin: 4px; }
    .run-btn { background: #2a9d8f; color: white; padding: 12px 18px; font-weight: bold; border: none; border-radius: 8px; margin: 4px; }
    .rev-btn { background: #e76f51; color: white; padding: 12px 18px; font-weight: bold; border: none; border-radius: 8px; margin: 4px; }
  </style>
</head>
<body>
  <h2>ESP32 WebSocket Controller</h2>
  <div id="status">Connecting to WebSocket...</div>

  <button class="estop-btn" onclick="sendWsCmd('ESTOP')">EMERGENCY STOP</button>

  <div class="grid">
    <div class="empty"></div>
    <button class="btn" onmousedown="startHold('1')" onmouseup="stopHold()" ontouchstart="startHold('1')" ontouchend="stopHold()">1<br>FRONT</button>
    <div class="empty"></div>
    
    <button class="btn" onmousedown="startHold('2')" onmouseup="stopHold()" ontouchstart="startHold('2')" ontouchend="stopHold()">2<br>LEFT</button>
    <div class="empty"></div>
    <button class="btn" onmousedown="startHold('4')" onmouseup="stopHold()" ontouchstart="startHold('4')" ontouchend="stopHold()">4<br>RIGHT</button>
    
    <div class="empty"></div>
    <button class="btn" onmousedown="startHold('3')" onmouseup="stopHold()" ontouchstart="startHold('3')" ontouchend="stopHold()">3<br>BACK</button>
    <div class="empty"></div>
  </div>

  <div class="path-section">
    <button class="rec-btn" onclick="sendCmd('/startRec')">START REC</button>
    <button class="stop-rec-btn" onclick="sendCmd('/stopRec')">STOP REC</button>
    <br><br>
    <button class="run-btn" onclick="sendCmd('/runPath')">RUN PATH 1</button>
    <button class="rev-btn" onclick="sendCmd('/reversePath')">REVERSE PATH 1</button>
  </div>

  <script>
    let ws = null;
    let holdTimer = null;
    let currentKey = null;

    function initWebSocket() {
      ws = new WebSocket('ws://' + window.location.hostname + ':81/');
      
      ws.onopen = function() {
        document.getElementById('status').innerText = "Status: Connected (WebSocket)";
      };
      
      ws.onclose = function() {
        document.getElementById('status').innerText = "Status: Disconnected. Retrying...";
        setTimeout(initWebSocket, 1000);
      };
    }

    function startHold(k) {
      if (currentKey === k) return;
      stopHold();
      currentKey = k;

      sendWsCmd('P:' + k);

      // Low-overhead WebSocket ping every 50ms
      holdTimer = setInterval(() => {
        sendWsCmd('P:' + k);
      }, 50);
    }

    function stopHold() {
      if (holdTimer) {
        clearInterval(holdTimer);
        holdTimer = null;
      }
      if (currentKey) {
        sendWsCmd('R:' + currentKey);
        currentKey = null;
      }
    }

    function sendWsCmd(msg) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(msg);
      }
    }

    function sendCmd(endpoint) { 
      fetch(endpoint).then(res => res.text()).then(txt => {
        document.getElementById('status').innerText = "Status: " + txt;
      });
    }

    window.addEventListener('mouseup', stopHold);
    window.addEventListener('touchend', stopHold);
    window.addEventListener('touchcancel', stopHold);
    window.onload = initWebSocket;
  </script>
</body>
</html>
)rawliteral";

// --- ROUTES ---
void handleRoot() { server.send(200, "text/html", HTML_PAGE); }

void handleStartRec() {
    if (isExecutingPath) {
        server.send(200, "text/plain", "Cannot record while path is executing!");
        return;
    }
    isRecording = true;
    recordedStepCount = 0;
    server.send(200, "text/plain", "Recording Started...");
}

void handleStopRec() {
    if (!isRecording) {
        server.send(200, "text/plain", "Not currently recording.");
        return;
    }
    isRecording = false;
    savePathToNVS();
    String msg = "Recording Saved to Flash (" + String(recordedStepCount) + " steps)";
    server.send(200, "text/plain", msg);
}

void handleRunPath() {
    if (isRecording) {
        server.send(200, "text/plain", "Cannot run path during recording!");
        return;
    }
    if (recordedStepCount == 0) {
        server.send(200, "text/plain", "No path saved!");
        return;
    }
    
    bool success = runPath1();
    
    if (success) {
        server.send(200, "text/plain", "Path Completed");
    } else {
        server.send(200, "text/plain", "EMERGENCY STOP ACTIVATED!");
    }
}

void handleReversePath() {
    if (isRecording) {
        server.send(200, "text/plain", "Cannot reverse path during recording!");
        return;
    }
    if (recordedStepCount == 0) {
        server.send(200, "text/plain", "No path saved!");
        return;
    }
    
    bool success = reversePath1();
    
    if (success) {
        server.send(200, "text/plain", "Path Completed");
    } else {
        server.send(200, "text/plain", "EMERGENCY STOP ACTIVATED!");
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(M1_STEP_PIN, OUTPUT);
    pinMode(M1_DIR_PIN, OUTPUT);
    pinMode(M2_STEP_PIN, OUTPUT);
    pinMode(M2_DIR_PIN, OUTPUT);

    loadPathFromNVS();

    WiFi.softAP(ssid, password);
    server.enableDelay(false);

    server.on("/", handleRoot);
    server.on("/startRec", handleStartRec);
    server.on("/stopRec", handleStopRec);
    server.on("/runPath", handleRunPath);
    server.on("/reversePath", handleReversePath);
    server.begin();

    // Start WebSocket Server on Port 81
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

void loop() {
    server.handleClient();
    webSocket.loop();

    // WATCHDOG CHECK
    if (!isExecutingPath && activeKey != 0) {
        if (millis() - lastHeartbeatTime > HEARTBEAT_TIMEOUT_MS) {
            stopDriveMotor();
        }
    }

    // Continuous 1/16 speed drive stepping
    if (!isExecutingPath && m2DriveDirection != 0) {
        digitalWrite(M2_DIR_PIN, (m2DriveDirection == 1) ? HIGH : LOW);

        if (micros() - lastM2StepMicros >= M2_STEP_HALF_PERIOD_US) {
            lastM2StepMicros = micros();
            m2StepPinState = !m2StepPinState;
            digitalWrite(M2_STEP_PIN, m2StepPinState);
        }
    }
}