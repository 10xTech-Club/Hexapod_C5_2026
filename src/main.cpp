#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <espnow.h>            // ESP-NOW replaces WebSocketsClient for satellite link
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <LittleFS.h>
#include <DHT.h>

// ── Debug Logging ─────────────────────────────────────────────────────────────
// Controlled by the PlatformIO build environment — do NOT hard-code here.
//   nodemcuv2       (production) → DEBUG_LOGGING=0 : pure no-op, zero overhead
//   nodemcuv2_debug (debug)      → DEBUG_LOGGING=1 : Serial + WebSocket stream
//
// When DEBUG_LOGGING=1:
//   LOG/LOGF → Serial.println/printf + wsLogEnqueue (WS dashboard).
//   CORE_DEBUG_LEVEL=3 + DEBUG_ESP_PORT=Serial expose WiFi/ESP-NOW/WS internals.
//
// When DEBUG_LOGGING=0 (default):
//   LOG/LOGF expand to ((void)0) — no String built, no UART write, no WS traffic.
//   CORE_DEBUG_LEVEL/DEBUG_ESP_PORT are not defined → framework stays completely silent.
#ifndef DEBUG_LOGGING
#  define DEBUG_LOGGING 0   // overridden by -DDEBUG_LOGGING=1 in platformio.ini
#endif

#if DEBUG_LOGGING
  #define LOG(msg)   do { Serial.println(msg); wsLogEnqueue(String(msg).c_str()); } while(0)
  #define LOGF(...)  do { Serial.printf(__VA_ARGS__); \
                          char _b[96]; snprintf(_b, sizeof(_b), __VA_ARGS__); wsLogEnqueue(_b); } while(0)
#else
  #define LOG(msg)   ((void)0)   // compile-time no-op — no String built, no Serial write
  #define LOGF(...)  ((void)0)
#endif

// Function declarations
void setServoAngle(int servo, int angle);
void setCamServoAngle(int channel, int angle);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void sendServoUpdate(uint8_t clientNum, int servo, int angle, int pwm);
void broadcastServoUpdate(int servo, int angle, int pwm);
void moveForward();
void stopMovement();
void executeWalkStep(int step);
void generateWalkPattern(int walkPattern[4][18], bool backward);
void moveBackward();
void waveHand();
void danceMove();
void waveAll_Left();
void waveAll_Right();

// ESP-NOW satellite declarations
void initEspNow();
void onEspNowSent(uint8_t *mac, uint8_t status);
void onEspNowRecv(uint8_t *mac, uint8_t *data, uint8_t len);
void sendTelemetryEspNow();
void broadcastSatStatus();

// Debug log streaming declarations (only compiled when DEBUG_LOGGING is true)
#if DEBUG_LOGGING
void wsLogEnqueue(const char* msg);
void wsLogFlush();
#endif

// WiFi Access Point Credentials
const char *ssid = "Hexapod-AP";
const char *password = "hexapod123";

// Static IP configuration for AP — using 192.168.5.x to avoid conflict with satellite 192.168.4.x
IPAddress local_IP(192, 168, 5, 1);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);

// ─── Satellite: ESP-NOW ────────────────────────────────────────────────────
// Both hexapod AP and satellite AP are pinned to channel 6.
// ESP-NOW works at the WiFi MAC layer — no TCP connection, no reconnection logic.
// Broadcast MAC means no MAC address discovery or pairing step required.

#define ESPNOW_MSG_TELEMETRY  0x01   // hexapod → satellite
#define ESPNOW_MSG_COMMAND    0x02   // satellite → hexapod
#define ESPNOW_MSG_CHAT       0x03   // bidirectional
#define ESPNOW_DEVICE_HEXAPOD 0x01   // device-type tag in telemetry[1]

#define DEVICE_ID_SATELLITE 0x01
#define DEVICE_ID_ROVER 0x02
#define DEVICE_ID_HEXAPOD 0x03
#define DEVICE_ID_BROADCAST 0xFF

#define ESPNOW_PKT_CHAT 0x91
#define ESPNOW_PKT_COMMAND 0x92
#define ESPNOW_PKT_MIRROR_LOG 0x93

struct __attribute__((packed)) EspNowHeader {
  uint8_t sender_id;
  uint8_t receiver_id;
  uint8_t packet_type;
  uint16_t sequence_no;
};

struct __attribute__((packed)) EspNowCommandV2 {
  EspNowHeader header;
  char cmd[16];
};

struct __attribute__((packed)) EspNowChatV2 {
  EspNowHeader header;
  char from[16];
  char to[16];
  char msg[64];
};

struct __attribute__((packed)) EspNowMirrorLogV2 {
  EspNowHeader header;
  uint8_t original_sender;
  uint8_t original_receiver;
  uint8_t original_packet_type;
  char msg[64];
};

struct __attribute__((packed)) EspNowTelemetry {
  uint8_t  msgType;           // ESPNOW_MSG_TELEMETRY
  uint8_t  deviceType;        // ESPNOW_DEVICE_HEXAPOD
  float    temp;
  float    hum;
  int16_t  gas;
  int16_t  aqi;
  int16_t  co2;
  int8_t   rssi;
  uint8_t  walking;
  uint8_t  step;
  uint8_t  clients;
  int8_t   legs[6];           // coxa angle offset from 90°
};  // 26 bytes — well under ESP-NOW 250-byte limit

struct __attribute__((packed)) EspNowCommand {
  uint8_t  msgType;           // ESPNOW_MSG_COMMAND
  char     cmd[16];           // "forward" / "backward" / "left" / "right" / "stop"
};  // 17 bytes

struct __attribute__((packed)) EspNowChat {
  uint8_t  msgType;           // ESPNOW_MSG_CHAT
  char     from[16];
  char     to[16];            // intended recipient: "satellite" | "hexapod" | "rover" | "all"
  char     msg[64];
};  // 97 bytes

// ── SATELLITE MAC ADDRESS — FILL THIS IN ──────────────────────────────────────
// Flash the satellite, open Serial Monitor, read:
//   "[ESP-NOW] Satellite AP MAC: XX:XX:XX:XX:XX:XX"
// Paste those 6 bytes here, then reflash the hexapod.
static uint8_t satMac[6] = {0x3C, 0x8A, 0x1F, 0x7E, 0x33, 0x55}; // Satellite ESP32 AP MAC
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t localSeqNo = 0;

static uint16_t nextSequence() {
  localSeqNo++;
  return localSeqNo;
}

bool          satEnabled         = false;   // user toggles via dashboard
unsigned long lastSatTelemetry   = 0;
const unsigned long SAT_TELEMETRY_MS = 2000;

// Deferred from onEspNowRecv (WiFi task context) → processed safely from loop()
// volatile ensures loop() always re-reads the flag rather than caching it.
static char          pendingEspNowCmd[16]      = "";
static volatile bool hasPendingEspNowCmd       = false;
static char          pendingEspNowChatFrom[16] = "";
static char          pendingEspNowChatMsg[64]  = "";
static volatile bool hasPendingEspNowChat      = false;

// Create PCA9685 objects
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);  // Right side (servos 0-17)
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41);  // Left side (servos 18-35)

// Servers
WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266WebServer server(80);

// EEPROM address for height offset
#define EEPROM_HEIGHT_ADDR 0

// Servo configuration
#define SERVOMIN  195
#define SERVOMAX  431
#define SERVO_FREQ 50

// DHT11 sensor
#define DHTPIN D6
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Gas sensor
#define GAS_PIN A0

// Camera servo channels on PCA1 (0x40)
#define CAM_PAN_CH  10   // PCA1 channel 9
#define CAM_TILT_CH 9  // PCA1 channel 10
int camPanAngle = 90;
int camTiltAngle = 90;

// Sensor reading interval
unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 5000; // 5 seconds — least priority, reduces loop overhead

// Cached sensor values — read once, used by both local broadcast and satellite telemetry
float g_temp = 0.0;
float g_hum  = 0.0;
int   g_gas  = 0;
int   g_aqi  = 0;
int   g_co2  = 400;
int   g_rssi = 0;

// Current servo positions and PWM values (only using 18 servos total)
int servoPositions[18];
int servoPWMValues[18];

int heightOffset = 0;
int stride = 40;
bool walkingForward = true;
enum MovementDirection {
  FORWARD,
  BACKWARD,
  TURN_RIGHT,
  TURN_LEFT
};

MovementDirection currentDirection = FORWARD;

// Deferred gesture actions — set from the WebSocket callback, executed from loop().
// Wave/dance functions block for 3-6 seconds; calling them directly inside
// webSocketEvent() starves the WebSocket library's internal state machine and
// triggers hardware WDT resets. Setting a flag here and running from loop()
// keeps the event callback fast and lets delay()->yield() feed the watchdog
// in the correct SDK context.
enum PendingGesture {
  GESTURE_NONE,
  GESTURE_WAVE_HAND,
  GESTURE_DANCE_MOVE,
  GESTURE_WAVE_ALL_LEFT,
  GESTURE_WAVE_ALL_RIGHT
};
PendingGesture pendingGesture = GESTURE_NONE;

// Movement control variables
bool isWalking = false;
// Set by stopMovement(); cleared and broadcast from loop() outside any WS callback.
bool pendingStopBroadcast = false;
// Set whenever satStatus needs broadcasting from inside a WS callback; sent in loop().
bool pendingSatStatusBroadcast = false;

// Circular queue for deferred WebSocket log messages (DEBUG_LOGGING only).
// Entries are written from any context (including callbacks) and flushed
// safely from loop() via broadcastTXT — no re-entrancy risk.
#if DEBUG_LOGGING
static char    wsLogQueue[4][96];   // 4 slots × 95 chars + null
static uint8_t wsLogHead = 0;
static uint8_t wsLogTail = 0;
#endif

unsigned long lastStepTime = 0;
int currentStep = 0;
unsigned long stepDelay = 500;

// Hexapod leg mapping:
// PCA2 (0x41) - Left side: Legs 1,2,3 (channels 0-8)
// PCA1 (0x40) - Right side: Legs 4,5,6 (channels 0-8)
// Each leg: Coxa, Femur, Tibia
// Left side (PCA2): 0-8 (Leg1: 0,1,2  Leg2: 3,4,5  Leg3: 6,7,8)
// Right side (PCA1): 9-17 (Leg4: 9,10,11  Leg5: 12,13,14  Leg6: 15,16,17)

int FemurHeight = -60;

void generateWalkPattern(int walkPattern[4][18], bool backward = false) {
  int base = 90;
  int dir = backward ? -1 : 1;
  int s = stride * dir;

  // ---- Step 1 ---- Lift Group A (L1,L3,R5), push Group B (L2,R4,R6) opposite
  walkPattern[0][0] = base + s;
  walkPattern[0][1] = base - FemurHeight;
  walkPattern[0][2] = base;

  walkPattern[0][3] = base - s;
  walkPattern[0][4] = base;
  walkPattern[0][5] = base;

  walkPattern[0][6] = base + s;
  walkPattern[0][7] = base - FemurHeight;
  walkPattern[0][8] = base;

  walkPattern[0][9]  = base + s;
  walkPattern[0][10] = base;
  walkPattern[0][11] = base;

  walkPattern[0][12] = base - s;
  walkPattern[0][13] = base + FemurHeight;
  walkPattern[0][14] = base;

  walkPattern[0][15] = base + s;
  walkPattern[0][16] = base;
  walkPattern[0][17] = base;

  // ---- Step 2 ---- Lower Group A
  walkPattern[1][0] = base + s;
  walkPattern[1][1] = base;
  walkPattern[1][2] = base;

  walkPattern[1][3] = base - s;
  walkPattern[1][4] = base;
  walkPattern[1][5] = base;

  walkPattern[1][6] = base + s;
  walkPattern[1][7] = base;
  walkPattern[1][8] = base;

  walkPattern[1][9]  = base + s;
  walkPattern[1][10] = base;
  walkPattern[1][11] = base;

  walkPattern[1][12] = base - s;
  walkPattern[1][13] = base;
  walkPattern[1][14] = base;

  walkPattern[1][15] = base + s;
  walkPattern[1][16] = base;
  walkPattern[1][17] = base;

  // ---- Step 3 ---- Lift Group B (L2,R4,R6), push Group A (L1,L3,R5) opposite
  walkPattern[2][0] = base - s;
  walkPattern[2][1] = base;
  walkPattern[2][2] = base;

  walkPattern[2][3] = base + s;
  walkPattern[2][4] = base - FemurHeight;
  walkPattern[2][5] = base;

  walkPattern[2][6] = base - s;
  walkPattern[2][7] = base;
  walkPattern[2][8] = base;

  walkPattern[2][9]  = base - s;
  walkPattern[2][10] = base + FemurHeight;
  walkPattern[2][11] = base;

  walkPattern[2][12] = base + s;
  walkPattern[2][13] = base;
  walkPattern[2][14] = base;

  walkPattern[2][15] = base - s;
  walkPattern[2][16] = base + FemurHeight;
  walkPattern[2][17] = base;

  // ---- Step 4 ---- Lower Group B
  walkPattern[3][0] = base - s;
  walkPattern[3][1] = base;
  walkPattern[3][2] = base;

  walkPattern[3][3] = base + s;
  walkPattern[3][4] = base;
  walkPattern[3][5] = base;

  walkPattern[3][6] = base - s;
  walkPattern[3][7] = base;
  walkPattern[3][8] = base;

  walkPattern[3][9]  = base - s;
  walkPattern[3][10] = base;
  walkPattern[3][11] = base;

  walkPattern[3][12] = base + s;
  walkPattern[3][13] = base;
  walkPattern[3][14] = base;

  walkPattern[3][15] = base - s;
  walkPattern[3][16] = base;
  walkPattern[3][17] = base;
}

void generateTurnPattern(int walkPattern[4][18], bool turnLeft) {
  int base = 90;
  int leftStride = turnLeft ? stride : -stride;
  int rightStride = turnLeft ? stride : -stride;

  // ---- Step 1 ----
  walkPattern[0][0] = base + leftStride;
  walkPattern[0][1] = base - FemurHeight;
  walkPattern[0][2] = base;
  walkPattern[0][3] = base;
  walkPattern[0][4] = base;
  walkPattern[0][5] = base;
  walkPattern[0][6] = base + leftStride;
  walkPattern[0][7] = base - FemurHeight;
  walkPattern[0][8] = base;
  walkPattern[0][9] = base;
  walkPattern[0][10] = base;
  walkPattern[0][11] = base;
  walkPattern[0][12] = base + rightStride;
  walkPattern[0][13] = base + FemurHeight;
  walkPattern[0][14] = base;
  walkPattern[0][15] = base;
  walkPattern[0][16] = base;
  walkPattern[0][17] = base;

  // ---- Step 2 ----
  walkPattern[1][0] = base + leftStride;
  walkPattern[1][1] = base;
  walkPattern[1][2] = base;
  walkPattern[1][3] = base;
  walkPattern[1][4] = base;
  walkPattern[1][5] = base;
  walkPattern[1][6] = base + leftStride;
  walkPattern[1][7] = base;
  walkPattern[1][8] = base;
  walkPattern[1][9] = base;
  walkPattern[1][10] = base;
  walkPattern[1][11] = base;
  walkPattern[1][12] = base + rightStride;
  walkPattern[1][13] = base;
  walkPattern[1][14] = base;
  walkPattern[1][15] = base;
  walkPattern[1][16] = base;
  walkPattern[1][17] = base;

  // ---- Step 3 ----
  walkPattern[2][0] = base;
  walkPattern[2][1] = base;
  walkPattern[2][2] = base;
  walkPattern[2][3] = base + leftStride;
  walkPattern[2][4] = base - FemurHeight;
  walkPattern[2][5] = base;
  walkPattern[2][6] = base;
  walkPattern[2][7] = base;
  walkPattern[2][8] = base;
  walkPattern[2][9] = base + rightStride;
  walkPattern[2][10] = base + FemurHeight;
  walkPattern[2][11] = base;
  walkPattern[2][12] = base;
  walkPattern[2][13] = base;
  walkPattern[2][14] = base;
  walkPattern[2][15] = base + rightStride;
  walkPattern[2][16] = base + FemurHeight;
  walkPattern[2][17] = base;

  // ---- Step 4 ----
  walkPattern[3][0] = base;
  walkPattern[3][1] = base;
  walkPattern[3][2] = base;
  walkPattern[3][3] = base + leftStride;
  walkPattern[3][4] = base;
  walkPattern[3][5] = base;
  walkPattern[3][6] = base;
  walkPattern[3][7] = base;
  walkPattern[3][8] = base;
  walkPattern[3][9] = base + rightStride;
  walkPattern[3][10] = base;
  walkPattern[3][11] = base;
  walkPattern[3][12] = base;
  walkPattern[3][13] = base;
  walkPattern[3][14] = base;
  walkPattern[3][15] = base + rightStride;
  walkPattern[3][16] = base;
  walkPattern[3][17] = base;
}

void moveTurnRight() {
  if (!isWalking) {
    isWalking = true;
    currentDirection = TURN_RIGHT;
    currentStep = 0;
    lastStepTime = millis();
    LOG("Started turning right");
  }
}

void moveTurnLeft() {
  if (!isWalking) {
    isWalking = true;
    currentDirection = TURN_LEFT;
    currentStep = 0;
    lastStepTime = millis();
    LOG("Started turning left");
  }
}

// ---- LittleFS file serving ----
String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  if (filename.endsWith(".css"))  return "text/css";
  if (filename.endsWith(".js"))   return "application/javascript";
  if (filename.endsWith(".json")) return "application/json";
  if (filename.endsWith(".png"))  return "image/png";
  if (filename.endsWith(".ico"))  return "image/x-icon";
  return "text/plain";
}

bool handleFileRead(String path) {
  if (path.endsWith("/")) path += "index.html";
  String contentType = getContentType(path);
  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

// ── Debug log implementation ──────────────────────────────────────────────────
#if DEBUG_LOGGING
// Enqueues a message for WebSocket delivery.
// Serial output is already handled by the LOG/LOGF macros — no duplication here.
// Silently drops when the 4-slot circular queue is full.
void wsLogEnqueue(const char* msg) {
  uint8_t next = (wsLogHead + 1) & 3;   // circular mod-4
  if (next != wsLogTail) {              // drop silently when full
    strncpy(wsLogQueue[wsLogHead], msg, 95);
    wsLogQueue[wsLogHead][95] = '\0';
    wsLogHead = next;
  }
}

// Called from loop() only — safe to call broadcastTXT here.
void wsLogFlush() {
  uint8_t count = 0;
  while (wsLogTail != wsLogHead && count++ < 4) {
    StaticJsonDocument<256> doc;
    char out[160];
    doc["type"] = "log";
    doc["msg"]  = wsLogQueue[wsLogTail];
    serializeJson(doc, out, sizeof(out));
    webSocket.broadcastTXT(out);
    wsLogTail = (wsLogTail + 1) & 3;
  }
}
#endif

void setup() {
  Serial.begin(115200);
  delay(100);  // let UART settle before first print

  // ── Disable software WDT ─────────────────────────────────────────────────
  // The ESP8266 software watchdog fires after ~3.2 s without a system yield.
  // Gesture functions and I2C servo bursts can approach this limit, causing
  // phantom resets. Disabling it here removes that trigger completely.
  // The hardware WDT (~8 s) cannot be disabled; it is still fed via delay()
  // which calls yield() → background SDK tasks on every tick.
  ESP.wdtDisable();

  // ── Reset Reason Diagnostic ──────────────────────────────────────────────
  // Log why the chip restarted. Helps distinguish WDT resets (software/hardware
  // watchdog), exception crashes, power-on, and OTA-initiated resets.
  LOG("\n===== HEXAPOD BOOT =====");
  LOG("Reset reason : " + ESP.getResetReason());
  LOG("Reset info   : " + ESP.getResetInfo());
  LOG("Free heap    : " + String(ESP.getFreeHeap()) + " bytes");
  LOG("Chip ID      : " + String(ESP.getChipId(), HEX));
  LOG("Flash size   : " + String(ESP.getFlashChipRealSize() / 1024) + " KB");

  // When the previous run crashed with an exception (hardware fault / null pointer /
  // stack overflow etc.), print the low-level registers so the exact fault address
  // can be decoded with the ELF symbol table.
  const rst_info* ri = ESP.getResetInfoPtr();
  if (ri && ri->reason == REASON_EXCEPTION_RST) {
    LOG("!!! EXCEPTION CRASH DETECTED !!!");
    LOGF("  Exception cause : %d\n",       ri->exccause);   // 0=illegal instr, 28=LoadStoreAlignmentCause …
    LOGF("  EPC1 (fault PC) : 0x%08X\n",  ri->epc1);       // program counter at crash
    LOGF("  EPC2            : 0x%08X\n",  ri->epc2);
    LOGF("  EPC3            : 0x%08X\n",  ri->epc3);
    LOGF("  Exception vaddr : 0x%08X\n",  ri->excvaddr);    // bad memory address accessed
    LOGF("  DEPC            : 0x%08X\n",  ri->depc);
  } else if (ri) {
    // 0=Power-on, 1=HW watchdog, 3=SW watchdog, 4=SW restart/OTA, 5=Deep-sleep, 6=Ext reset
    LOGF("  Raw reset code  : %d\n", ri->reason);
  }
  LOG("========================");

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    LOG("LittleFS mount failed!");
  } else {
    LOG("LittleFS mounted successfully");
  }

  // Initialize EEPROM and restore height offset
  EEPROM.begin(512);
  heightOffset = (int8_t)EEPROM.read(EEPROM_HEIGHT_ADDR);
  if (heightOffset < -30 || heightOffset > 30) heightOffset = 0;
  LOG("Restored height offset: " + String(heightOffset));

  LOG("Starting Hexapod Controller...");

  // Wait for PCA9685 to stabilize after power-on before initializing I2C.
  // On a cold boot the PCA9685 needs ~100-500 ms to complete its internal
  // power-on reset before it will ACK I2C transactions reliably.
  // Without this delay setup() silently fails and servos don't move until
  // the MCU is manually reset (by which time the PCA9685 is already stable).
  delay(500);

  // Initialize I2C
  Wire.begin(D2, D1);
  LOG("I2C initialized on pins SDA=D2, SCL=D1");

  // Initialize PCA9685 controllers
  pca1.begin();
  pca2.begin();
  pca1.setOscillatorFrequency(27000000);
  pca2.setOscillatorFrequency(27000000);
  pca1.setPWMFreq(SERVO_FREQ);
  pca2.setPWMFreq(SERVO_FREQ);
  LOG("PCA9685 controllers initialized");

  // Initialize all leg servos to 90 degrees
  for (int i = 0; i < 18; i++) {
    setServoAngle(i, 90);
    servoPositions[i] = 90;
    delay(10);
  }
  LOG("All 18 leg servos initialized to 90 degrees");

  // Initialize camera servos to center
  setCamServoAngle(CAM_PAN_CH, 90);
  setCamServoAngle(CAM_TILT_CH, 90);
  LOG("Camera pan/tilt servos initialized");

  // Initialize DHT sensor
  dht.begin();
  LOG("DHT11 sensor initialized on pin D7");

  // ── WiFi AP on channel 6 ──────────────────────────────────────────────────
  // Channel 6 matches the satellite AP. ESP-NOW requires both peers on the same
  // channel. With ESP-NOW replacing the WebSocket client link, STA mode is no
  // longer needed — AP-only dedicates the full radio to the dashboard browser.
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password, 6);   // pinned to channel 6

  LOG("WiFi AP started: " + String(ssid));
  LOGF("AP IP: %s", WiFi.softAPIP().toString().c_str());
  // Print AP MAC so you can copy it into the satellite's HEXAPOD_MAC field.
  uint8_t apMac[6];
  WiFi.softAPmacAddress(apMac);
  Serial.printf("[ESP-NOW] Hexapod AP MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                apMac[0], apMac[1], apMac[2], apMac[3], apMac[4], apMac[5]);

  // Serve files from LittleFS
  server.onNotFound([]() {
    if (!handleFileRead(server.uri())) {
      server.send(404, "text/plain", "Not Found");
    }
  });

  server.begin();
  LOG("HTTP server started on port 80");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  // Keep idle connections healthy and drop stale sockets before they
  // become half-open and crash inside the networking stack.
  webSocket.enableHeartbeat(15000, 3000, 2);
  LOG("WebSocket server started on port 81");

  // OTA (Over-The-Air) firmware update
  ArduinoOTA.setHostname("Hexapod-AP");
  ArduinoOTA.setPassword("hexapod123");
  ArduinoOTA.onStart([]() {
    Serial.println("OTA: Starting update...");   // Serial only — WS not safe during OTA
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA: Done.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA: %u%%\r", progress * 100 / total);   // high-frequency, Serial only
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });
  ArduinoOTA.begin();
  LOG("OTA ready - hostname: Hexapod-AP");

  // ── ESP-NOW init — must come after WiFi.softAP() ─────────────────────────
  initEspNow();

  LOG("=================================");
  LOG("Hexapod Controller Ready!");
  LOG("=================================");
}

void loop() {
  ArduinoOTA.handle();
  webSocket.loop();
  server.handleClient();

  // Flush any queued log messages to the WebSocket dashboard.
  // Must run after webSocket.loop() returns so we are outside any callback.
#if DEBUG_LOGGING
  wsLogFlush();
#endif

  // ─── Deferred stop broadcast ──────────────────────────────────────────────
  // stopMovement() is always called from inside webSocketEvent() which is inside
  // webSocket.loop(). Sending broadcastTXT() from there is re-entrant and causes
  // the sys-context software WDT crash. Instead, stopMovement() just sets a flag;
  // we broadcast here, safely outside any callback, on the very next loop tick.
  if (pendingStopBroadcast) {
    pendingStopBroadcast = false;
    for (int i = 0; i < 18; i++) {
      broadcastServoUpdate(i, servoPositions[i], servoPWMValues[i]);
    }
    StaticJsonDocument<200> stopDoc;
    stopDoc["type"]    = "walkStatus";
    stopDoc["walking"] = false;
    stopDoc["step"]    = 0;
    String stopMsg;
    serializeJson(stopDoc, stopMsg);
    webSocket.broadcastTXT(stopMsg);
  }

  // ─── Deferred satellite status broadcast ─────────────────────────────────
  // connectSatellite / disconnectSatellite handlers and satWebSocketEvent all
  // fire from inside a webSocket.loop() or satWs.loop() callback. Calling
  // webSocket.broadcastTXT() from there is re-entrant and smashes the stack
  // (Exception 5). Broadcast here, safely outside any callback.
  if (pendingSatStatusBroadcast) {
    pendingSatStatusBroadcast = false;
    broadcastSatStatus();
  }

  // ─── Process ESP-NOW command received from satellite ──────────────────────
  // onEspNowRecv() runs in the WiFi task — it only sets the flag and copies the
  // string. We apply the movement change here, safely outside any callback.
  if (hasPendingEspNowCmd) {
    hasPendingEspNowCmd = false;
    LOG("[SAT] Command: " + String(pendingEspNowCmd));
    if (strcmp(pendingEspNowCmd, "forward") == 0) {
      currentDirection = FORWARD;
      if (!isWalking) { isWalking = true; currentStep = 0; lastStepTime = millis(); }
    } else if (strcmp(pendingEspNowCmd, "backward") == 0) {
      currentDirection = BACKWARD;
      if (!isWalking) { isWalking = true; currentStep = 0; lastStepTime = millis(); }
    } else if (strcmp(pendingEspNowCmd, "left") == 0) {
      currentDirection = TURN_LEFT;
      if (!isWalking) { isWalking = true; currentStep = 0; lastStepTime = millis(); }
    } else if (strcmp(pendingEspNowCmd, "right") == 0) {
      currentDirection = TURN_RIGHT;
      if (!isWalking) { isWalking = true; currentStep = 0; lastStepTime = millis(); }
    } else if (strcmp(pendingEspNowCmd, "stop") == 0) {
      stopMovement();
    }
  }

  // ─── Relay ESP-NOW chat from satellite to dashboard browser ───────────────
  if (hasPendingEspNowChat) {
    hasPendingEspNowChat = false;
    StaticJsonDocument<256> relay;
    relay["type"] = "chatReceived";
    relay["from"] = pendingEspNowChatFrom;
    relay["msg"]  = pendingEspNowChatMsg;
    String relayMsg;
    serializeJson(relay, relayMsg);
    webSocket.broadcastTXT(relayMsg);
    LOG("[SAT] Chat from " + String(pendingEspNowChatFrom) + ": " + String(pendingEspNowChatMsg));
  }

  // ─── Send periodic telemetry to satellite via ESP-NOW ─────────────────────
  // No connection state to check — ESP-NOW is connectionless.
  if (satEnabled && millis() - lastSatTelemetry > SAT_TELEMETRY_MS) {
    lastSatTelemetry = millis();
    sendTelemetryEspNow();
  }

  // Handle walking movement — no WebSocket broadcast during walking;
  // every broadcast costs radio airtime and reduces camera stream smoothness.
  if (isWalking) {
    if (millis() - lastStepTime > stepDelay) {
      executeWalkStep(currentStep);
      currentStep = (currentStep + 1) % 4;
      lastStepTime = millis();
    }
  }

  // Sensor refresh — always read so globals stay current for satellite telemetry,
  // but only broadcast to the dashboard when not walking (saves radio for camera).
  if (millis() - lastSensorRead > sensorInterval) {
    lastSensorRead = millis();

    float temp = dht.readTemperature();
    float hum  = dht.readHumidity();
    g_temp = isnan(temp) ? 0.0 : temp;
    g_hum  = isnan(hum)  ? 0.0 : hum;
    g_gas  = analogRead(GAS_PIN);
    {
      int raw = g_gas;
      float aqi;
      if      (raw < 300) aqi = raw / 300.0f * 50.0f;
      else if (raw < 500) aqi = 50.0f  + (raw - 300) / 200.0f * 50.0f;
      else if (raw < 700) aqi = 100.0f + (raw - 500) / 200.0f * 50.0f;
      else if (raw < 900) aqi = 150.0f + (raw - 700) / 200.0f * 50.0f;
      else                aqi = fminf(200.0f + (raw - 900) / 124.0f * 100.0f, 500.0f);

      float co2;
      if      (raw < 200) co2 = 400.0f;
      else if (raw < 500) co2 = 400.0f  + (raw - 200) / 300.0f * 400.0f;
      else if (raw < 800) co2 = 800.0f  + (raw - 500) / 300.0f * 1200.0f;
      else                co2 = fminf(2000.0f + (raw - 800) / 224.0f * 3000.0f, 9999.0f);

      g_aqi = (int)roundf(aqi);
      g_co2 = (int)roundf(co2);
    }
    g_rssi = WiFi.RSSI();

    if (!isWalking) {
      StaticJsonDocument<256> doc;
      doc["type"] = "sensorData";
      doc["temp"] = g_temp;
      doc["hum"]  = g_hum;
      doc["gas"]  = g_gas;
      doc["aqi"]  = g_aqi;
      doc["co2"]  = g_co2;
      doc["rssi"] = g_rssi;

      String message;
      serializeJson(doc, message);
      webSocket.broadcastTXT(message);
    }
  }

  // Execute deferred gestures from loop() — never from the WebSocket callback.
  if (!isWalking && pendingGesture != GESTURE_NONE) {
    PendingGesture g = pendingGesture;
    pendingGesture = GESTURE_NONE;
    switch (g) {
      case GESTURE_WAVE_HAND:     waveHand();     break;
      case GESTURE_DANCE_MOVE:    danceMove();    break;
      case GESTURE_WAVE_ALL_LEFT: waveAll_Left(); break;
      case GESTURE_WAVE_ALL_RIGHT:waveAll_Right();break;
      default: break;
    }
  }

  // Smart status logging: print on meaningful changes, with occasional summary.
  static unsigned long lastStatusLog = 0;
  static int lastClientCount = -1;
  static bool lastWalkingState = false;
  const unsigned long STATUS_LOG_INTERVAL_MS = 60000;  // periodic summary
  unsigned long now = millis();
  int clientCount = webSocket.connectedClients();
  bool clientChanged = (clientCount != lastClientCount);
  bool walkingChanged = (isWalking != lastWalkingState);

  if (clientChanged || walkingChanged || (now - lastStatusLog >= STATUS_LOG_INTERVAL_MS)) {
    char _sBuf[72];
    if (isWalking)
      snprintf(_sBuf, sizeof(_sBuf), "[T+%lus] Status: clients=%d, walking=yes, step=%d/4",
               now / 1000UL, clientCount, currentStep + 1);
    else
      snprintf(_sBuf, sizeof(_sBuf), "[T+%lus] Status: clients=%d, walking=no",
               now / 1000UL, clientCount);
    LOG(_sBuf);

    lastStatusLog = now;
    lastClientCount = clientCount;
    lastWalkingState = isWalking;
  }
}

void setServoAngle(int servo, int angle) {
  angle = constrain(angle, 0, 180);
  int jointNum = servo % 3;

  int adjustedAngle = angle;
  if (jointNum == 1 || jointNum == 2) {
    // PCA2 (left, servos 0-8) and PCA1 (right, servos 9-17) are physically
    // mirrored, so the height offset must be negated for the right side.
    int heightSign = (servo < 9) ? 1 : -1;
    adjustedAngle += heightSign * heightOffset;
    adjustedAngle = constrain(adjustedAngle, 0, 180);
  }

  int finalAngle = adjustedAngle;
  int pwmValue = map(finalAngle, 0, 180, SERVOMIN, SERVOMAX);

  if (servo < 9) {
    pca2.setPWM(servo, 0, pwmValue);
  } else {
    pca1.setPWM(servo - 9, 0, pwmValue);
  }

  servoPositions[servo] = angle;  // store base angle; height offset is applied at write time only
  servoPWMValues[servo] = pwmValue;
}

void setCamServoAngle(int channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pwmValue = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pca1.setPWM(channel, 0, pwmValue);  // Camera servos on PCA1
}

void executeWalkStep(int step) {
  Serial.println("Executing walk step: " + String(step + 1));

  int targetPattern[4][18];

  if (currentDirection == FORWARD) {
    generateWalkPattern(targetPattern, false);
  } else if (currentDirection == BACKWARD) {
    generateWalkPattern(targetPattern, true);
  } else if (currentDirection == TURN_RIGHT) {
    generateTurnPattern(targetPattern, false);
  } else if (currentDirection == TURN_LEFT) {
    generateTurnPattern(targetPattern, true);
  }

  const int interpSteps = 10;

  for (int s = 1; s <= interpSteps; s++) {
    for (int i = 0; i < 18; i++) {
      int startAngle = servoPositions[i];
      int endAngle   = targetPattern[step][i];
      int interpAngle = startAngle + (endAngle - startAngle) * s / interpSteps;
      setServoAngle(i, interpAngle);
    }
    // Feed watchdogs then yield via delay(). Do NOT call webSocket.loop() or
    // server.handleClient() here — doing so allows webSocketEvent() to fire
    // from inside the interpolation loop. stopMovement() then calls broadcastTXT()
    // in a re-entrant call inside webSocket.loop(), which starves the WiFi MAC
    // and triggers the SDK's sys-context software WDT (ESP.wdtDisable() cannot
    // suppress this). delay() → yield() feeds all WDTs safely without re-entry.
    ESP.wdtFeed();
    delay(stepDelay / interpSteps);
  }
  // Servo position broadcasts removed from here — broadcasting 18 WS frames
  // per step saturated the radio and caused camera stream latency.
  // The servo panel is not needed during walking.
}

void moveForward() {
  if (!isWalking) {
    isWalking = true;
    currentDirection = FORWARD;
    currentStep = 0;
    lastStepTime = millis();
    LOG("Started walking forward");
  }
}

void moveBackward() {
  if (!isWalking) {
    isWalking = true;
    currentDirection = BACKWARD;
    currentStep = 0;
    lastStepTime = millis();
    LOG("Started walking backward");
  }
}

void stopMovement() {
  if (isWalking) {
    isWalking = false;
    LOG("Stopped walking");

    // Reset servos immediately — I2C writes are ~100 µs each, no delay needed.
    for (int i = 0; i < 18; i++) {
      setServoAngle(i, 90);
    }

    // Do NOT call broadcastTXT() / broadcastServoUpdate() here.
    // This function is always invoked from inside webSocketEvent(), which is
    // itself called from webSocket.loop(). Any broadcastTXT() call here re-enters
    // the WebSocket library while it is already processing a frame, saturating
    // the WiFi MAC layer and triggering the SDK sys-context software WDT crash.
    // Set the flag; loop() will broadcast everything safely on the next tick.
    pendingStopBroadcast = true;
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      LOGF("[%u] Client disconnected", num);
      break;

    case WStype_CONNECTED: {
      // On ESP8266, rapid reconnects or stale slots can occasionally leave
      // remoteIP() unsafe; guard by validating client slot first.
      if (webSocket.clientIsConnected(num)) {
        IPAddress ip = webSocket.remoteIP(num);
        LOGF("[%u] Connected from %d.%d.%d.%d", num, ip[0], ip[1], ip[2], ip[3]);
      } else {
        LOGF("[%u] Connected (IP unavailable)", num);
      }

      // Send current servo positions — no delay() here; delay() inside a
      // WebSocket callback calls yield() which can re-enter the library
      // and corrupt its internal state, causing resets.
      for (int i = 0; i < 18; i++) {
        sendServoUpdate(num, i, servoPositions[i], servoPWMValues[i]);
      }

      // Send walk status
      StaticJsonDocument<200> doc;
      doc["type"] = "walkStatus";
      doc["walking"] = isWalking;
      doc["step"] = currentStep;
      String message;
      serializeJson(doc, message);
      webSocket.sendTXT(num, message);

      // Send speed
      StaticJsonDocument<128> speedDoc;
      speedDoc["type"] = "speedUpdate";
      speedDoc["speed"] = stepDelay;
      String speedMessage;
      serializeJson(speedDoc, speedMessage);
      webSocket.sendTXT(num, speedMessage);

      // Send stride
      StaticJsonDocument<128> strideDoc;
      strideDoc["type"] = "strideUpdate";
      strideDoc["stride"] = stride;
      String strideMessage;
      serializeJson(strideDoc, strideMessage);
      webSocket.sendTXT(num, strideMessage);

      break;
    }

    case WStype_TEXT: {
      String message = String((char*)payload);
      LOGF("[%u] Received: %s", num, message.c_str());

      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, message);
      if (error) {
        LOG("JSON parse failed: " + String(error.c_str()));
        return;
      }

      const char* msgType = doc["type"];
      if (msgType == nullptr) {
        LOGF("[%u] JSON missing 'type'", num);
        return;
      }

      if (strcmp(msgType, "setServo") == 0) {
        int servo = doc["servo"];
        int angle = doc["angle"];
        if (servo >= 0 && servo < 18 && angle >= 0 && angle <= 180) {
          setServoAngle(servo, angle);
          broadcastServoUpdate(servo, angle, servoPWMValues[servo]);
        }
      }
      else if (strcmp(msgType, "setCamServo") == 0) {
        const char* axis = doc["axis"];
        int angle = doc["angle"];
        angle = constrain(angle, 0, 180);
        if (strcmp(axis, "pan") == 0) {
          camPanAngle = angle;
          setCamServoAngle(CAM_PAN_CH, angle);
          Serial.println("Camera pan: " + String(angle));   // Serial only — joystick fires rapidly
        } else if (strcmp(axis, "tilt") == 0) {
          camTiltAngle = angle;
          setCamServoAngle(CAM_TILT_CH, angle);
          Serial.println("Camera tilt: " + String(angle));  // Serial only — joystick fires rapidly
        }
      }
      else if (strcmp(msgType, "startWalk") == 0) {
        moveForward();
      }
      else if (strcmp(msgType, "startWalkBackward") == 0) {
        moveBackward();
      }
      else if (strcmp(msgType, "stopWalk") == 0) {
        stopMovement();
      }
      else if (strcmp(msgType, "startTurnRight") == 0) {
        moveTurnLeft();
      }
      else if (strcmp(msgType, "startTurnLeft") == 0) {
        moveTurnRight();
      }
      else if (strcmp(msgType, "waveHand") == 0) {
        pendingGesture = GESTURE_WAVE_HAND;
      }
      else if (strcmp(msgType, "danceMove") == 0) {
        pendingGesture = GESTURE_DANCE_MOVE;
      }
      else if (strcmp(msgType, "waveAll_Left") == 0) {
        pendingGesture = GESTURE_WAVE_ALL_LEFT;
      }
      else if (strcmp(msgType, "waveAll_Right") == 0) {
        pendingGesture = GESTURE_WAVE_ALL_RIGHT;
      }
      else if (strcmp(msgType, "setSpeed") == 0) {
        int newSpeed = doc["speed"];
        if (newSpeed >= 100 && newSpeed <= 1000) {
          stepDelay = newSpeed;
          LOG("Speed updated: " + String(stepDelay) + "ms");

          StaticJsonDocument<128> speedDoc;
          speedDoc["type"] = "speedUpdate";
          speedDoc["speed"] = stepDelay;
          String speedMessage;
          serializeJson(speedDoc, speedMessage);
          webSocket.broadcastTXT(speedMessage);
        }
      }
      else if (strcmp(msgType, "setHeight") == 0) {
        int newHeight = doc["height"];
        if (newHeight >= -30 && newHeight <= 30) {
          heightOffset = newHeight;
          EEPROM.write(EEPROM_HEIGHT_ADDR, (int8_t)heightOffset);
          EEPROM.commit();
          LOG("Height offset: " + String(heightOffset));

          for (int i = 0; i < 18; i++) {
            int jointNum = i % 3;
            if (jointNum == 1 || jointNum == 2) {
              setServoAngle(i, servoPositions[i]);
              broadcastServoUpdate(i, servoPositions[i], servoPWMValues[i]);
            }
          }
        }
      }
      else if (strcmp(msgType, "setStride") == 0) {
        int newStride = doc["stride"];
        if (newStride >= 10 && newStride <= 90) {
          stride = newStride;
          LOG("Stride updated: " + String(stride) + " degrees");

          StaticJsonDocument<128> strideDoc;
          strideDoc["type"] = "strideUpdate";
          strideDoc["stride"] = stride;
          String strideMessage;
          serializeJson(strideDoc, strideMessage);
          webSocket.broadcastTXT(strideMessage);
        }
      }
      // ─── Satellite Comms Commands ────────────────────────────────
      else if (strcmp(msgType, "connectSatellite") == 0) {
        LOG("[SAT] ESP-NOW telemetry activated");
        satEnabled = true;
        pendingSatStatusBroadcast = true;  // deferred — safe from loop() only
      }
      else if (strcmp(msgType, "disconnectSatellite") == 0) {
        LOG("[SAT] ESP-NOW telemetry deactivated");
        satEnabled = false;
        pendingSatStatusBroadcast = true;
      }
      else if (strcmp(msgType, "sendChat") == 0) {
        String chatMsg = doc["msg"].as<String>();
        String chatTo  = doc["to"].as<String>();
        if (chatTo.isEmpty()) chatTo = "satellite";
        if (satEnabled && chatMsg.length() > 0) {
          EspNowChatV2 pkt = {};
          pkt.header.sender_id = DEVICE_ID_HEXAPOD;
          // Enforce hub relay: hexapod sends chat only to satellite.
          pkt.header.receiver_id = DEVICE_ID_SATELLITE;
          pkt.header.packet_type = ESPNOW_PKT_CHAT;
          pkt.header.sequence_no = nextSequence();
          strncpy(pkt.from, "hexapod", sizeof(pkt.from) - 1);
          pkt.from[sizeof(pkt.from) - 1] = '\0';
          strncpy(pkt.to, chatTo.c_str(), sizeof(pkt.to) - 1);
          pkt.to[sizeof(pkt.to) - 1] = '\0';
          strncpy(pkt.msg, chatMsg.c_str(), sizeof(pkt.msg) - 1);
          pkt.msg[sizeof(pkt.msg) - 1] = '\0';
          esp_now_send(satMac, (uint8_t*)&pkt, sizeof(pkt));

          LOG("[SAT] Chat sent to " + chatTo + ": " + chatMsg);
        }
      }
      break;
    }

    case WStype_ERROR:
      LOGF("[%u] WebSocket Error", num);
      break;

    default:
      break;
  }
}

void sendServoUpdate(uint8_t clientNum, int servo, int angle, int pwm) {
  // Static buffer avoids repeated heap alloc/free that causes fragmentation
  // when called 18 times per walk step. Buffer size 96 bytes is sufficient for
  // {"type":"servoUpdate","servo":17,"angle":180,"pwm":431} (54 chars).
  static char buf[96];
  StaticJsonDocument<128> doc;
  doc["type"]  = "servoUpdate";
  doc["servo"] = servo;
  doc["angle"] = angle;
  doc["pwm"]   = pwm;
  serializeJson(doc, buf, sizeof(buf));
  webSocket.sendTXT(clientNum, buf);
}

void broadcastServoUpdate(int servo, int angle, int pwm) {
  static char buf[96];
  StaticJsonDocument<128> doc;
  doc["type"]  = "servoUpdate";
  doc["servo"] = servo;
  doc["angle"] = angle;
  doc["pwm"]   = pwm;
  serializeJson(doc, buf, sizeof(buf));
  webSocket.broadcastTXT(buf);
}

void waveHand() {
  LOG("Starting Wave Hand");
  ESP.wdtFeed(); yield(); webSocket.loop();
  int coxa = 0, femur = 1, tibia = 2;

  setServoAngle(femur, 180);  ESP.wdtFeed(); delay(500);
  setServoAngle(tibia, 0);    ESP.wdtFeed(); delay(500);
  setServoAngle(coxa, 180);   ESP.wdtFeed(); delay(500);
  setServoAngle(coxa, 0);     ESP.wdtFeed(); delay(500);
  setServoAngle(coxa, 90);    ESP.wdtFeed(); delay(500);

  for (int i = 0; i < 2; i++) {
    ESP.wdtFeed(); webSocket.loop();
    setServoAngle(tibia, 0);   ESP.wdtFeed(); delay(300);
    setServoAngle(tibia, 180); ESP.wdtFeed(); delay(300);
  }

  setServoAngle(femur, 90);
  setServoAngle(tibia, 90);
  setServoAngle(coxa, 90);
  LOG("Wave Hand finished");
}

void danceMove() {
  LOG("Starting Dance Move");
  int L1_F = 1, L2_F = 4, L3_F = 7;
  int R4_F = 10, R5_F = 13, R6_F = 16;
  int L1_C = 0, L2_C = 3, L3_C = 6;
  int R4_C = 9, R5_C = 12, R6_C = 15;

  for (int repeat = 0; repeat < 3; repeat++) {
    // Each repeat takes ~2000 ms; explicitly feed both WDTs and service the
    // WebSocket at the top of every repeat so connections stay alive.
    ESP.wdtFeed(); yield(); webSocket.loop();
    setServoAngle(L1_F, 60); setServoAngle(L2_F, 60); setServoAngle(L3_F, 60);
    delay(400);
    setServoAngle(L1_F, 90); setServoAngle(L2_F, 90); setServoAngle(L3_F, 90);

    setServoAngle(R4_F, 120); setServoAngle(R5_F, 120); setServoAngle(R6_F, 120);
    delay(400);
    setServoAngle(R4_F, 90); setServoAngle(R5_F, 90); setServoAngle(R6_F, 90);

    setServoAngle(L1_C, 60); setServoAngle(L2_C, 60); setServoAngle(L3_C, 60);
    setServoAngle(R4_C, 120); setServoAngle(R5_C, 120); setServoAngle(R6_C, 120);
    delay(400);
    setServoAngle(L1_C, 120); setServoAngle(L2_C, 120); setServoAngle(L3_C, 120);
    setServoAngle(R4_C, 60); setServoAngle(R5_C, 60); setServoAngle(R6_C, 60);
    delay(400);

    setServoAngle(L1_C, 90); setServoAngle(L2_C, 90); setServoAngle(L3_C, 90);
    setServoAngle(R4_C, 90); setServoAngle(R5_C, 90); setServoAngle(R6_C, 90);
    delay(400);
  }
  LOG("Dance finished");
}

void waveAll_Left() {
  LOG("Starting Wave All Left");
  ESP.wdtFeed(); yield(); webSocket.loop();
  int coxa[6]  = {0, 3, 6, 9, 12, 15};
  int femur[6] = {1, 4, 7, 10, 13, 16};
  int tibia[6] = {2, 5, 8, 11, 14, 17};

  for (int i = 0; i < 6; i++) setServoAngle(femur[i], 20);
  ESP.wdtFeed(); delay(500);
  for (int i = 0; i < 6; i++) setServoAngle(tibia[i], 150);
  ESP.wdtFeed(); delay(500);
  for (int i = 0; i < 6; i++) setServoAngle(coxa[i], 150);
  ESP.wdtFeed(); delay(500);
  for (int i = 0; i < 6; i++) setServoAngle(coxa[i], 20);
  ESP.wdtFeed(); delay(500);
  for (int i = 0; i < 6; i++) setServoAngle(coxa[i], 90);
  ESP.wdtFeed(); delay(500);

  for (int j = 0; j < 3; j++) {
    ESP.wdtFeed(); webSocket.loop();
    for (int i = 0; i < 6; i++) setServoAngle(tibia[i], 0);
    ESP.wdtFeed(); delay(300);
    for (int i = 0; i < 6; i++) setServoAngle(tibia[i], 150);
    ESP.wdtFeed(); delay(300);
  }

  for (int i = 0; i < 6; i++) {
    setServoAngle(femur[i], 90);
    setServoAngle(tibia[i], 90);
    setServoAngle(coxa[i], 90);
  }
  ESP.wdtFeed(); delay(500);
  LOG("Wave All Left finished");
}

void waveAll_Right() {
  LOG("Starting Wave All Right");
  ESP.wdtFeed(); yield(); webSocket.loop();
  int coxa[6]  = {0, 3, 6, 9, 12, 15};
  int femur[6] = {1, 4, 7, 10, 13, 16};
  int tibia[6] = {2, 5, 8, 11, 14, 17};

  for (int i = 0; i < 6; i++) setServoAngle(femur[i], 150);
  ESP.wdtFeed(); delay(500);
  for (int i = 0; i < 6; i++) setServoAngle(tibia[i], 20);
  ESP.wdtFeed(); delay(500);
  for (int i = 0; i < 6; i++) setServoAngle(coxa[i], 150);
  ESP.wdtFeed(); delay(500);
  for (int i = 0; i < 6; i++) setServoAngle(coxa[i], 50);
  ESP.wdtFeed(); delay(500);
  for (int i = 0; i < 6; i++) setServoAngle(coxa[i], 90);
  ESP.wdtFeed(); delay(500);

  for (int j = 0; j < 3; j++) {
    ESP.wdtFeed(); webSocket.loop();
    for (int i = 0; i < 6; i++) setServoAngle(tibia[i], 0);
    ESP.wdtFeed(); delay(300);
    for (int i = 0; i < 6; i++) setServoAngle(tibia[i], 150);
    ESP.wdtFeed(); delay(300);
  }

  for (int i = 0; i < 6; i++) {
    setServoAngle(femur[i], 90);
    setServoAngle(tibia[i], 90);
    setServoAngle(coxa[i], 90);
  }
  ESP.wdtFeed(); delay(500);
  LOG("Wave All Right finished");
}

// ============================================================
// SATELLITE COMMUNICATION — ESP-NOW
// ============================================================

void broadcastSatStatus() {
  // Inform the dashboard browser about the current ESP-NOW link state.
  StaticJsonDocument<128> doc;
  doc["type"]    = "satStatus";
  doc["enabled"] = satEnabled;
  doc["mode"]    = "ESP-NOW";
  String msg;
  serializeJson(doc, msg);
  webSocket.broadcastTXT(msg);
}

// Must be called after WiFi.softAP() so the radio is up and the channel is set.
void initEspNow() {
  if (esp_now_init() != 0) {
    LOG("[ESP-NOW] Init failed!");
    return;
  }
  // COMBO role = both sender and receiver
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(onEspNowSent);
  esp_now_register_recv_cb(onEspNowRecv);

  // Register satellite as unicast peer on channel 6.
  // satMac must be filled in with the satellite's AP MAC (see top of file).
  int r = esp_now_add_peer(satMac, ESP_NOW_ROLE_COMBO, 6, NULL, 0);
  if (r == 0)
    Serial.printf("[ESP-NOW] Satellite peer registered: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  satMac[0], satMac[1], satMac[2], satMac[3], satMac[4], satMac[5]);
  else
    Serial.println("[ESP-NOW] WARNING: Satellite peer failed — update satMac");
  LOG("[ESP-NOW] Initialized — unicast satellite peer on ch 6");
}

// Called from WiFi task after each esp_now_send().
// status 0 = success, 1 = fail (broadcast always reports success).
void onEspNowSent(uint8_t *mac, uint8_t status) {
  (void)mac; (void)status;  // nothing to do — fire and forget
}

// Called from WiFi task when a packet arrives from the satellite.
// Keep this ISR-safe: only copy data into static buffers and set flags.
// The actual movement / broadcast logic runs from loop().
void onEspNowRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  (void)mac;
  if (len < 1) return;

  if (len >= sizeof(EspNowHeader)) {
    const EspNowHeader* header = (const EspNowHeader*)data;
    const bool isV2Type = header->packet_type == ESPNOW_PKT_COMMAND ||
                          header->packet_type == ESPNOW_PKT_CHAT ||
                          header->packet_type == ESPNOW_PKT_MIRROR_LOG;
    if (isV2Type) {
      if (header->receiver_id != DEVICE_ID_HEXAPOD && header->receiver_id != DEVICE_ID_BROADCAST) return;

      if (header->packet_type == ESPNOW_PKT_COMMAND && len >= sizeof(EspNowCommandV2)) {
        const EspNowCommandV2* pkt = (const EspNowCommandV2*)data;
        strncpy(pendingEspNowCmd, pkt->cmd, sizeof(pendingEspNowCmd) - 1);
        pendingEspNowCmd[sizeof(pendingEspNowCmd) - 1] = '\0';
        hasPendingEspNowCmd = true;
      }
      else if (header->packet_type == ESPNOW_PKT_CHAT && len >= sizeof(EspNowChatV2)) {
        const EspNowChatV2* pkt = (const EspNowChatV2*)data;
        if (header->sender_id == DEVICE_ID_HEXAPOD || strncmp(pkt->from, "hexapod", 7) == 0) return;
        strncpy(pendingEspNowChatFrom, pkt->from, sizeof(pendingEspNowChatFrom) - 1);
        pendingEspNowChatFrom[sizeof(pendingEspNowChatFrom) - 1] = '\0';
        strncpy(pendingEspNowChatMsg, pkt->msg, sizeof(pendingEspNowChatMsg) - 1);
        pendingEspNowChatMsg[sizeof(pendingEspNowChatMsg) - 1] = '\0';
        hasPendingEspNowChat = true;
      }
      return;
    }
  }

  uint8_t msgType = data[0];

  if (msgType == ESPNOW_MSG_COMMAND && len >= sizeof(EspNowCommand)) {
    const EspNowCommand* pkt = (const EspNowCommand*)data;
    strncpy(pendingEspNowCmd, pkt->cmd, sizeof(pendingEspNowCmd) - 1);
    pendingEspNowCmd[sizeof(pendingEspNowCmd) - 1] = '\0';
    hasPendingEspNowCmd = true;
  }
  else if (msgType == ESPNOW_MSG_CHAT && len >= sizeof(EspNowChat)) {
    const EspNowChat* pkt = (const EspNowChat*)data;
    // Echo filter: satellite may broadcast our own relayed message before it
    // learns the rover's unicast MAC. Ignore any packet that claims "hexapod".
    if (strncmp(pkt->from, "hexapod", 7) == 0) return;
    strncpy(pendingEspNowChatFrom, pkt->from, sizeof(pendingEspNowChatFrom) - 1);
    pendingEspNowChatFrom[sizeof(pendingEspNowChatFrom) - 1] = '\0';
    strncpy(pendingEspNowChatMsg, pkt->msg, sizeof(pendingEspNowChatMsg) - 1);
    pendingEspNowChatMsg[sizeof(pendingEspNowChatMsg) - 1] = '\0';
    hasPendingEspNowChat = true;
  }
}

// Pack sensor + walking state into a 26-byte struct and fire it to the satellite.
// No connection state to guard — ESP-NOW is connectionless.
void sendTelemetryEspNow() {
  EspNowTelemetry pkt;
  pkt.msgType    = ESPNOW_MSG_TELEMETRY;
  pkt.deviceType = ESPNOW_DEVICE_HEXAPOD;
  pkt.temp    = g_temp;
  pkt.hum     = g_hum;
  pkt.gas     = (int16_t)g_gas;
  pkt.aqi     = (int16_t)g_aqi;
  pkt.co2     = (int16_t)g_co2;
  pkt.rssi    = (int8_t)constrain(g_rssi, -128, 127);
  pkt.walking = isWalking ? 1 : 0;
  pkt.step    = (uint8_t)currentStep;
  pkt.clients = (uint8_t)webSocket.connectedClients();
  for (int i = 0; i < 6; i++) {
    // Store coxa angle as signed offset from 90° to fit in int8_t
    pkt.legs[i] = (int8_t)constrain(servoPositions[i * 3] - 90, -128, 127);
  }
  esp_now_send(satMac, (uint8_t*)&pkt, sizeof(pkt));
}
