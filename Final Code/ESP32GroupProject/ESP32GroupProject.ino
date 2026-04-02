/*
  ESP32-S3 Lynxmotion Robotic Arm Controller (Teach + Playback) - Logan Smith [22020998]

  Quick overview:
  - ESP32-S3 connects to a phone/PC UI over BLE.
  - UI sends short text commands (e.g. "ARM", "G1:90", "REC1", "PF").
  - Commands get converted into servo PWM pulse widths (microseconds).
  - Servo motion is smoothed (slew/rate-limited) to avoid snapping/jolting the arm and spiking current.
  - “Teach” mode records what you command over time, then you can play it back forward or in reverse.
  - Recording + a few settings are saved in NVS so they survive a reset.

  Status LED (NeoPixel on IO38):
  - DISARMED  -> solid RED
  - ARMED     -> solid AMBER
  - MOVING    -> AMBER fast blink

  Safety bits:
  - Optional DISARM when BLE disconnects.
  - Heartbeat watchdog: if the UI stops sending "HB", the arm disarms.
  - ABORT command stops record/play straight away.

  Main commands (sent to BLE RX characteristic):
    ARM / DISARM / CA / STATUS / HELP / ABORT
    HB                            (heartbeat ping from UI)
    G<idx>:<deg>                  (set joint angle with per-command limiter)
    S<idx><U/D/L/R/C>             (jog joint)
    REC1 / REC0 / RCI<ms>         (record start/stop/set interval)
    PF / PR                       (play forward / play reverse)
    EXPORT / POSES / CLR / SAVE   (utility/debug; REC is the main workflow)
    DW<ms>                        (legacy dwell setting; still stored)
*/

#include <BLEDevice.h>         // BLE device init / advertising
#include <BLEUtils.h>          // BLE helper types
#include <BLEServer.h>         // GATT server support
#include <BLE2902.h>           // Descriptor needed for notifications (most clients expect this)

#include <ESP32Servo.h>        // Servo control on ESP32 (PWM attach/detach)
#include <math.h>              // fabsf, roundf
#include <Preferences.h>       // NVS storage (keeps data after reset)
#include <Adafruit_NeoPixel.h> // NeoPixel status LED

// --------------------------- BLE UUIDs ----------------------------
// Custom service + two characteristics:
// - RX: UI writes commands here
// - TX: ESP32 notifies status strings back to the UI
#define SERVICE_UUID           "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID_RX "12345678-1234-1234-1234-123456789abd" // write / writeNR
#define CHARACTERISTIC_UUID_TX "12345678-1234-1234-1234-123456789abe" // notify

// ------------------------ RGB Status LED --------------------------
// Board has a single RGB LED on IO38. Used as a simple state indicator.
static constexpr int STATUS_NEOPIXEL_PIN   = 38;  // data pin for NeoPixel
static constexpr int STATUS_NEOPIXEL_COUNT = 1;   // one LED
static Adafruit_NeoPixel g_strip(
  STATUS_NEOPIXEL_COUNT,
  STATUS_NEOPIXEL_PIN,
  NEO_GRB + NEO_KHZ800                             // standard NeoPixel timing/format
);

// LED behaviour modes. In this project we mainly use SOLID + FAST blink.
enum LedMode { LED_OFF=0, LED_SOLID, LED_BLINK_SLOW, LED_BLINK_FAST };
static LedMode  g_ledMode         = LED_SOLID;     // current mode
static bool     g_ledState        = false;         // toggles during blink
static uint32_t g_ledLastToggleMs = 0;             // last time we flipped g_ledState

// --------------------------- Servos -------------------------------
// 6 servos for this Lynxmotion arm (including gripper).
static const int NUM_SERVOS = 6;

// Pin mapping (make sure this matches your wiring harness).
static const int SERVO_PINS[NUM_SERVOS] = {
  18, // 0 Base
  17, // 1 Shoulder
  16, // 2 Elbow
   6, // 3 Second Elbow
  15, // 4 Wrist Rotation
   7  // 5 Gripper
};

// Pulse limits per joint (µs). Adjust if a joint binds or hits a hard stop.
static int SERVO_MIN_US[NUM_SERVOS] = {1000,1000,1000,1000,1000, 500};
static int SERVO_MAX_US[NUM_SERVOS] = {2000,2000,2000,2000,2000,2500};

// Invert joints if the mechanical installation is flipped.
static bool SERVO_INV[NUM_SERVOS]   = {false, true, true, true, false, false};

// Neutral “safe-ish” pose (degrees). Used on ARM.
int NEUTRAL_DEG[NUM_SERVOS] = { 90, 60, 120, 90, 90, 90 };

// Manual limiter: max degrees a single manual command is allowed to jump.
// (Playback/record can still do bigger moves; smoothing handles the motion.)
static const int MAX_DEG_STEP_PER_CMD_JOINT[NUM_SERVOS] = {
  12, // 0 Base
  10, // 1 Shoulder
  10, // 2 Elbow
  10, // 3 Second Elbow
  12, // 4 Wrist Rot
  90  // 5 Gripper
};

// Slew rate for the smoothing loop (µs/s). Lower = gentler motion.
float speedUsPerSec = 300.0f;

// Jog step in degrees for S<idx><U/D/...>.
int jogStepDeg = 3;

// Small pause between servo writes to avoid all channels updating at once.
const int STAGGER_WRITE_US = 300;

// --------------------- Runtime motion state -----------------------
// Servo objects (one per joint).
Servo servos[NUM_SERVOS];

// Smoothed “current” output (float so we can step smoothly).
float curUs[NUM_SERVOS]    = {1500,1500,1500,1500,1500,1500};

// Target values that commands/playback set (what we're trying to reach).
int   targetUs[NUM_SERVOS] = {1500,1500,1500,1500,1500,1500};

// Last PWM value actually written (used to avoid spamming the same number).
int   lastOutUs[NUM_SERVOS]= {-1,-1,-1,-1,-1,-1};

volatile bool isConnected = false;          // set in BLE callbacks
bool armed = false;                         // only true when servos are attached + outputs allowed
bool autoDisarmOnDisconnect = true;         // safety: drop outputs if BLE link dies

BLECharacteristic* txChar = nullptr;        // TX notify characteristic (for messages back to UI)

// Heartbeat watchdog: UI should keep sending "HB" while controlling the arm.
static uint32_t g_lastHeartbeatMs = 0;
static const uint32_t HEARTBEAT_TIMEOUT_MS = 2000; // ms without HB -> DISARM

// Movement detection (used for LED fast blink).
static bool g_isMoving = false;
static const float MOVING_EPS_US = 8.0f;    // "close enough" threshold

// ------------------------- Timeline frames ------------------------
// One Pose = one recorded frame (pulse widths for all joints).
struct Pose { uint16_t u[NUM_SERVOS]; };

// Max frames stored in RAM/NVS.
static const int MAX_POSES = 256;

Pose poses[MAX_POSES];                      // recorded timeline frames
int  poseCount = 0;                         // how many frames are valid

// Stored for older pose tools / compatibility. Timeline playback uses rec interval.
uint16_t dwellMs = 500;

// -------------------- NVS (Preferences) storage -------------------
// Saved so recordings persist across resets.
Preferences prefs;
static const char* NVS_NS         = "arm";
static const char* NVS_KEY_POSES  = "poses_v4"; // bump if Pose layout changes
static const char* NVS_KEY_COUNT  = "count";
static const char* NVS_KEY_DWELL  = "dwell";
static const char* NVS_KEY_RECINT = "recInt";

// ------------------------ Recording state -------------------------
// Recording = sample targetUs[] every g_recIntervalMs.
static bool     g_recActive     = false;   // recording flag
static uint16_t g_recIntervalMs = 150;     // sampling interval (ms)
static uint32_t g_recLastMs     = 0;       // last sample timestamp
static bool     g_recDirty      = false;   // only save to flash once at the end

// ------------------------ Playback state --------------------------
// Playback = step through poses[] and apply to targets.
static bool     g_playActive       = false;
static bool     g_playReverse      = false;
static int      g_playIndex        = 0;
static uint32_t g_playPhaseStartMs = 0;
static bool     g_playWaiting      = false;

// --------------------------- Helpers ------------------------------
// Small clamp helper so we don't repeat bounds checks everywhere.
static inline int clampi(int v, int lo, int hi){ return v < lo ? lo : (v > hi ? hi : v); }

// Midpoint pulse width for joint i (centering).
static inline int midUs(int i){ return (SERVO_MIN_US[i] + SERVO_MAX_US[i]) / 2; }

// Degrees to microseconds for a specific joint's calibrated range.
static inline int degToUs(int i, int deg){
  deg = clampi(deg, 0, 180);
  return map(deg, 0, 180, SERVO_MIN_US[i], SERVO_MAX_US[i]);
}

// Microseconds to degrees (mainly for limiting and export/debug).
static inline int usToDeg(int i, int us){
  us = clampi(us, SERVO_MIN_US[i], SERVO_MAX_US[i]);
  return map(us, SERVO_MIN_US[i], SERVO_MAX_US[i], 0, 180);
}

// Send a short status message back to the BLE client (if notifications are enabled).
static inline void notifyMsg(const char* m){
  if (txChar){
    txChar->setValue(m);
    txChar->notify();
  }
}

// Write PWM to a servo, with clamp and optional inversion.
// Also avoids writing the same value repeatedly.
void writeServoUs(uint8_t i, int us){
  if (!armed || i >= NUM_SERVOS) return;

  us = clampi(us, SERVO_MIN_US[i], SERVO_MAX_US[i]);
  if (SERVO_INV[i]) us = SERVO_MIN_US[i] + SERVO_MAX_US[i] - us;

  if (lastOutUs[i] != us){
    servos[i].writeMicroseconds(us);
    lastOutUs[i] = us;
  }
}

// Set the target PWM for a joint (clamped).
void setTargetUs(uint8_t i, int us){
  if (i < NUM_SERVOS) targetUs[i] = clampi(us, SERVO_MIN_US[i], SERVO_MAX_US[i]);
}

// Manual set with per-command limiter (stops a single command from snapping a joint).
void setTargetDegLimited(uint8_t i, int deg){
  if (i >= NUM_SERVOS) return;
  deg = clampi(deg, 0, 180);

  int curDeg = usToDeg(i, targetUs[i]);
  int lim = MAX_DEG_STEP_PER_CMD_JOINT[i];
  int delta = clampi(deg - curDeg, -lim, lim);
  int newDeg = clampi(curDeg + delta, 0, 180);

  setTargetUs(i, degToUs(i, newDeg));
}

// Full set (no limiter): used for record/play/center where smoothing is doing the safety work.
void setTargetDegFull(uint8_t i, int deg){
  if (i >= NUM_SERVOS) return;
  setTargetUs(i, degToUs(i, deg));
}

// Push the neutral pose into targets (the smoothing loop will actually move the arm).
void applyNeutralPose(){
  for (int i=0; i<NUM_SERVOS; i++){
    setTargetDegFull((uint8_t)i, NEUTRAL_DEG[i]);
  }
}

// Jog helper for S commands.
void jogDeg(uint8_t i, char action){
  if (i >= NUM_SERVOS) return;

  int deg = usToDeg(i, targetUs[i]);
  switch(action){
    case 'U': deg += jogStepDeg; break;
    case 'D': deg -= jogStepDeg; break;
    case 'L': deg = 0; break;
    case 'R': deg = 180; break;
    case 'C': deg = 90; break;
  }
  setTargetDegLimited(i, deg);
}

// Check if all joints are basically at their targets (used to pace playback).
static bool allReached(float epsUs){
  for (int i=0; i<NUM_SERVOS; i++){
    if (fabsf((float)targetUs[i] - curUs[i]) > epsUs) return false;
  }
  return true;
}

// -------------------------- NVS save/load -------------------------
// Save poseCount and settings and pose buffer to flash.
static void saveToNVS(){
  prefs.begin(NVS_NS, false);

  prefs.putUChar(NVS_KEY_COUNT, (uint8_t)poseCount);
  prefs.putUShort(NVS_KEY_DWELL, dwellMs);
  prefs.putUShort(NVS_KEY_RECINT, g_recIntervalMs);

  // We save the whole buffer for simplicity.
  prefs.putBytes(NVS_KEY_POSES, poses, sizeof(poses));

  prefs.end();

  notifyMsg("POSES SAVED");
}

// If nothing valid is stored, start with a simple centred frame.
static void initDefaultTimeline(){
  poseCount = 0;

  Pose p{};
  for (int i=0; i<NUM_SERVOS; i++){
    p.u[i] = (uint16_t)midUs(i);
  }
  poses[poseCount++] = p;
}

// Try to load from NVS; if it doesn’t match expected size/version, fall back to defaults.
static void loadFromNVS(){
  prefs.begin(NVS_NS, true);

  uint8_t cnt = prefs.getUChar(NVS_KEY_COUNT, 0);
  dwellMs = prefs.getUShort(NVS_KEY_DWELL, 500);
  g_recIntervalMs = prefs.getUShort(NVS_KEY_RECINT, 150);

  size_t got = prefs.getBytesLength(NVS_KEY_POSES);
  size_t want = sizeof(poses);

  if (got == want && cnt <= MAX_POSES){
    prefs.getBytes(NVS_KEY_POSES, poses, want);
    poseCount = (int)cnt;
    prefs.end();
    notifyMsg("POSES LOADED");
    return;
  }
  prefs.end();

  // Nothing valid saved yet (or version mismatch).
  initDefaultTimeline();
  saveToNVS();
  notifyMsg("POSES INIT");
}

// --------------------------- Utility ops --------------------------
// Clear current recording and reset to a single centred frame.
static void posesClearAndCenter(){
  initDefaultTimeline();
  saveToNVS();
  notifyMsg("CLEARED");
}

// Legacy “snap” (adds one frame immediately and saves).
static void poseSnap(){
  if (!armed){ notifyMsg("NOT ARMED"); return; }
  if (poseCount >= MAX_POSES){ notifyMsg("POSES FULL"); return; }

  Pose p{};
  for (int i=0; i<NUM_SERVOS; i++){
    p.u[i] = (uint16_t)clampi(targetUs[i], 0, 65535);
  }
  poses[poseCount++] = p;

  saveToNVS();
  notifyMsg("SNAP");
}

// Dump frames back to the UI as degrees (helpful when tuning limits/inversion).
static void exportPoses(){
  char line[96];
  snprintf(line, sizeof(line), "POSES %d", poseCount);
  notifyMsg(line);

  for (int k=0; k<poseCount; k++){
    int d[NUM_SERVOS];
    for (int i=0; i<NUM_SERVOS; i++){
      d[i] = clampi(usToDeg(i, poses[k].u[i]), 0, 180);
    }
    snprintf(line, sizeof(line), "POSE %d: %d,%d,%d,%d,%d,%d",
             k, d[0], d[1], d[2], d[3], d[4], d[5]);
    notifyMsg(line);
  }
}

// --------------------------- ARM/DISARM ---------------------------
// Attach servos and enable output.
static void armAll(){
  if (armed){ notifyMsg("ARMED"); return; }

  // ESP32Servo uses LEDC timers; allocate them up front.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach each servo channel.
  for (int i=0; i<NUM_SERVOS; i++){
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2500);
    lastOutUs[i] = -1;
  }

  armed = true;
  g_lastHeartbeatMs = millis();

  applyNeutralPose();
  notifyMsg("ARMED");
}

// Detach servos and disable output.
static void disarmAll(){
  for (int i=0; i<NUM_SERVOS; i++){
    if (servos[i].attached()) servos[i].detach();
    lastOutUs[i] = -1;
  }
  armed = false;
  notifyMsg("DISARMED");
}

// --------------------------- Smooth motion ------------------------
uint32_t lastUpdateMs = 0;

static void smoothUpdate(){
  // Rate-limited chase from curUs[] -> targetUs[] (simple smoothing).
  uint32_t now = millis();
  uint32_t dtMs = now - lastUpdateMs;
  if (dtMs < 20) return; // ~50Hz update
  lastUpdateMs = now;

  float step = speedUsPerSec * (dtMs / 1000.0f);
  bool anyMoving = false;

  for (int i=0; i<NUM_SERVOS; i++){
    float diff = (float)targetUs[i] - curUs[i];

    // Snap when we're close to avoid tiny oscillations.
    if (fabsf(diff) <= 10.0f) curUs[i] = (float)targetUs[i];
    else curUs[i] += (fabsf(diff) <= step) ? diff : (diff > 0 ? step : -step);

    int out = clampi((int)roundf(curUs[i]), SERVO_MIN_US[i], SERVO_MAX_US[i]);
    writeServoUs((uint8_t)i, out);

    if (armed && STAGGER_WRITE_US > 0) delayMicroseconds(STAGGER_WRITE_US);

    if (fabsf((float)targetUs[i] - curUs[i]) > MOVING_EPS_US) anyMoving = true;
  }

  g_isMoving = armed && anyMoving;
}

// --------------------------- LED control --------------------------
// Quick RGB flash on boot so you can tell the LED is alive.
static void ledBootTest(){
  g_strip.begin();
  g_strip.setBrightness(64);
  g_strip.setPixelColor(0, g_strip.Color(255,0,0)); g_strip.show(); delay(120);
  g_strip.setPixelColor(0, g_strip.Color(0,255,0)); g_strip.show(); delay(120);
  g_strip.setPixelColor(0, g_strip.Color(0,0,255)); g_strip.show(); delay(120);
  g_strip.clear(); g_strip.show();
}

// Update LED based on armed and moving state.
static void updateStatusLED(){
  LedMode desired = LED_SOLID;
  if (armed && g_isMoving) desired = LED_BLINK_FAST;

  if (desired != g_ledMode){
    g_ledMode = desired;
    g_ledLastToggleMs = millis();
    g_ledState = false;
  }

  const uint32_t AMBER = g_strip.Color(255,170,0);
  const uint32_t RED   = g_strip.Color(255,0,0);
  const uint32_t OFF   = g_strip.Color(0,0,0);
  const uint32_t ONC   = armed ? AMBER : RED;

  uint32_t now = millis();

  switch (g_ledMode){
    case LED_SOLID:
      g_strip.fill(ONC);
      g_strip.show();
      break;

    case LED_BLINK_FAST:
      if (now - g_ledLastToggleMs >= 150){
        g_ledState = !g_ledState;
        g_strip.fill(g_ledState ? ONC : OFF);
        g_strip.show();
        g_ledLastToggleMs = now;
      }
      break;

    case LED_BLINK_SLOW:
      if (now - g_ledLastToggleMs >= 800){
        g_ledState = !g_ledState;
        g_strip.fill(g_ledState ? ONC : OFF);
        g_strip.show();
        g_ledLastToggleMs = now;
      }
      break;

    case LED_OFF:
    default:
      g_strip.fill(OFF);
      g_strip.show();
      break;
  }
}

// -------------------------- Recording -----------------------------
// Teach mode: sample current targetUs[] every g_recIntervalMs.
static void recStart(){
  if (!armed){ notifyMsg("NOT ARMED"); return; }
  if (g_playActive){ notifyMsg("BUSY PLAY"); return; }

  // New recording overwrites the old one.
  poseCount = 0;
  g_recActive = true;
  g_recDirty = true;
  g_recLastMs = millis();

  // Grab an initial frame straight away.
  Pose p{};
  for (int i=0; i<NUM_SERVOS; i++){
    p.u[i] = (uint16_t)clampi(targetUs[i], 0, 65535);
  }
  poses[poseCount++] = p;

  notifyMsg("REC START");
}

static void recStop(){
  if (!g_recActive){ notifyMsg("REC IDLE"); return; }
  g_recActive = false;

  // Save once at the end (flash wear is a thing).
  if (g_recDirty){
    saveToNVS();
    g_recDirty = false;
  }

  char b[32];
  snprintf(b, sizeof(b), "REC STOP %d", poseCount);
  notifyMsg(b);
}

static void recTick(){
  if (!g_recActive) return;
  if (!armed){ recStop(); return; }

  uint32_t now = millis();
  if ((uint32_t)(now - g_recLastMs) < g_recIntervalMs) return;
  g_recLastMs = now;

  if (poseCount >= MAX_POSES){
    notifyMsg("REC FULL");
    recStop();
    return;
  }

  Pose p{};
  for (int i=0; i<NUM_SERVOS; i++){
    p.u[i] = (uint16_t)clampi(targetUs[i], 0, 65535);
  }
  poses[poseCount++] = p;
  g_recDirty = true;
}

// -------------------------- Playback ------------------------------
// Playback: apply frames into targetUs[] and let smoothing handle the motion between them.
static void playStart(bool reverse){
  if (!armed){ notifyMsg("NOT ARMED"); return; }
  if (g_recActive){ notifyMsg("BUSY REC"); return; }
  if (poseCount <= 1){ notifyMsg("POSES 0"); return; }

  g_playActive = true;
  g_playReverse = reverse;
  g_playWaiting = false;
  g_playPhaseStartMs = millis();

  g_playIndex = reverse ? (poseCount - 1) : 0;

  // Apply the first frame.
  for (int i=0; i<NUM_SERVOS; i++){
    setTargetUs((uint8_t)i, poses[g_playIndex].u[i]);
  }

  notifyMsg(reverse ? "PLAY REV START" : "PLAY FWD START");
}

static void playStop(const char* msg){
  g_playActive = false;
  g_playWaiting = false;
  notifyMsg(msg);
}

static void playTick(){
  if (!g_playActive) return;

  if (!armed){
    playStop("PLAY ABORT (DISARM)");
    return;
  }

  // Wait until we actually reach the current frame before stepping on.
  if (!g_playWaiting){
    if (allReached(8.0f)){
      g_playWaiting = true;
      g_playPhaseStartMs = millis();
    }
    return;
  }

  // Use the record interval as the playback timestep.
  if (millis() - g_playPhaseStartMs < g_recIntervalMs) return;

  if (!g_playReverse){
    g_playIndex++;
    if (g_playIndex >= poseCount){
      playStop("PLAY DONE");
      return;
    }
  } else {
    g_playIndex--;
    if (g_playIndex < 0){
      playStop("PLAY DONE");
      return;
    }
  }

  // Apply next frame.
  for (int i=0; i<NUM_SERVOS; i++){
    setTargetUs((uint8_t)i, poses[g_playIndex].u[i]);
  }

  g_playWaiting = false;
  g_playPhaseStartMs = millis();
}

// -------------------------- ABORT ---------------------------------
// Hard stop for both record and playback.
static void abortAll(){
  if (g_playActive) playStop("PLAY ABORT");
  if (g_recActive)  recStop();
  notifyMsg("ABORT");
}

// =========================== Commands =============================
// Parse one command string from BLE and execute it.
// During playback we block manual motion commands so playback isn’t fighting the user.
void handleCommand(String cmd){
  cmd.trim();
  if (!cmd.length()) return;

  if (cmd == "HB"){
    g_lastHeartbeatMs = millis();
    return;
  }

  if (cmd == "ARM"){ armAll(); return; }
  if (cmd == "DISARM"){ disarmAll(); return; }

  if (cmd == "ABORT"){ abortAll(); return; }

  // ----- Record control -----
  if (cmd == "REC1"){ recStart(); return; }
  if (cmd == "REC0"){ recStop();  return; }
  if (cmd.startsWith("RCI")){
    int ms = cmd.substring(3).toInt();
    ms = clampi(ms, 50, 1000);
    g_recIntervalMs = (uint16_t)ms;
    saveToNVS();                 // keep it across resets
    notifyMsg("RCI OK");
    return;
  }

  // ----- Playback control -----
  if (cmd == "PF"){ playStart(false); return; }
  if (cmd == "PR"){ playStart(true);  return; }

  // While playing, ignore manual commands that would mess with playback.
  if (g_playActive){
    if (cmd == "CA" || cmd == "SAVE" || cmd == "CLR" || cmd == "EXPORT" ||
        cmd.startsWith("G") || cmd.startsWith("S") || cmd.startsWith("VU") || cmd.startsWith("JSD")){
      notifyMsg("BUSY PLAY");
      return;
    }
  }

  // Center all joints.
  if (cmd == "CA"){
    if (!armed){ notifyMsg("NOT ARMED"); return; }
    for (int i=0; i<NUM_SERVOS; i++) setTargetUs((uint8_t)i, midUs(i));
    notifyMsg("CENTER ALL");
    return;
  }

  // Joint Set: G<idx>:<deg>
  if (cmd.charAt(0) == 'G' && cmd.indexOf(':') >= 0){
    if (!armed){ notifyMsg("NOT ARMED"); return; }
    int idx = cmd.charAt(1) - '0';
    int c = cmd.indexOf(':');
    int deg = cmd.substring(c + 1).toInt();
    if (idx >= 0 && idx < NUM_SERVOS) setTargetDegLimited((uint8_t)idx, deg);
    return;
  }

  // Jog: S<digit><U/D/L/R/C>
  if (cmd.length() == 3 && cmd.charAt(0) == 'S'){
    if (!armed){ notifyMsg("NOT ARMED"); return; }
    char c1 = cmd.charAt(1);
    char a  = cmd.charAt(2);
    if (c1 >= '0' && c1 <= '9' && (a=='U'||a=='D'||a=='L'||a=='R'||a=='C')){
      int idx = c1 - '0';
      if (idx >= 0 && idx < NUM_SERVOS) jogDeg((uint8_t)idx, a);
    }
    return;
  }

  // Speed setting: VU<value>
  if (cmd.startsWith("VU")){
    float sp = cmd.substring(2).toFloat();
    if (sp < 50) sp = 50;
    if (sp > 2000) sp = 2000;
    speedUsPerSec = sp;
    notifyMsg("SPEEDUS");
    return;
  }

  // Jog step setting: JSD<deg>
  if (cmd.startsWith("JSD")){
    int st = cmd.substring(3).toInt();
    jogStepDeg = clampi(st, 1, 20);
    notifyMsg("JSTEPDEG");
    return;
  }

  // Utilities
  if (cmd == "SAVE" || cmd == "SNAP"){ poseSnap(); return; }
  if (cmd == "CLR"){ posesClearAndCenter(); return; }
  if (cmd == "POSES"){
    char b[32];
    snprintf(b, sizeof(b), "POSES %d", poseCount);
    notifyMsg(b);
    return;
  }
  if (cmd == "EXPORT"){ exportPoses(); return; }
  if (cmd.startsWith("DW")){
    int ms = cmd.substring(2).toInt();
    dwellMs = (uint16_t)clampi(ms, 0, 5000);
    saveToNVS();
    notifyMsg("DWELL");
    return;
  }

  // Debug status readout
  if (cmd == "STATUS"){
    char b[220];
    snprintf(
      b, sizeof(b),
      "ARM:%d CONN:%d SPD:%.0f HB:%lu REC:%d RCI:%u PLAY:%d REV:%d IDX:%d POSES:%d",
      armed?1:0, isConnected?1:0, speedUsPerSec,
      (unsigned long)(millis() - g_lastHeartbeatMs),
      g_recActive?1:0, (unsigned)g_recIntervalMs,
      g_playActive?1:0, g_playReverse?1:0, g_playIndex, poseCount
    );
    notifyMsg(b);
    return;
  }

  // Command list for the UI / quick reference.
  if (cmd == "HELP"){
    notifyMsg("CMDS: ARM,DISARM,STATUS,HELP,HB,CA,G,S,VU,JSD,SAVE,CLR,POSES,EXPORT,DW,REC1,REC0,RCI,PF,PR,ABORT");
    return;
  }

  notifyMsg("UNKNOWN");
}

// =========================== BLE glue =============================
// RX callback: BLE client writes commands to RX characteristic -> we handle them here.
class RXCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    auto v = ch->getValue();
    if (v.length() == 0) return;
    handleCommand(String(v.c_str()));
  }
};

// Connection callbacks: track link state + do safety disarm on disconnect.
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    isConnected = true;
    g_lastHeartbeatMs = millis();
    notifyMsg("CONNECTED");
  }

  void onDisconnect(BLEServer* pServer) override {
    isConnected = false;
    notifyMsg("DISCONNECTED");

    if (autoDisarmOnDisconnect) disarmAll();

    // Keep advertising so the UI can reconnect.
    pServer->getAdvertising()->start();
  }
};

// ============================ Arduino =============================
void setup(){
  Serial.begin(115200);                      // serial console (debug)

  ledBootTest();                             // quick LED check

  loadFromNVS();                             // restore timeline + settings

  lastUpdateMs = millis();                   // smoothing timer baseline
  g_lastHeartbeatMs = millis();              // watchdog baseline

  BLEDevice::init("ESP32S3-SERVO");          // device name shown in BLE scan list

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  // RX = commands from UI (write / write without response)
  BLECharacteristic* rx = service->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  rx->setCallbacks(new RXCallback());

  // TX = status messages back to UI (notify)
  txChar = service->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  txChar->addDescriptor(new BLE2902());

  service->start();
  server->getAdvertising()->start();

  notifyMsg("READY");
}

void loop(){
  // Watchdog: if the UI stops sending HB while armed, disarm for safety.
  if (isConnected && armed){
    uint32_t age = millis() - g_lastHeartbeatMs;
    if (age > HEARTBEAT_TIMEOUT_MS){
      notifyMsg("HB TIMEOUT");
      disarmAll();
    }
  }

  // Teach/play state machines.
  recTick();
  playTick();

  // Update outputs + LED.
  smoothUpdate();
  updateStatusLED();

  delay(5); // tiny sleep to keep things responsive and avoid maxing the CPU
}

