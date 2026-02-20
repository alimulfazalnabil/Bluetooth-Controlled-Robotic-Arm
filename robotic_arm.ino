
/*
  Bluetooth Controlled Robotic Arm (Arduino Mega 2560)
  - Bluetooth: HC-05 via Serial1 (RX1=19, TX1=18) at 9600
  - Command format (line-based):
      J<id>:<angle>\n   e.g. J2:90
      STOP\n           emergency stop (freeze targets)
      HOME\n           moves to home position safely
      STATUS\n         prints current angles
  - Features:
      * Command buffering
      * Rate limiting (anti-burst)
      * Servo smoothing (slew-rate limiting)
      * Joint limits + safe home
*/

#include <Servo.h>

// -------------------- CONFIG --------------------
static const uint32_t BT_BAUD = 9600;

// Number of joints (including gripper if servo-based)
static const uint8_t NUM_JOINTS = 6;

// Servo signal pins (edit per your build)
static const uint8_t SERVO_PINS[NUM_JOINTS] = {2, 3, 4, 5, 6, 7};

// Joint IDs in command are 1..NUM_JOINTS (J1..J6)
static const uint8_t MIN_ANGLE[NUM_JOINTS] = {  0,  10,  10,   0,   0,  20};
static const uint8_t MAX_ANGLE[NUM_JOINTS] = {180, 170, 170, 180, 180, 160};

// Home positions
static const uint8_t HOME_ANGLE[NUM_JOINTS] = {90, 90, 90, 90, 90, 60};

// Slew rate limiting: degrees per update tick
static const float MAX_DEG_PER_TICK = 1.5f;

// Control loop tick (ms)
static const uint16_t CONTROL_PERIOD_MS = 20;

// Rate limit incoming commands (min gap)
static const uint16_t MIN_CMD_GAP_MS = 30;

// Serial input line length
static const uint8_t LINE_MAX = 48;

// -------------------- STATE --------------------
Servo servos[NUM_JOINTS];

float currentAngle[NUM_JOINTS];
float targetAngle[NUM_JOINTS];

bool emergencyStop = false;

char lineBuf[LINE_MAX];
uint8_t lineLen = 0;

uint32_t lastControlMs = 0;
uint32_t lastCmdMs = 0;

// -------------------- UTILS --------------------
static inline uint8_t clampU8(int v, uint8_t lo, uint8_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (uint8_t)v;
}

static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  J<id>:<angle>   e.g. J2:90"));
  Serial.println(F("  STOP"));
  Serial.println(F("  HOME"));
  Serial.println(F("  STATUS"));
}

static void statusPrint(Stream &out) {
  out.print(F("STOP=")); out.print(emergencyStop ? F("1") : F("0"));
  out.print(F(" | "));
  for (uint8_t i=0; i<NUM_JOINTS; i++) {
    out.print(F("J")); out.print(i+1);
    out.print(F("=")); out.print((int)currentAngle[i]);
    if (i < NUM_JOINTS-1) out.print(F(","));
  }
  out.println();
}

// Move targets to home (does not instantly jump; smoothing handles it)
static void setHomeTargets() {
  for (uint8_t i=0; i<NUM_JOINTS; i++) {
    targetAngle[i] = HOME_ANGLE[i];
  }
}

static bool parseJointCommand(const char* s, uint8_t &jointIdx, uint8_t &angle) {
  // Expect "J<id>:<angle>"
  // Examples: J1:90, J6:120
  if (s[0] != 'J' && s[0] != 'j') return false;

  // parse id
  int id = 0;
  int i = 1;
  while (s[i] >= '0' && s[i] <= '9') {
    id = id * 10 + (s[i] - '0');
    i++;
  }
  if (id < 1 || id > (int)NUM_JOINTS) return false;
  if (s[i] != ':') return false;
  i++;

  // parse angle
  int ang = 0;
  bool hasDigits = false;
  while (s[i] >= '0' && s[i] <= '9') {
    hasDigits = true;
    ang = ang * 10 + (s[i] - '0');
    i++;
  }
  if (!hasDigits) return false;

  jointIdx = (uint8_t)(id - 1);
  // clamp within joint limits
  angle = clampU8(ang, MIN_ANGLE[jointIdx], MAX_ANGLE[jointIdx]);
  return true;
}

// Handles a full line command
static void handleCommandLine(char* cmdLine, Stream &out) {
  // Trim CR/LF and spaces
  // Minimal trim: remove trailing \r
  for (int i=(int)strlen(cmdLine)-1; i>=0; i--) {
    if (cmdLine[i] == '\r' || cmdLine[i] == '\n' || cmdLine[i] == ' ') cmdLine[i] = '\0';
    else break;
  }

  if (cmdLine[0] == '\0') return;

  // Rate limiting to protect against bursty BT packets
  uint32_t now = millis();
  if (now - lastCmdMs < MIN_CMD_GAP_MS) {
    // Ignore to reduce jitter from command storms
    return;
  }
  lastCmdMs = now;

  // Uppercase compare for simple tokens
  if (!strcasecmp(cmdLine, "STOP")) {
    emergencyStop = true;
    // Freeze targets at current to avoid sudden moves after release
    for (uint8_t i=0; i<NUM_JOINTS; i++) targetAngle[i] = currentAngle[i];
    out.println(F("OK STOP"));
    return;
  }

  if (!strcasecmp(cmdLine, "HOME")) {
    emergencyStop = false;
    setHomeTargets();
    out.println(F("OK HOME"));
    return;
  }

  if (!strcasecmp(cmdLine, "STATUS")) {
    statusPrint(out);
    return;
  }

  if (!strcasecmp(cmdLine, "HELP")) {
    printHelp();
    return;
  }

  // Joint angle command
  uint8_t jIdx = 0, ang = 0;
  if (parseJointCommand(cmdLine, jIdx, ang)) {
    if (!emergencyStop) {
      targetAngle[jIdx] = ang;
      out.print(F("OK J")); out.print(jIdx+1);
      out.print(F("=")); out.println(ang);
    } else {
      out.println(F("IGNORED (STOP)"));
    }
    return;
  }

  out.println(F("ERR BAD_CMD"));
}

// Read bytes from BT, accumulate line buffer until '\n'
static void readBluetooth() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();

    // Basic line discipline
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      handleCommandLine(lineBuf, Serial1);   // respond on BT
      handleCommandLine(lineBuf, Serial);    // mirror to USB serial for debugging
      lineLen = 0;
      continue;
    }

    // Ignore oversized lines
    if (lineLen < LINE_MAX - 1) {
      lineBuf[lineLen++] = c;
    } else {
      // Reset to avoid buffer overflow
      lineLen = 0;
    }
  }
}

// Control loop: move currentAngle toward targetAngle smoothly
static void controlTick() {
  for (uint8_t i=0; i<NUM_JOINTS; i++) {
    float cur = currentAngle[i];
    float tgt = targetAngle[i];

    float diff = tgt - cur;
    if (diff > MAX_DEG_PER_TICK) diff = MAX_DEG_PER_TICK;
    if (diff < -MAX_DEG_PER_TICK) diff = -MAX_DEG_PER_TICK;

    float next = cur + diff;

    // Safety clamp again
    if (next < MIN_ANGLE[i]) next = MIN_ANGLE[i];
    if (next > MAX_ANGLE[i]) next = MAX_ANGLE[i];

    currentAngle[i] = next;
    servos[i].write((int)(next + 0.5f));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Robotic Arm starting..."));
  printHelp();

  // Bluetooth serial
  Serial1.begin(BT_BAUD);

  // Attach servos and initialize positions
  for (uint8_t i=0; i<NUM_JOINTS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    currentAngle[i] = HOME_ANGLE[i];
    targetAngle[i]  = HOME_ANGLE[i];
    servos[i].write(HOME_ANGLE[i]);
  }

  delay(300);
  Serial.println(F("READY"));
  statusPrint(Serial);
  statusPrint(Serial1);
}

void loop() {
  readBluetooth();

  uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    if (!emergencyStop) {
      controlTick();
    } else {
      // In STOP: keep servos at current (no drift)
      for (uint8_t i=0; i<NUM_JOINTS; i++) {
        servos[i].write((int)(currentAngle[i] + 0.5f));
      }
    }
  }
}
