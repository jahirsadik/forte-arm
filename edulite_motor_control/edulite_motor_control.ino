// =============================================================
// RobStride EduLite 05 (EL05) - independent control of each motor
// Board: Dash_Ethernet_Teensy_Ethernet_Board, connector CANH1_1/CANL1_1 (J2)
//   J2 -> MCP2562FD (U4) -> Teensy U1 pin 22 (CTX1) / pin 23 (CRX1)  => CAN1
//
// Serial Monitor @115200. Type a command and press Enter.
// [ID] means optional: leave it out to apply to all motors.
//
//   e [ID]            enable (free, Kp=0)
//   d [ID]            disable
//   f [ID]            free (turn by hand, position still reported)
//   h [ID]            hold current position
//   t ID DEG [SPEED]  rotate motor ID by DEG degrees (relative, negative = other way)
//                     SPEED in rad/s, optional (default 2.0, max 8.0)
//                     e.g.  t 12 720      t 1 -90      t 11 360 4
//   s                 scan bus for IDs 1..127
//   i OLD NEW         change a motor's CAN ID (only ONE motor may have ID OLD)
//   r                 toggle raw frame printing
//   x                 CAN error diagnostics
//
// Several commands on one line, separated by commas, run at the same time:
//   t 11 120, t 12 720, t 1 -360
//   e, t 12 720          (enable all, then start the move)
//
// How "t" works: the motor is driven with a speed command (Kp=0), the Teensy
// counts how far it has actually turned from the feedback, slows down near the
// end, then holds at the target. This works for any angle, even several turns.
// =============================================================
#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;   // CAN1 = pins 22/23 (J2)

const uint8_t HOST_ID = 0xFD;                     // 253

// ---- Motors on the bus ----
const int N = 3;
const uint8_t MOTOR_IDS[N] = {1, 11, 12};

// EL05 operation-control ranges. VERIFY against the EL05 manual.
const float P_MIN = -12.57f, P_MAX = 12.57f;
const float V_MIN = -50.0f,  V_MAX = 50.0f;
const float T_MIN = -6.0f,   T_MAX = 6.0f;
const float KP_MAX = 500.0f, KD_MAX = 5.0f;

// Gains / motion settings (gentle, for bench testing)
const float FREE_KD      = 0.05f;
const float HOLD_KP      = 5.0f,  HOLD_KD = 0.2f;
const float MOVE_KD      = 0.5f;    // how hard the motor pushes to reach the commanded speed
const float DEFAULT_SPEED = 2.0f;   // rad/s  (720 deg = 12.57 rad -> about 6-7 s)
const float MAX_SPEED    = 8.0f;    // rad/s  hard cap for bench safety
const float MIN_SPEED    = 0.3f;    // rad/s  crawl speed at the very end
const float SLOWDOWN     = 2.0f;    // speed = SLOWDOWN * remaining, near the end
const float DONE_TOL     = 0.02f;   // rad (~1 deg): close enough, switch to hold
const uint32_t STALE_MS  = 100;     // feedback older than this is considered lost

const float PI_F = 3.14159265f;

// ---------- state ----------
enum MState { M_OFF, M_FREE, M_HOLD, M_MOVE };
const char *stateName[] = {"off", "free", "hold", "move"};

struct Motor {
  // feedback
  float pos = 0, vel = 0, tor = 0, temp = 0;
  uint8_t fault = 0, mode = 0;
  uint32_t count = 0, last_ms = 0;
  bool valid = false;
  // control
  MState st = M_OFF;
  float hold = 0;
  // move tracking
  float lastPos = 0;       // previous feedback, for counting travel
  float traveled = 0;      // rad turned since the move started (signed)
  float goal = 0;          // rad to turn (signed)
  float speed = DEFAULT_SPEED;
  uint32_t moveStart = 0, moveTimeout = 0;
};
Motor mot[N];
bool rawPrint = false;

// ---------- helpers ----------
uint16_t f2u(float x, float lo, float hi) {
  x = constrain(x, lo, hi);
  return (uint16_t)((x - lo) / (hi - lo) * 65535.0f);
}
float u2f(uint16_t x, float lo, float hi) { return lo + (float)x * (hi - lo) / 65535.0f; }

int indexOf(int id) {
  for (int i = 0; i < N; i++) if (MOTOR_IDS[i] == id) return i;
  return -1;
}
bool fresh(int i) { return mot[i].valid && (millis() - mot[i].last_ms) < STALE_MS; }

void sendExt(uint32_t id, const uint8_t *data) {
  CAN_message_t m;
  m.flags.extended = 1;
  m.id  = id;
  m.len = 8;
  for (int i = 0; i < 8; i++) m.buf[i] = data ? data[i] : 0;
  can.write(m);
}

// ---------- RobStride commands ----------
void pingId(uint8_t id)        { sendExt((0u << 24) | (HOST_ID << 8) | id, nullptr); }   // type 0
void enableMotor(uint8_t id)   { sendExt((3u << 24) | (HOST_ID << 8) | id, nullptr); }   // type 3
void disableMotor(uint8_t id)  { sendExt((4u << 24) | (HOST_ID << 8) | id, nullptr); }   // type 4
void changeId(uint8_t oldId, uint8_t newId) {                                              // type 7
  sendExt((7u << 24) | ((uint32_t)newId << 16) | (HOST_ID << 8) | oldId, nullptr);
}
void setRunModeOperation(uint8_t id) {   // type 18: write 0x7005 (run_mode) = 0
  uint8_t d[8] = {0x05, 0x70, 0, 0, 0, 0, 0, 0};
  sendExt((18u << 24) | (HOST_ID << 8) | id, d);
}
void opControl(uint8_t id, float t_ff, float pos, float vel, float kp, float kd) {       // type 1
  uint16_t p = f2u(pos, P_MIN, P_MAX), v = f2u(vel, V_MIN, V_MAX);
  uint16_t k = f2u(kp, 0, KP_MAX),     c = f2u(kd, 0, KD_MAX);
  uint16_t t = f2u(t_ff, T_MIN, T_MAX);
  uint8_t d[8] = {(uint8_t)(p >> 8), (uint8_t)p, (uint8_t)(v >> 8), (uint8_t)v,
                  (uint8_t)(k >> 8), (uint8_t)k, (uint8_t)(c >> 8), (uint8_t)c};
  sendExt((1u << 24) | ((uint32_t)t << 8) | id, d);
}

// ---------- receive ----------
void onRx(const CAN_message_t &m) {
  if (rawPrint) {
    Serial.printf("RX %s id=0x%08lX data:", m.flags.extended ? "EXT" : "STD", (unsigned long)m.id);
    for (int i = 0; i < m.len; i++) Serial.printf(" %02X", m.buf[i]);
    Serial.println();
  }
  if (!m.flags.extended) return;
  uint8_t type = (m.id >> 24) & 0x1F;
  uint8_t src  = (m.id >> 8) & 0xFF;

  if (type == 0 && (m.id & 0xFF) == 0xFE) {
    Serial.printf(">>> Found motor with CAN ID %u\r\n", src);
  } else if (type == 2) {
    int i = indexOf(src);
    if (i < 0) return;
    Motor &M = mot[i];
    M.pos   = u2f((m.buf[0] << 8) | m.buf[1], P_MIN, P_MAX);
    M.vel   = u2f((m.buf[2] << 8) | m.buf[3], V_MIN, V_MAX);
    M.tor   = u2f((m.buf[4] << 8) | m.buf[5], T_MIN, T_MAX);
    M.temp  = ((m.buf[6] << 8) | m.buf[7]) / 10.0f;
    M.fault = (m.id >> 16) & 0x3F;
    M.mode  = (m.id >> 22) & 0x03;

    // Count travel during a move. If the reported angle jumps by more than half
    // the range, it wrapped around (e.g. +12.57 -> -12.57), so correct for that.
    if (M.st == M_MOVE && M.valid) {
      float d = M.pos - M.lastPos;
      const float range = P_MAX - P_MIN;
      if (d >  range / 2) d -= range;
      if (d < -range / 2) d += range;
      M.traveled += d;
    }
    M.lastPos = M.pos;
    M.valid   = true;
    M.last_ms = millis();
    M.count++;
  }
}

// ---------- diagnostics (FlexCAN1 registers) ----------
#define CANX_ECR  (*(volatile uint32_t *)(0x401D0000 + 0x1C))
#define CANX_ESR1 (*(volatile uint32_t *)(0x401D0000 + 0x20))
void printDiag() {
  uint32_t esr = CANX_ESR1, ecr = CANX_ECR;
  uint8_t flt = (esr >> 4) & 0x3;
  Serial.printf("  CAN1 diag: TX errors=%lu RX errors=%lu state=%s\r\n",
                (unsigned long)(ecr & 0xFF), (unsigned long)((ecr >> 8) & 0xFF),
                flt == 0 ? "error-active (ok)" : flt == 1 ? "error-passive" : "BUS-OFF");
}

// ---------- actions ----------
void pumpCan(uint32_t ms) { uint32_t t0 = millis(); while (millis() - t0 < ms) can.events(); }

void doEnable(int i) {
  setRunModeOperation(MOTOR_IDS[i]); pumpCan(20);
  enableMotor(MOTOR_IDS[i]);         pumpCan(20);
  mot[i].st = M_FREE;
  Serial.printf("Motor %u enabled (free)\r\n", MOTOR_IDS[i]);
}
void doDisable(int i) {
  disableMotor(MOTOR_IDS[i]); pumpCan(20);
  mot[i].st = M_OFF;
  Serial.printf("Motor %u disabled\r\n", MOTOR_IDS[i]);
}
void doFree(int i) {
  if (mot[i].st == M_OFF) { Serial.printf("Motor %u: enable first (e)\r\n", MOTOR_IDS[i]); return; }
  mot[i].st = M_FREE;
  Serial.printf("Motor %u free\r\n", MOTOR_IDS[i]);
}
void doHold(int i) {
  if (mot[i].st == M_OFF) { Serial.printf("Motor %u: enable first (e)\r\n", MOTOR_IDS[i]); return; }
  if (!fresh(i))          { Serial.printf("Motor %u: no feedback, can't hold\r\n", MOTOR_IDS[i]); return; }
  mot[i].hold = mot[i].pos;
  mot[i].st = M_HOLD;
  Serial.printf("Motor %u holding at %.3f rad\r\n", MOTOR_IDS[i], mot[i].hold);
}

void doTurn(int i, float deg, float speed) {
  Motor &M = mot[i];
  if (M.st == M_OFF) { Serial.printf("Motor %u: enable first (e)\r\n", MOTOR_IDS[i]); return; }
  if (!fresh(i))     { Serial.printf("Motor %u: no feedback, can't move\r\n", MOTOR_IDS[i]); return; }
  speed = constrain(fabsf(speed), MIN_SPEED, MAX_SPEED);
  M.goal     = deg * PI_F / 180.0f;
  M.traveled = 0;
  M.lastPos  = M.pos;
  M.speed    = speed;
  M.moveStart = millis();
  // safety timeout: twice the expected time, plus 3 s
  M.moveTimeout = (uint32_t)(2000.0f * fabsf(M.goal) / speed) + 3000;
  M.st = M_MOVE;
  Serial.printf("Motor %u: turning %.1f deg at up to %.1f rad/s ...\r\n", MOTOR_IDS[i], deg, speed);
}

// Called at 100 Hz: send each motor its command for this cycle
void controlStep() {
  for (int i = 0; i < N; i++) {
    Motor &M = mot[i];
    uint8_t id = MOTOR_IDS[i];
    switch (M.st) {
      case M_OFF: break;

      case M_FREE:
        opControl(id, 0, 0, 0, 0, FREE_KD);
        break;

      case M_HOLD:
        opControl(id, 0, M.hold, 0, HOLD_KP, HOLD_KD);
        break;

      case M_MOVE: {
        float remaining = M.goal - M.traveled;

        if (!fresh(i)) {                                   // lost feedback: stop pushing
          M.st = M_FREE;
          opControl(id, 0, 0, 0, 0, FREE_KD);
          Serial.printf("Motor %u: feedback lost during move -> free\r\n", id);
          break;
        }
        if (millis() - M.moveStart > M.moveTimeout) {      // took far too long: give up, hold
          M.hold = M.pos; M.st = M_HOLD;
          opControl(id, 0, M.hold, 0, HOLD_KP, HOLD_KD);
          Serial.printf("Motor %u: move timed out after %.1f of %.1f deg -> holding\r\n",
                        id, M.traveled * 180.0f / PI_F, M.goal * 180.0f / PI_F);
          break;
        }
        if (fabsf(remaining) < DONE_TOL) {                 // arrived: hold exactly at the target
          M.hold = constrain(M.pos + remaining, P_MIN, P_MAX);
          M.st = M_HOLD;
          opControl(id, 0, M.hold, 0, HOLD_KP, HOLD_KD);
          Serial.printf("Motor %u: done, turned %.1f deg -> holding\r\n",
                        id, M.traveled * 180.0f / PI_F);
          break;
        }
        // speed command: full speed, slowing down over the last part of the move
        float v = constrain(SLOWDOWN * fabsf(remaining), MIN_SPEED, M.speed);
        if (remaining < 0) v = -v;
        opControl(id, 0, 0, v, 0, MOVE_KD);
        break;
      }
    }
  }
}

void scanIds() {
  Serial.println("Scanning IDs 1..127 ...");
  for (int id = 1; id <= 127; id++) { pingId(id); pumpCan(2); }
  pumpCan(100);
  Serial.println("Scan done.");
  printDiag();
}

void printHelp() {
  Serial.println("Commands: e [ID] | d [ID] | f [ID] | h [ID] | t ID DEG [SPEED] | s | i OLD NEW | r | x");
  Serial.println("Combine with commas to run together, e.g.  t 11 120, t 12 720, t 1 -360");
}

// Run fn on one motor (if an ID was given) or on all motors
void forTarget(const char *args, void (*fn)(int)) {
  int id;
  if (sscanf(args, "%d", &id) == 1) {
    int i = indexOf(id);
    if (i < 0) Serial.printf("ID %d is not in MOTOR_IDS\r\n", id);
    else fn(i);
  } else {
    for (int i = 0; i < N; i++) fn(i);
  }
}

// A line may hold several commands separated by commas or semicolons, e.g.
//   t 11 120, t 12 720, t 1 -360
// They are all started in the same instant, so the motors move together.
void handleCommand(String line);

void handleLine(String line) {
  int start = 0;
  for (int k = 0; k <= (int)line.length(); k++) {
    if (k == (int)line.length() || line[k] == ',' || line[k] == ';') {
      handleCommand(line.substring(start, k));
      start = k + 1;
    }
  }
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  char c = line[0];
  const char *args = line.c_str() + 1;

  switch (c) {
    case 'e': forTarget(args, doEnable);  break;
    case 'd': forTarget(args, doDisable); break;
    case 'f': forTarget(args, doFree);    break;
    case 'h': forTarget(args, doHold);    break;

    case 't': {
      int id; float deg, speed = DEFAULT_SPEED;
      int n = sscanf(args, "%d %f %f", &id, &deg, &speed);
      if (n < 2) { Serial.println("Usage: t ID DEG [SPEED]   e.g. t 12 720"); break; }
      int i = indexOf(id);
      if (i < 0) { Serial.printf("ID %d is not in MOTOR_IDS\r\n", id); break; }
      doTurn(i, deg, speed);
      break;
    }

    case 's': scanIds(); break;

    case 'i': {
      int oldId, newId;
      if (sscanf(args, "%d %d", &oldId, &newId) == 2 &&
          oldId >= 1 && oldId <= 127 && newId >= 1 && newId <= 127 && oldId != newId) {
        for (int i = 0; i < N; i++) if (mot[i].st != M_OFF) doDisable(i);
        Serial.printf("Changing ID %d -> %d (only ONE motor may currently have ID %d!)\r\n", oldId, newId, oldId);
        changeId(oldId, newId);
        pumpCan(100);
        Serial.println("Done. Power-cycle the motors, then type 's' to verify.");
      } else {
        Serial.println("Usage: i OLD NEW   e.g. i 127 12");
      }
      break;
    }

    case 'r':
      rawPrint = !rawPrint;
      Serial.printf("Raw print %s\r\n", rawPrint ? "ON" : "OFF");
      break;

    case 'x': printDiag(); break;

    default: printHelp();
  }
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  while (!Serial && millis() < 3000);
  pinMode(LED_BUILTIN, OUTPUT);

  can.begin();
  can.setBaudRate(1000000);
  can.setMaxMB(16);
  can.enableFIFO();
  can.enableFIFOInterrupt();
  can.onReceive(onRx);

  Serial.println("=== EL05 independent motor control on CAN1 (J2) ===");
  Serial.print("Motors:");
  for (int i = 0; i < N; i++) Serial.printf(" %u", MOTOR_IDS[i]);
  Serial.println();
  printHelp();
}

void loop() {
  can.events();

  if (Serial.available()) handleLine(Serial.readStringUntil('\n'));

  static elapsedMillis ctl;           // 100 Hz control
  if (ctl >= 10) { ctl = 0; controlStep(); }

  // status print 2 Hz, only motors with new data
  static elapsedMillis pr;
  static uint32_t lastCount[N] = {0};
  if (pr >= 500) {
    pr = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    bool any = false;
    for (int i = 0; i < N; i++) {
      Motor &M = mot[i];
      if (M.valid && M.count != lastCount[i]) {
        lastCount[i] = M.count;
        any = true;
        Serial.printf("[M%-3u] %-4s | pos %7.3f rad | vel %6.2f | tor %5.2f Nm | %4.1f C | fault 0x%02X",
                      MOTOR_IDS[i], stateName[M.st], M.pos, M.vel, M.tor, M.temp, M.fault);
        if (M.st == M_MOVE)
          Serial.printf(" | turned %.0f / %.0f deg", M.traveled * 180.0f / PI_F, M.goal * 180.0f / PI_F);
        Serial.println();
      }
    }
    if (any) Serial.println();
  }
}
