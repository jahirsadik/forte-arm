// =============================================================
// RobStride EduLite 05 (EL05) - three motors on one CAN bus
// Board: Dash_Ethernet_Teensy_Ethernet_Board, connector CANH1_1/CANL1_1 (J2)
//   J2 -> MCP2562FD (U4) -> Teensy U1 pin 22 (CTX1) / pin 23 (CRX1)  => CAN1
//
// Serial Monitor @115200. Type a command and press Enter:
//   s            scan bus for motor IDs 1..127
//   i OLD NEW    change a motor's CAN ID, e.g. "i 127 12"
//                (only ONE motor on the bus may have ID OLD)
//   e            enable all motors (free, Kp=0)
//   f            free all (Kp=0)
//   h            every motor that is responding holds its current position
//   m            mirror: each follower follows its leader (see LEADER[] below)
//   d            disable all
//   r            toggle raw frame printing
//   x            CAN error diagnostics
// =============================================================
#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;   // CAN1 = pins 22/23 (J2)

const uint8_t HOST_ID = 0xFD;                     // 253

// ---- Motor setup: edit these lines to change IDs / roles ----
const int N = 3;
const uint8_t MOTOR_IDS[N] = {1, 11, 12};
const uint8_t LEADER[N]    = {0,  1,  1};   // ID each motor follows in mirror mode (0 = it's a leader)
const float   DIR[N]       = {1,  1,  1};   // set a follower to -1 if it turns the wrong way
// Example: third motor as a second leader instead -> MOTOR_IDS {1, 11, 2}, LEADER {0, 1, 0}

// EL05 operation-control ranges. VERIFY against the EL05 manual.
const float P_MIN = -12.57f, P_MAX = 12.57f;
const float V_MIN = -50.0f,  V_MAX = 50.0f;
const float T_MIN = -6.0f,   T_MAX = 6.0f;
const float KP_MAX = 500.0f, KD_MAX = 5.0f;

// Gains (kept gentle for bench testing)
const float FREE_KD   = 0.05f;
const float HOLD_KP   = 5.0f,  HOLD_KD   = 0.2f;
const float FOLLOW_KP = 5.0f,  FOLLOW_KD = 0.2f;
const uint32_t STALE_MS = 100;   // feedback older than this is considered lost

// ---------- state ----------
struct Motor {
  float pos = 0, vel = 0, tor = 0, temp = 0;
  uint8_t fault = 0, mode = 0;
  uint32_t count = 0, last_ms = 0;
  bool valid = false;
  float hold = 0;
  float offset = 0;   // mirror offset
};
Motor mot[N];

enum RunMode { OFF, FREE, HOLD, MIRROR };
RunMode runMode = OFF;
bool rawPrint = false;

// ---------- helpers ----------
uint16_t f2u(float x, float lo, float hi) {
  x = constrain(x, lo, hi);
  return (uint16_t)((x - lo) / (hi - lo) * 65535.0f);
}
float u2f(uint16_t x, float lo, float hi) { return lo + (float)x * (hi - lo) / 65535.0f; }

int indexOf(uint8_t id) {
  for (int i = 0; i < N; i++) if (MOTOR_IDS[i] == id) return i;
  return -1;
}

bool fresh(int i) { return i >= 0 && mot[i].valid && (millis() - mot[i].last_ms) < STALE_MS; }

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
    mot[i].pos   = u2f((m.buf[0] << 8) | m.buf[1], P_MIN, P_MAX);
    mot[i].vel   = u2f((m.buf[2] << 8) | m.buf[3], V_MIN, V_MAX);
    mot[i].tor   = u2f((m.buf[4] << 8) | m.buf[5], T_MIN, T_MAX);
    mot[i].temp  = ((m.buf[6] << 8) | m.buf[7]) / 10.0f;
    mot[i].fault = (m.id >> 16) & 0x3F;
    mot[i].mode  = (m.id >> 22) & 0x03;
    mot[i].valid = true;
    mot[i].last_ms = millis();
    mot[i].count++;
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

// ---------- commands ----------
void pumpCan(uint32_t ms) { uint32_t t0 = millis(); while (millis() - t0 < ms) can.events(); }

void scanIds() {
  Serial.println("Scanning IDs 1..127 ...");
  for (int id = 1; id <= 127; id++) { pingId(id); pumpCan(2); }
  pumpCan(100);
  Serial.println("Scan done.");
  printDiag();
}

void enableAll() {
  for (int i = 0; i < N; i++) {
    setRunModeOperation(MOTOR_IDS[i]); pumpCan(20);
    enableMotor(MOTOR_IDS[i]);         pumpCan(20);
  }
}

void disableAll() {
  for (int i = 0; i < N; i++) { disableMotor(MOTOR_IDS[i]); pumpCan(20); }
}

bool allFresh() {
  bool ok = true;
  for (int i = 0; i < N; i++) if (!fresh(i)) {
    Serial.printf("No recent feedback from motor %u - is it connected, enabled, and set to this ID?\r\n",
                  MOTOR_IDS[i]);
    ok = false;
  }
  return ok;
}

void printHelp() { Serial.println("Commands: s | i OLD NEW | e | f | h | m | d | r | x"); }

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  char c = line[0];

  switch (c) {
    case 's': scanIds(); break;

    case 'i': {
      int oldId, newId;
      if (sscanf(line.c_str() + 1, "%d %d", &oldId, &newId) == 2 &&
          oldId >= 1 && oldId <= 127 && newId >= 1 && newId <= 127 && oldId != newId) {
        if (runMode != OFF) { disableAll(); runMode = OFF; }
        Serial.printf("Changing ID %d -> %d (only ONE motor may currently have ID %d!)\r\n", oldId, newId, oldId);
        changeId(oldId, newId);
        pumpCan(100);
        Serial.println("Done. Power-cycle the motors, then type 's' to verify the new ID.");
      } else {
        Serial.println("Usage: i OLD NEW   e.g. i 127 12");
      }
      break;
    }

    case 'e':
      enableAll();
      runMode = FREE;
      Serial.print("Enabled motors:");
      for (int i = 0; i < N; i++) Serial.printf(" %u", MOTOR_IDS[i]);
      Serial.println(" (free)");
      break;

    case 'f':
      if (runMode == OFF) { Serial.println("Enable first (e)."); break; }
      runMode = FREE;
      Serial.println("Free (Kp=0)");
      break;

    case 'h':
      if (runMode == OFF) { Serial.println("Enable first (e)."); break; }
      for (int i = 0; i < N; i++) {
        if (fresh(i)) {
          mot[i].hold = mot[i].pos;
          Serial.printf("Motor %u holding at %.3f rad\r\n", MOTOR_IDS[i], mot[i].hold);
        } else {
          Serial.printf("Motor %u: no feedback, left free\r\n", MOTOR_IDS[i]);
        }
      }
      runMode = HOLD;
      break;

    case 'm': {
      if (runMode == OFF) { Serial.println("Enable first (e)."); break; }
      if (!allFresh()) break;   // mirror needs every motor responding
      bool okCfg = true;
      for (int i = 0; i < N; i++) {
        if (LEADER[i] != 0 && indexOf(LEADER[i]) < 0) {
          Serial.printf("Leader %u of motor %u is not in MOTOR_IDS!\r\n", LEADER[i], MOTOR_IDS[i]);
          okCfg = false;
        }
      }
      if (!okCfg) break;
      for (int i = 0; i < N; i++) {
        if (LEADER[i] == 0) continue;
        int li = indexOf(LEADER[i]);
        mot[i].offset = mot[i].pos - DIR[i] * mot[li].pos;   // no jump at start
        mot[i].hold   = mot[i].pos;
        Serial.printf("Mirror: motor %u follows motor %u (offset %.3f rad)\r\n",
                      MOTOR_IDS[i], LEADER[i], mot[i].offset);
      }
      runMode = MIRROR;
      Serial.println("Mirror ON. Turn the leader(s) by hand.");
      break;
    }

    case 'd':
      disableAll();
      runMode = OFF;
      Serial.println("Disabled all");
      break;

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
  Serial.setTimeout(50);   // works with any Serial Monitor line-ending setting
  while (!Serial && millis() < 3000);
  pinMode(LED_BUILTIN, OUTPUT);

  can.begin();
  can.setBaudRate(1000000);
  can.setMaxMB(16);
  can.enableFIFO();
  can.enableFIFOInterrupt();
  can.onReceive(onRx);

  Serial.println("=== EL05 three-motor test on CAN1 (J2) ===");
  for (int i = 0; i < N; i++) {
    if (LEADER[i] == 0) Serial.printf("Motor %u: leader\r\n", MOTOR_IDS[i]);
    else                Serial.printf("Motor %u: follows %u\r\n", MOTOR_IDS[i], LEADER[i]);
  }
  printHelp();
}

void loop() {
  can.events();

  if (Serial.available()) handleLine(Serial.readStringUntil('\n'));

  // 100 Hz control
  static elapsedMillis ctl;
  if (runMode != OFF && ctl >= 10) {
    ctl = 0;
    for (int i = 0; i < N; i++) {
      uint8_t id = MOTOR_IDS[i];
      switch (runMode) {
        case FREE:
          opControl(id, 0, 0, 0, 0, FREE_KD);
          break;

        case HOLD:
          if (mot[i].valid) opControl(id, 0, mot[i].hold, 0, HOLD_KP, HOLD_KD);
          else              opControl(id, 0, 0, 0, 0, FREE_KD);   // never heard from it: stay free
          break;

        case MIRROR:
          if (LEADER[i] == 0) {
            opControl(id, 0, 0, 0, 0, FREE_KD);                  // leader: free, you move it
          } else {
            int li = indexOf(LEADER[i]);
            if (fresh(li)) mot[i].hold = DIR[i] * mot[li].pos + mot[i].offset;
            // if leader feedback is lost, follower keeps its last target
            opControl(id, 0, mot[i].hold, 0, FOLLOW_KP, FOLLOW_KD);
          }
          break;

        default: break;
      }
    }
  }

  // status print 4 Hz, only lines with new data
  static elapsedMillis pr;
  static uint32_t lastCount[N] = {0};
  if (pr >= 250) {
    pr = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    bool any = false;
    for (int i = 0; i < N; i++) {
      if (mot[i].valid && mot[i].count != lastCount[i]) {
        lastCount[i] = mot[i].count;
        any = true;
        Serial.printf("[M%-3u] pos %7.3f rad | vel %6.2f | tor %5.2f Nm | %4.1f C | mode %u | fault 0x%02X\r\n",
                      MOTOR_IDS[i], mot[i].pos, mot[i].vel, mot[i].tor, mot[i].temp,
                      mot[i].mode, mot[i].fault);
      }
    }
    if (any) Serial.println();   // blank line between snapshots
  }
}
