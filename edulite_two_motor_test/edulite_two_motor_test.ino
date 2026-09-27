// =============================================================
// RobStride EduLite 05 (EL05) - two motors on one CAN bus
// Board: Dash_Ethernet_Teensy_Ethernet_Board, connector CANH1_1/CANL1_1 (J2)
//   J2 -> MCP2562FD (U4) -> Teensy U1 pin 22 (CTX1) / pin 23 (CRX1)  => CAN1
//
// Serial Monitor @115200. Type a command and press Enter:
//   s            scan bus for motor IDs 1..127
//   i OLD NEW    change a motor's CAN ID (ONLY that motor connected!), e.g. "i 127 1"
//   e            enable both motors (free, Kp=0)
//   f            free both (Kp=0)
//   h            both hold their current position
//   m            mirror: motor MOTOR_IDS[1] follows MOTOR_IDS[0] (leader stays free)
//   d            disable both
//   r            toggle raw frame printing
//   x            CAN error diagnostics
// =============================================================
#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;   // CAN1 = pins 22/23 (J2)

const uint8_t HOST_ID = 0xFD;                     // 253
const int N = 2;
const uint8_t MOTOR_IDS[N] = {1, 11};             // [0] = leader, [1] = follower

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
};
Motor mot[N];

enum RunMode { OFF, FREE, HOLD, MIRROR };
RunMode runMode = OFF;
float mirror_offset = 0;
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
  for (int i = 0; i < N; i++) if (!fresh(i)) {
    Serial.printf("No recent feedback from motor %u - is it connected, enabled, and set to this ID?\r\n",
                  MOTOR_IDS[i]);
    return false;
  }
  return true;
}

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
        Serial.printf("Changing ID %d -> %d (make sure ONLY this motor is connected!)\r\n", oldId, newId);
        changeId(oldId, newId);
        pumpCan(100);
        Serial.println("Done. Power-cycle the motor, then type 's' to verify the new ID.");
      } else {
        Serial.println("Usage: i OLD NEW   e.g. i 127 1");
      }
      break;
    }

    case 'e':
      enableAll();
      runMode = FREE;
      Serial.printf("Enabled motors %u and %u (free)\r\n", MOTOR_IDS[0], MOTOR_IDS[1]);
      break;

    case 'f':
      if (runMode == OFF) { Serial.println("Enable first (e)."); break; }
      runMode = FREE;
      Serial.println("Free (Kp=0)");
      break;

    case 'h':
      if (runMode == OFF) { Serial.println("Enable first (e)."); break; }
      if (!allFresh()) break;
      for (int i = 0; i < N; i++) mot[i].hold = mot[i].pos;
      runMode = HOLD;
      Serial.printf("Holding: motor %u at %.3f, motor %u at %.3f rad\r\n",
                    MOTOR_IDS[0], mot[0].hold, MOTOR_IDS[1], mot[1].hold);
      break;

    case 'm':
      if (runMode == OFF) { Serial.println("Enable first (e)."); break; }
      if (!allFresh()) break;
      mirror_offset = mot[1].pos - mot[0].pos;   // no jump at start
      mot[1].hold   = mot[1].pos;
      runMode = MIRROR;
      Serial.printf("Mirror ON: motor %u follows motor %u (offset %.3f rad). Turn motor %u by hand.\r\n",
                    MOTOR_IDS[1], MOTOR_IDS[0], mirror_offset, MOTOR_IDS[0]);
      break;

    case 'd':
      disableAll();
      runMode = OFF;
      Serial.println("Disabled both");
      break;

    case 'r':
      rawPrint = !rawPrint;
      Serial.printf("Raw print %s\r\n", rawPrint ? "ON" : "OFF");
      break;

    case 'x': printDiag(); break;

    default: Serial.println("Commands: s | i OLD NEW | e | f | h | m | d | r | x");
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

  Serial.println("=== EL05 two-motor test on CAN1 (J2) ===");
  Serial.printf("Motor IDs: leader %u, follower %u\r\n", MOTOR_IDS[0], MOTOR_IDS[1]);
  Serial.println("Commands: s | i OLD NEW | e | f | h | m | d | r | x");
}

void loop() {
  can.events();

  if (Serial.available()) handleLine(Serial.readStringUntil('\n'));

  // 100 Hz control
  static elapsedMillis ctl;
  if (runMode != OFF && ctl >= 10) {
    ctl = 0;
    switch (runMode) {
      case FREE:
        for (int i = 0; i < N; i++) opControl(MOTOR_IDS[i], 0, 0, 0, 0, FREE_KD);
        break;

      case HOLD:
        for (int i = 0; i < N; i++) opControl(MOTOR_IDS[i], 0, mot[i].hold, 0, HOLD_KP, HOLD_KD);
        break;

      case MIRROR: {
        opControl(MOTOR_IDS[0], 0, 0, 0, 0, FREE_KD);              // leader: free, you move it
        if (fresh(0)) mot[1].hold = mot[0].pos + mirror_offset;    // follower target
        // if leader feedback is lost, follower just keeps its last target
        opControl(MOTOR_IDS[1], 0, mot[1].hold, 0, FOLLOW_KP, FOLLOW_KD);
        break;
      }
      default: break;
    }
  }

  // status print 4 Hz, only lines with new data
  static elapsedMillis pr;
  static uint32_t lastCount[N] = {0};
  if (pr >= 250) {
    pr = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    for (int i = 0; i < N; i++) {
      if (mot[i].valid && mot[i].count != lastCount[i]) {
        lastCount[i] = mot[i].count;
        Serial.printf("[M%-3u] pos %7.3f rad | vel %6.2f | tor %5.2f Nm | %4.1f C | mode %u | fault 0x%02X\r\n",
                      MOTOR_IDS[i], mot[i].pos, mot[i].vel, mot[i].tor, mot[i].temp,
                      mot[i].mode, mot[i].fault);
      }
    }
  }
}
