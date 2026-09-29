
#include <Servo.h>
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <string.h>

// ============================================================
// DEBUG ke USB Serial (115200)
// ============================================================
#define DEBUG 1
#if DEBUG
  #define DBG(x)   Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

// ============================================================
// PIN
// ============================================================
// Motor KIRI = L298N kanal B
#define MK_EN   9    // ENB
#define MK_INF  8    // IN4 -> HIGH = maju
#define MK_INR  7    // IN3 -> HIGH = mundur
// Motor KANAN = L298N kanal A
#define MR_EN   6    // ENA
#define MR_INF  4    // IN1 -> HIGH = maju
#define MR_INR  5    // IN2 -> HIGH = mundur

#define ENC_L_A  2   // INT0
#define ENC_L_B  3   // INT1
#define ENC_R_A 10   // PB2 (sampling Timer2)
#define ENC_R_B 11   // PB3 (sampling Timer2)

// Balik tanda hitungan jika maju terbaca negatif
#define ENC_L_INVERT 0
#define ENC_R_INVERT 0

#define SERVO_PIN  12
#define CUTTER_PIN A5

#define ESP_TX A0
#define ESP_RX A1

// ============================================================
// KONSTANTA
// ============================================================
#define USB_BAUD       115200
#define ESP_BAUD        19200
#define TEL_MIN_MS        200   // telemetri lebih panjang -> jarak min dinaikkan
#define TEL_IDLE_MS       500
#define WATCHDOG_MS      1000
#define RX_BUF_LEN         80
#define RX_STALE_MS        50

#define TEST_MS          2000
#define SWEEP_STEP_MS      15

#define SERVO_CENTER  75
#define SERVO_MIN     45
#define SERVO_MAX    110

// ============================================================
// OBJEK & STATE
// ============================================================
Servo servoSteering;
SoftwareSerial espSerial(ESP_RX, ESP_TX);

int8_t  motorKiriDir  = 0;
int8_t  motorKananDir = 0;
uint8_t motorSpeed    = 0;     // perintah terakhir (hanya ON/OFF)
uint8_t servoAngle    = SERVO_CENTER;
bool    cutterOn      = false;
bool    wdLatched     = false;
uint8_t badFrames     = 0;

unsigned long lastTelMs    = 0;
unsigned long lastCmdMs    = 0;
unsigned long lastRxByteMs = 0;
bool          telRequested = false;

uint16_t      rxCount      = 0;
bool          testActive   = false;
unsigned long testUntil    = 0;
bool          diagPending  = false;
char          diagAct      = '-';
int8_t        sweepDir     = 0;
uint8_t       sweepPos     = SERVO_CENTER;
unsigned long sweepT       = 0;

char    rxBuf[RX_BUF_LEN];
uint8_t rxLen      = 0;
bool    rxOverflow = false;

// ============================================================
// ENCODER (quadrature 4x)
// ============================================================
// index = (stateLama << 2) | stateBaru, state = (A << 1) | B
// 2 = transisi ilegal (dua pin berubah sekaligus -> pulsa terlewat)
static const int8_t QDEC[16] = { 0, -1,  1,  2,
                                 1,  0,  2, -1,
                                -1,  2,  0,  1,
                                 2,  1, -1,  0 };

volatile int32_t  encL = 0, encR = 0;
volatile uint16_t errL = 0, errR = 0;
volatile uint8_t  stL = 0,  stR = 0;

// Kiri: D2 = PD2 (A), D3 = PD3 (B)
void isrEncL() {
  uint8_t p = PIND;
  uint8_t s = (((p >> 2) & 0x01) << 1) | ((p >> 3) & 0x01);   // (A<<1)|B
  int8_t d = QDEC[(stL << 2) | s];
  if (d == 2) errL++; else encL += d;
  stL = s;
}

// Kanan: D10 = PB2 (A), D11 = PB3 (B), di-sampling 20 kHz
ISR(TIMER2_COMPA_vect) {
  uint8_t p = PINB;
  uint8_t s = (((p >> 2) & 0x01) << 1) | ((p >> 3) & 0x01);
  if (s == stR) return;
  int8_t d = QDEC[(stR << 2) | s];
  if (d == 2) errR++; else encR += d;
  stR = s;
}

static void timer2Start20k() {
  cli();
  TCCR2A = (1 << WGM21);          // CTC
  TCCR2B = (1 << CS21);           // prescaler 8 -> 2 MHz
  OCR2A  = 99;                    // 2 MHz / 100 = 20 kHz
  TCNT2  = 0;
  TIMSK2 = (1 << OCIE2A);
  sei();
}

static void encRead(int32_t& l, int32_t& r, uint16_t& xl, uint16_t& xr) {
  uint8_t sreg = SREG; cli();
  l = encL; r = encR; xl = errL; xr = errR;
  SREG = sreg;
#if ENC_L_INVERT
  l = -l;
#endif
#if ENC_R_INVERT
  r = -r;
#endif
}

static void encReset() {
  uint8_t sreg = SREG; cli();
  encL = encR = 0; errL = errR = 0;
  SREG = sreg;
}

// ============================================================
// AKTUATOR
// ============================================================
static int8_t toDir(long v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }

// inF HIGH = maju, inR HIGH = mundur. EN dimatikan dulu saat ganti arah.
static void driveOne(uint8_t en, uint8_t inF, uint8_t inR, int8_t dir) {
  digitalWrite(en, LOW);
  digitalWrite(inF, dir > 0 ? HIGH : LOW);
  digitalWrite(inR, dir < 0 ? HIGH : LOW);
  if (dir != 0) digitalWrite(en, HIGH);
}

static void applyMotors(int8_t mk, int8_t mr) {
  driveOne(MK_EN, MK_INF, MK_INR, mk);
  driveOne(MR_EN, MR_INF, MR_INR, mr);
  motorKiriDir  = mk;
  motorKananDir = mr;
}

// Hanya tulis pin bila berubah (heartbeat tidak mengganggu motor)
void setMotors(int8_t mk, int8_t mr, uint8_t ms) {
  if (ms == 0) { mk = 0; mr = 0; }
  if (mk == motorKiriDir && mr == motorKananDir && ms == motorSpeed) return;
  applyMotors(mk, mr);
  motorSpeed = ms;
}

static void forceServo(uint8_t a) {
  servoAngle = a;
  servoSteering.write(a);
}

void setServo(long angle) {
  uint8_t a = (uint8_t)constrain(angle, SERVO_MIN, SERVO_MAX);
  if (a == servoAngle) return;
  forceServo(a);
}

void setCutter(bool on) {
  cutterOn = on;
  digitalWrite(CUTTER_PIN, on ? HIGH : LOW);
}

void emergencyStop() {
  applyMotors(0, 0);
  motorSpeed = 0;
  setCutter(false);
}

// ============================================================
// JSON MINI-PARSER
// ============================================================
static const char* findValue(const char* js, const char* key) {
  size_t kl = strlen(key);
  const char* p = js;
  while ((p = strchr(p, '"')) != NULL) {
    if (strncmp(p + 1, key, kl) == 0 && p[1 + kl] == '"') {
      const char* q = p + 2 + kl;
      while (*q == ' ') q++;
      if (*q == ':') {
        q++;
        while (*q == ' ') q++;
        return q;
      }
    }
    p++;
  }
  return NULL;
}

static bool jsonGetInt(const char* js, const char* key, long& out) {
  const char* p = findValue(js, key);
  if (!p) return false;
  char* end;
  long v = strtol(p, &end, 10);
  if (end == p) return false;
  out = v;
  return true;
}

static bool jsonGetStr(const char* js, const char* key, char* out, size_t n) {
  const char* p = findValue(js, key);
  if (!p || *p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"') {
    if (i + 1 >= n) return false;
    out[i++] = *p++;
  }
  if (*p != '"') return false;
  out[i] = '\0';
  return true;
}

static bool frameValid(const char* s, uint8_t n) {
  if (n < 2 || s[0] != '{' || s[n - 1] != '}') return false;
  for (uint8_t i = 0; i < n; i++) {
    if (s[i] < 0x20 || s[i] > 0x7E) return false;
  }
  return true;
}

// ============================================================
// DIAGNOSTIK
// ============================================================
static void testDrive(int8_t mk, int8_t mr) {
  applyMotors(mk, mr);
  motorSpeed = (mk || mr) ? 255 : 0;
  testActive = (mk || mr);
  testUntil  = millis() + TEST_MS;
}

static void runTest(char a) {
  setCutter(false);
  if (a != 'j' && a != 'k' && a != 'l' && a != 'v' && a != 'p' && a != 'r') sweepDir = 0;
  switch (a) {
    case 'q': testDrive( 1,  0); break;   // kiri maju
    case 'a': testDrive(-1,  0); break;   // kiri mundur
    case 'e': testDrive( 0,  1); break;   // kanan maju
    case 'd': testDrive( 0, -1); break;   // kanan mundur
    case 'w': testDrive( 1,  1); break;
    case 's': testDrive(-1, -1); break;
    case 'x': emergencyStop(); testActive = false; sweepDir = 0; break;
    case 'j': forceServo(SERVO_MIN);    break;
    case 'k': forceServo(SERVO_CENTER); break;
    case 'l': forceServo(SERVO_MAX);    break;
    case 'v': sweepDir = 1; sweepPos = SERVO_MIN; sweepT = millis(); forceServo(SERVO_MIN); break;
    case 'r': encReset(); break;
    case 'p': break;
    default:  return;
  }
  diagAct = a;
  diagPending = true;
}

// ============================================================
// TX TERSINKRON FRAME SERVO (anti-jitter)
// ============================================================
// Library Servo (AVR, 16 MHz): Timer1 prescaler 8 -> 1 tick = 0,5 us,
// TCNT1 di-reset 0 di awal frame, pulsa servo mulai ~0 dan berakhir di
// ~readMicroseconds()*2 tick, frame 20 ms = 40000 tick.
#define SERVO_FRAME_TICKS   40000u
#define SAFE_AFTER_PULSE      800u   // +400 us setelah pulsa selesai
#define SAFE_BEFORE_FRAME    1600u   // byte 520 us harus selesai >280 us sebelum frame baru

static uint16_t readTcnt1() {
  uint8_t sreg = SREG; cli();
  uint16_t t = TCNT1;
  SREG = sreg;
  return t;
}

static void sendPaced(const char* s) {
  uint16_t pulseEnd = (uint16_t)servoSteering.readMicroseconds() * 2u + SAFE_AFTER_PULSE;
  for (; *s; s++) {
    unsigned long t0 = micros();
    for (;;) {
      uint16_t t = readTcnt1();
      if (t > pulseEnd && t < SERVO_FRAME_TICKS - SAFE_BEFORE_FRAME) break;
      if (micros() - t0 > 25000UL) break;      // pengaman: jangan pernah macet
    }
    espSerial.write(*s);
  }
  espSerial.write('\n');
}

static uint8_t outState(uint8_t pin) {
  return (*portOutputRegister(digitalPinToPort(pin)) & digitalPinToBitMask(pin)) ? 1 : 0;
}

static void sendDiag() {
  int32_t l, r; uint16_t xl, xr;
  encRead(l, r, xl, xr);
  char b[224];
  snprintf(b, sizeof(b),
    "{\"dg\":1,\"a\":\"%c\",\"p\":\"KIRI EN(D9)=%u MAJU(D8)=%u MUNDUR(D7)=%u | "
    "KANAN EN(D6)=%u MAJU(D4)=%u MUNDUR(D5)=%u\","
    "\"sv\":%u,\"rx\":%u,\"bf\":%u,\"wd\":%u,\"el\":%ld,\"er\":%ld,\"xl\":%u,\"xr\":%u}",
    diagAct,
    outState(MK_EN), outState(MK_INF), outState(MK_INR),
    outState(MR_EN), outState(MR_INF), outState(MR_INR),
    (unsigned)servoAngle, (unsigned)rxCount, (unsigned)badFrames,
    wdLatched ? 1u : 0u, (long)l, (long)r, (unsigned)xl, (unsigned)xr);
  sendPaced(b);
}

// ============================================================
// PROSES COMMAND
// ============================================================
void processLine(const char* s, uint8_t n) {
  if (!frameValid(s, n)) { badFrames++; DBGLN(F("[RX] frame rusak")); return; }

  char cmd[10];
  if (!jsonGetStr(s, "cmd", cmd, sizeof(cmd))) { badFrames++; return; }

  long mk, mr, ms, sv, ct;
  bool ok = true;

  if (strcmp(cmd, "stop") == 0) {
    emergencyStop();
    testActive = false; sweepDir = 0;
    wdLatched = false;
  }
  else if (strcmp(cmd, "ping") == 0) {
  }
  else if (strcmp(cmd, "test") == 0) {
    char a[4];
    if (jsonGetStr(s, "a", a, sizeof(a)) && a[0]) runTest(a[0]);
    else ok = false;
  }
  else if (strcmp(cmd, "drive") == 0) {
    if (jsonGetInt(s, "mk", mk) && jsonGetInt(s, "mr", mr) && jsonGetInt(s, "ms", ms)) {
      testActive = false; sweepDir = 0;
      if (jsonGetInt(s, "sv", sv)) setServo(sv);
      if (!wdLatched) {
        setMotors(toDir(mk), toDir(mr), ms > 0 ? 255 : 0);
        if (jsonGetInt(s, "ct", ct)) setCutter(ct > 0);
      }
    } else ok = false;
  }
  else if (strcmp(cmd, "motor") == 0) {
    if (jsonGetInt(s, "mk", mk) && jsonGetInt(s, "mr", mr) && jsonGetInt(s, "ms", ms)) {
      testActive = false;
      if (!wdLatched) setMotors(toDir(mk), toDir(mr), ms > 0 ? 255 : 0);
    } else ok = false;
  }
  else if (strcmp(cmd, "servo") == 0) {
    if (jsonGetInt(s, "sv", sv)) setServo(sv); else ok = false;
  }
  else if (strcmp(cmd, "cutter") == 0) {
    if (jsonGetInt(s, "ct", ct)) { if (!wdLatched) setCutter(ct > 0); }
    else ok = false;
  }
  else ok = false;

  if (!ok) { badFrames++; DBG(F("[RX] ditolak: ")); DBGLN(s); return; }

  rxCount++;
  lastCmdMs    = millis();
  telRequested = true;
}

// ============================================================
// TELEMETRI
// ============================================================
void sendTelemetry() {
  int32_t l, r; uint16_t xl, xr;
  encRead(l, r, xl, xr);
  uint8_t eff = (motorKiriDir || motorKananDir) ? 255 : 0;
  char tx[128];
  snprintf(tx, sizeof(tx),
           "{\"mk\":%d,\"mr\":%d,\"ms\":%u,\"sv\":%u,\"ct\":%u,\"wd\":%u,\"bf\":%u,"
           "\"el\":%ld,\"er\":%ld,\"up\":%lu}",
           (int)motorKiriDir, (int)motorKananDir, (unsigned)eff,
           (unsigned)servoAngle, cutterOn ? 1u : 0u, wdLatched ? 1u : 0u,
           (unsigned)badFrames, (long)l, (long)r,
           (unsigned long)(millis() / 100UL));
  sendPaced(tx);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  pinMode(CUTTER_PIN, OUTPUT);
  digitalWrite(CUTTER_PIN, LOW);

  pinMode(MK_EN, OUTPUT); pinMode(MK_INF, OUTPUT); pinMode(MK_INR, OUTPUT);
  pinMode(MR_EN, OUTPUT); pinMode(MR_INF, OUTPUT); pinMode(MR_INR, OUTPUT);
  applyMotors(0, 0);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  delay(5);
  { uint8_t p = PIND; stL = (((p >> 2) & 0x01) << 1) | ((p >> 3) & 0x01); }
  { uint8_t p = PINB; stR = (((p >> 2) & 0x01) << 1) | ((p >> 3) & 0x01); }
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isrEncL, CHANGE);

  servoSteering.attach(SERVO_PIN);
  forceServo(SERVO_CENTER);

  timer2Start20k();

  Serial.begin(USB_BAUD);
  DBGLN(F("[ELITA] Uno v4.1 boot OK (motor ON/OFF, encoder, servo anti-jitter)"));

  espSerial.begin(ESP_BAUD);
  lastCmdMs = millis();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // --- 1. Terima command ---
  while (espSerial.available()) {
    char c = (char)espSerial.read();
    lastRxByteMs = millis();
    if (c == '\r') continue;
    if (c == '\n') {
      if (rxOverflow)     badFrames++;
      else if (rxLen > 0) { rxBuf[rxLen] = '\0'; processLine(rxBuf, rxLen); }
      rxLen = 0;
      rxOverflow = false;
      continue;
    }
    if (rxLen < RX_BUF_LEN - 1) rxBuf[rxLen++] = c;
    else rxOverflow = true;
  }
  if (espSerial.overflow()) badFrames++;

  unsigned long now = millis();

  if ((rxLen > 0 || rxOverflow) && (now - lastRxByteMs > RX_STALE_MS)) {
    badFrames++;
    rxLen = 0;
    rxOverflow = false;
  }

  // --- 2a. Tes: auto-stop & sweep servo ---
  if (testActive && (long)(now - testUntil) >= 0) {
    applyMotors(0, 0);
    motorSpeed = 0;
    testActive = false;
    diagAct = 't';
    diagPending = true;
  }
  if (sweepDir && now - sweepT >= SWEEP_STEP_MS) {
    sweepT = now;
    if (sweepDir > 0) {
      if (sweepPos < SERVO_MAX) sweepPos++; else sweepDir = -1;
    } else {
      if (sweepPos > SERVO_MIN) sweepPos--;
      else { sweepDir = 0; sweepPos = SERVO_CENTER; diagAct = 'V'; diagPending = true; }
    }
    forceServo(sweepPos);
  }

  // --- 2b. Watchdog (latch), dikecualikan saat tes ---
  if (!wdLatched && !testActive && (motorKiriDir || motorKananDir || cutterOn) &&
      (now - lastCmdMs > WATCHDOG_MS)) {
    emergencyStop();
    wdLatched = true;
    DBGLN(F("[WD] tidak ada command > 1 s -> STOP + latch"));
  }

  // --- 3. Kirim diag / telemetri saat jalur RX diam ---
  bool rxIdle = (rxLen == 0) && !rxOverflow && (espSerial.available() == 0);
  if (rxIdle && diagPending) {
    sendDiag();
    diagPending = false;
    lastTelMs   = now;
  }
  else if (rxIdle) {
    unsigned long dt = now - lastTelMs;
    if ((telRequested && dt >= TEL_MIN_MS) || dt >= TEL_IDLE_MS) {
      sendTelemetry();
      lastTelMs    = now;
      telRequested = false;
    }
  }
}