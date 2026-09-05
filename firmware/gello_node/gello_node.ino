// GELLO leader arm node: ESP32-S3-DevKitC-1 reads seven DYNAMIXEL X330 servos (Protocol 2.0, 1 Mbps) with a
// 74AHCT125 as the half-duplex TTL buffer, and streams their positions over USB CDC at 100 Hz.
//
//   line format (ASCII, one per sample):  G,<seq>,<t_ms>,<p1>,<p2>,<p3>,<p4>,<p5>,<p6>,<p7>,<ok_mask>\n
//   p = raw 12-bit position (0..4095 = 360 deg), 65535 when that servo did not answer; ok_mask bit i = servo i replied
//
// Wiring: see docs/electronics.md. The 5 V AHCT output must not directly feed the ESP32 RX.
// This revision is a passive encoder reader. Torque remains OFF; 't' is rejected.
// Calibration lives in the host's calibration.json; 'z' does not write a pretend device zero.
#include <Arduino.h>

static const int PIN_TX = 17, PIN_RX = 18, PIN_DIR = 8;     // VERIFY against the board silkscreen
static const uint8_t IDS[7] = {1, 2, 3, 4, 5, 6, 7};
static const uint32_t BAUD = 1000000;
static const uint16_t ADDR_TORQUE_ENABLE = 64, ADDR_PRESENT_POSITION = 132;
static const int RATE_HZ = 100;

HardwareSerial bus(1);
uint32_t seq = 0;

// --- Protocol 2.0 ------------------------------------------------------------------------------------------------
uint16_t crc16(const uint8_t *d, size_t n) {
  static uint16_t tbl[256];
  static bool init = false;
  if (!init) {                                   // the DYNAMIXEL CRC-16/BUYPASS table
    for (int i = 0; i < 256; i++) { uint16_t c = i << 8; for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (c << 1) ^ 0x8005 : c << 1; tbl[i] = c; }
    init = true;
  }
  uint16_t crc = 0;
  for (size_t i = 0; i < n; i++) crc = (crc << 8) ^ tbl[((crc >> 8) ^ d[i]) & 0xFF];
  return crc;
}

void txPacket(uint8_t id, uint8_t instr, const uint8_t *params, uint16_t plen) {
  uint8_t pkt[64];
  uint16_t len = plen + 3;
  pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = 0xFD; pkt[3] = 0x00; pkt[4] = id; pkt[5] = len & 0xFF; pkt[6] = len >> 8; pkt[7] = instr;
  memcpy(pkt + 8, params, plen);
  uint16_t crc = crc16(pkt, 8 + plen);
  pkt[8 + plen] = crc & 0xFF; pkt[9 + plen] = crc >> 8;
  digitalWrite(PIN_DIR, HIGH);
  bus.write(pkt, 10 + plen);
  bus.flush();
  digitalWrite(PIN_DIR, LOW);
}

// read a status packet; returns the parameter bytes' count or -1
int rxPacket(uint8_t id, uint8_t *params, size_t cap, uint32_t timeout_us = 1500) {
  uint8_t buf[64]; size_t n = 0; uint32_t t0 = micros();
  while (micros() - t0 < timeout_us) {
    while (bus.available() && n < sizeof(buf)) buf[n++] = bus.read();
    if (n >= 7) {
      uint16_t len = buf[5] | (buf[6] << 8);
      if (len < 4 || len > sizeof(buf) - 7) return -1;
      if (n >= (size_t)(7 + len)) {
        if (buf[0] != 0xFF || buf[1] != 0xFF || buf[2] != 0xFD || buf[3] != 0 || buf[4] != id || buf[7] != 0x55) return -1;
        uint16_t crc = crc16(buf, 5 + len);
        if ((buf[5 + len] | (buf[6 + len] << 8)) != crc) return -1;
        if (buf[8] != 0) return -1;             // hardware error flag / alert
        int pn = len - 4;                         // instr, error, crc(2)
        if (pn > (int)cap) return -1;
        memcpy(params, buf + 9, pn);
        return pn;
      }
    }
  }
  return -1;
}

void writeReg(uint8_t id, uint16_t addr, const uint8_t *val, uint8_t n) {
  uint8_t p[8] = {(uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8)};
  memcpy(p + 2, val, n);
  txPacket(id, 0x03, p, 2 + n);
  uint8_t r[8]; rxPacket(id, r, sizeof r);
}

bool readPosition(uint8_t id, int32_t &pos) {
  uint8_t p[4] = {(uint8_t)(ADDR_PRESENT_POSITION & 0xFF), (uint8_t)(ADDR_PRESENT_POSITION >> 8), 4, 0};
  txPacket(id, 0x02, p, 4);
  uint8_t r[4];
  if (rxPacket(id, r, 4) != 4) return false;
  pos = (int32_t)(r[0] | (r[1] << 8) | ((uint32_t)r[2] << 16) | ((uint32_t)r[3] << 24));
  return true;
}

void disableTorque() {
  uint8_t v = 0;
  for (uint8_t id : IDS) writeReg(id, ADDR_TORQUE_ENABLE, &v, 1);
}

void setup() {
  Serial.begin(921600);
  pinMode(PIN_DIR, OUTPUT); digitalWrite(PIN_DIR, LOW);
  bus.begin(BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(200);
  disableTorque();
}

void loop() {
  static uint32_t next = 0;
  uint32_t now = micros();
  if ((int32_t)(now - next) < 0) return;
  next = now + 1000000UL / RATE_HZ;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 't') { disableTorque(); Serial.println("E,passive_encoder_only"); }
    if (c == 'z') Serial.println("E,use_host_calibration");
  }

  int32_t pos[7]; uint8_t ok = 0;
  for (int i = 0; i < 7; i++) {
    if (readPosition(IDS[i], pos[i])) ok |= 1 << i; else pos[i] = 65535;
  }
  Serial.printf("G,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u\n", (unsigned long)seq++, (unsigned long)millis(),
                (long)pos[0], (long)pos[1], (long)pos[2], (long)pos[3], (long)pos[4], (long)pos[5], (long)pos[6], ok);
}
