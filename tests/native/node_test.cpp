#include <cassert>
#include "../../firmware/gello_node/gello_node.ino"

void feed(std::vector<uint8_t> bytes) { bus.input.assign(bytes.begin(), bytes.end()); }
std::vector<uint8_t> status(uint8_t id, uint8_t error) {
  std::vector<uint8_t> p = {0xff, 0xff, 0xfd, 0, id, 8, 0, 0x55, error, 0, 8, 0, 0};
  uint16_t crc = crc16(p.data(), p.size());
  p.push_back(crc & 255); p.push_back(crc >> 8);
  return p;
}
int main() {
  uint8_t r[4] = {};
  feed({0xff, 0xff, 0xfd, 0, 1, 0, 0});
  assert(rxPacket(1, r, sizeof r) == -1);  // short lengths previously underflowed memcpy size
  feed({0xff, 0xff, 0xfd, 0, 1, 255, 255});
  assert(rxPacket(1, r, sizeof r) == -1);
  feed(status(2, 0)); assert(rxPacket(1, r, sizeof r) == -1);
  feed(status(1, 0x80)); assert(rxPacket(1, r, sizeof r) == -1);
  feed(status(1, 0)); assert(rxPacket(1, r, sizeof r) == 4);
  assert(r[0] == 0 && r[1] == 8);
  auto corrupt = status(1, 0); corrupt[9] ^= 1;
  feed(corrupt); assert(rxPacket(1, r, sizeof r) == -1);
  feed(status(1, 0)); assert(rxPacket(1, r, 2) == -1);
  bus.input.clear();
  setup();
  Serial.input.push_back('t'); loop();
  int torque_writes = 0;
  for (auto& p : bus.output) {
    if (p[7] == 3 && p[8] == 64 && p[9] == 0) {
      ++torque_writes; assert(p.size() == 13 && p[10] == 0);
    }
  }
  assert(torque_writes == 14);  // boot and a rejected parking request each disable all seven
}
