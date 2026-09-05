#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <deque>
#include <vector>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define SERIAL_8N1 0
inline uint32_t test_clock = 0;
inline uint32_t micros() { return test_clock++; }
inline uint32_t millis() { return test_clock / 1000; }
inline void delay(int n) { test_clock += n * 1000; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
struct HardwareSerial {
  std::deque<uint8_t> input;
  std::vector<std::vector<uint8_t>> output;
  HardwareSerial(int = 0) {}
  void begin(uint32_t, int = 0, int = 0, int = 0) {}
  size_t available() { return input.size(); }
  int read() { int v = input.front(); input.pop_front(); return v; }
  void write(const uint8_t* p, size_t n) { output.emplace_back(p, p + n); }
  void flush() {}
  void println(const char*) {}
  template<class... T> void printf(const char*, T...) {}
};
inline HardwareSerial Serial;
