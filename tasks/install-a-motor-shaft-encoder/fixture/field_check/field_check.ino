#include <Wire.h>

constexpr uint8_t SENSOR_ADDRESS = 0x36;
constexpr uint8_t STATUS_REGISTER = 0x0B;
constexpr uint8_t MAGNET_FLAGS = 0x38;
constexpr uint8_t MAGNET_DETECTED = 0x20;
constexpr uint8_t TEST_KEY = 2;
constexpr uint8_t PASS_LED = 8;
constexpr uint8_t FAIL_LED = 9;
constexpr unsigned long HOLD_MS = 300;

bool holding = false;
bool sampled = false;
unsigned long pressedAt = 0;

bool fieldPasses() {
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(STATUS_REGISTER);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(SENSOR_ADDRESS, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  const int status = Wire.read();
  return status >= 0 && (status & MAGNET_FLAGS) == MAGNET_DETECTED;
}

void setup() {
  pinMode(TEST_KEY, INPUT_PULLUP);
  pinMode(PASS_LED, OUTPUT);
  pinMode(FAIL_LED, OUTPUT);
  Wire.begin();
  Wire.setWireTimeout(25000, true);
}

void loop() {
  if (digitalRead(TEST_KEY) == HIGH) {
    holding = false;
    return;
  }
  if (!holding) {
    pressedAt = millis();
    holding = true;
    sampled = false;
    digitalWrite(PASS_LED, LOW);
    digitalWrite(FAIL_LED, LOW);
  }
  if (!sampled && millis() - pressedAt >= HOLD_MS) {
    const bool pass = fieldPasses();
    digitalWrite(PASS_LED, pass ? HIGH : LOW);
    digitalWrite(FAIL_LED, pass ? LOW : HIGH);
    sampled = true;
  }
}
