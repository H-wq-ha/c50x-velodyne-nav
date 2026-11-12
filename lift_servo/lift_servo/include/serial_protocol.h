#pragma once

#include <Arduino.h>

#include "motor_controller.h"

class SerialProtocol {
 public:
  SerialProtocol(Stream& serial, MotorController& motor)
      : serial_(serial), motor_(motor) {}

  void begin();
  void update();

 private:
  static constexpr size_t kMaxLineLength = 95;

  void processLine(char* line);
  void sendStatus();
  void sendHelp();
  void ok(const char* command);
  void error(const char* code, const char* detail);
  bool parseDirection(const char* token, MotorController::Direction& direction);
  bool parseUint32(const char* token, uint32_t& value);
  bool noExtraTokens(char* context);

  Stream& serial_;
  MotorController& motor_;
  char line_[kMaxLineLength + 1] = {};
  size_t length_ = 0;
  bool discarding_ = false;
};
