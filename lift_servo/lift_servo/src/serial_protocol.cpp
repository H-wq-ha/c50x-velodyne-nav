#include "serial_protocol.h"

#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

namespace {

char* nextToken(char* first, char** context) {
  return strtok_r(first, " \t", context);
}

void uppercase(char* text) {
  while (*text != '\0') {
    *text = static_cast<char>(toupper(static_cast<unsigned char>(*text)));
    ++text;
  }
}

}  // namespace

void SerialProtocol::begin() {
  serial_.println("READY LIFT_SERVO V1");
  sendStatus();
}

void SerialProtocol::update() {
  while (serial_.available() > 0) {
    const int input = serial_.read();
    if (input < 0) {
      return;
    }
    const char ch = static_cast<char>(input);
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      if (discarding_) {
        discarding_ = false;
        length_ = 0;
        error("LINE_TOO_LONG", "maximum=95");
      } else if (length_ > 0) {
        line_[length_] = '\0';
        processLine(line_);
        length_ = 0;
      }
      continue;
    }
    if (discarding_) {
      continue;
    }
    if (length_ >= kMaxLineLength) {
      discarding_ = true;
      continue;
    }
    line_[length_++] = ch;
  }
}

void SerialProtocol::processLine(char* line) {
  uppercase(line);
  char* context = nullptr;
  const char* command = nextToken(line, &context);
  if (command == nullptr) {
    return;
  }

  // Every syntactically complete command is also a watchdog heartbeat.
  motor_.keepAlive();

  if (strcmp(command, "PING") == 0 || strcmp(command, "KEEPALIVE") == 0) {
    if (!noExtraTokens(context)) return;
    ok(command);
    return;
  }
  if (strcmp(command, "HELP") == 0) {
    if (!noExtraTokens(context)) return;
    sendHelp();
    return;
  }
  if (strcmp(command, "STATUS") == 0) {
    if (!noExtraTokens(context)) return;
    sendStatus();
    return;
  }
  if (strcmp(command, "ENABLE") == 0) {
    if (!noExtraTokens(context)) return;
    if (motor_.enable()) {
      ok(command);
    } else {
      error("SAFETY_LATCHED", "use CLEAR after alarm is inactive");
    }
    return;
  }
  if (strcmp(command, "DISABLE") == 0) {
    if (!noExtraTokens(context)) return;
    motor_.disable();
    ok(command);
    return;
  }
  if (strcmp(command, "STOP") == 0) {
    if (!noExtraTokens(context)) return;
    motor_.stop();
    ok(command);
    return;
  }
  if (strcmp(command, "ESTOP") == 0) {
    if (!noExtraTokens(context)) return;
    motor_.emergencyStop();
    ok(command);
    return;
  }
  if (strcmp(command, "CLEAR") == 0) {
    if (!noExtraTokens(context)) return;
    if (motor_.clearSafetyLatch()) {
      ok(command);
    } else {
      error("ALARM_ACTIVE", "drive ALM input is still asserted");
    }
    return;
  }

  if (strcmp(command, "SPEED") == 0) {
    const char* speedToken = nextToken(nullptr, &context);
    uint32_t speed = 0;
    if (speedToken == nullptr) {
      error("ARGUMENT", "usage: SPEED <pps>");
      return;
    }
    if (!parseUint32(speedToken, speed) || !noExtraTokens(context)) {
      return;
    }
    if (!motor_.setSpeed(speed)) {
      error("RANGE", "speed_pps must be 1..20000");
      return;
    }
    ok(command);
    return;
  }

  if (strcmp(command, "WATCHDOG") == 0) {
    const char* timeoutToken = nextToken(nullptr, &context);
    uint32_t timeout = 0;
    if (timeoutToken == nullptr) {
      error("ARGUMENT", "usage: WATCHDOG <0..60000_ms>");
      return;
    }
    if (!parseUint32(timeoutToken, timeout) || !noExtraTokens(context)) {
      return;
    }
    if (!motor_.setWatchdog(timeout)) {
      error("RANGE", "watchdog_ms must be 0..60000");
      return;
    }
    ok(command);
    return;
  }

  if (strcmp(command, "RUN") == 0) {
    const char* directionToken = nextToken(nullptr, &context);
    const char* speedToken = nextToken(nullptr, &context);
    MotorController::Direction direction;
    uint32_t speed = motor_.speedPps();
    if (directionToken == nullptr ||
        !parseDirection(directionToken, direction)) {
      error("ARGUMENT", "usage: RUN <FWD|REV> [pps]");
      return;
    }
    if (speedToken != nullptr && !parseUint32(speedToken, speed)) {
      return;
    }
    if (!noExtraTokens(context)) return;
    if (!motor_.run(direction, speed)) {
      error("REJECTED", "check speed range, ALM, ESTOP and CLEAR state");
      return;
    }
    ok(command);
    return;
  }

  if (strcmp(command, "MOVE") == 0) {
    const char* directionToken = nextToken(nullptr, &context);
    const char* pulsesToken = nextToken(nullptr, &context);
    const char* speedToken = nextToken(nullptr, &context);
    MotorController::Direction direction;
    uint32_t pulses = 0;
    uint32_t speed = motor_.speedPps();
    if (directionToken == nullptr || pulsesToken == nullptr ||
        !parseDirection(directionToken, direction)) {
      error("ARGUMENT", "usage: MOVE <FWD|REV> <pulses> [pps]");
      return;
    }
    if (!parseUint32(pulsesToken, pulses)) {
      return;
    }
    if (speedToken != nullptr && !parseUint32(speedToken, speed)) {
      return;
    }
    if (!noExtraTokens(context)) return;
    if (!motor_.move(direction, pulses, speed)) {
      error("REJECTED", "check pulses, speed, ALM, ESTOP and CLEAR state");
      return;
    }
    ok(command);
    return;
  }

  error("UNKNOWN_COMMAND", command);
}

void SerialProtocol::sendStatus() {
  serial_.print("STATUS state=");
  serial_.print(MotorController::stateName(motor_.state()));
  serial_.print(" dir=");
  serial_.print(MotorController::directionName(motor_.direction()));
  serial_.print(" enabled=");
  serial_.print(motor_.driveEnabled() ? 1 : 0);
  serial_.print(" alarm=");
  serial_.print(motor_.alarmAsserted() ? 1 : 0);
  serial_.print(" pend=");
  serial_.print(motor_.inPosition() ? 1 : 0);
  serial_.print(" speed_pps=");
  serial_.print(motor_.speedPps());
  serial_.print(" sent=");
  serial_.print(motor_.sentPulses());
  serial_.print(" mode=");
  serial_.print(motor_.continuous() ? "CONTINUOUS" : "FINITE");
  serial_.print(" remaining=");
  serial_.print(motor_.remainingPulses());
  serial_.print(" watchdog_ms=");
  serial_.print(motor_.watchdogMs());
  serial_.print(" reason=");
  serial_.println(motor_.lastStopReason());
}

void SerialProtocol::sendHelp() {
  serial_.println("CMDS PING | KEEPALIVE | STATUS | ENABLE | DISABLE | STOP");
  serial_.println("CMDS ESTOP | CLEAR | SPEED <pps> | WATCHDOG <ms>");
  serial_.println("CMDS RUN <FWD|REV> [pps]");
  serial_.println("CMDS MOVE <FWD|REV> <pulses> [pps]");
}

void SerialProtocol::ok(const char* command) {
  serial_.print("OK ");
  serial_.println(command);
}

void SerialProtocol::error(const char* code, const char* detail) {
  serial_.print("ERR ");
  serial_.print(code);
  serial_.print(' ');
  serial_.println(detail);
}

bool SerialProtocol::parseDirection(const char* token,
                                    MotorController::Direction& direction) {
  if (strcmp(token, "FWD") == 0 || strcmp(token, "FORWARD") == 0) {
    direction = MotorController::Direction::Forward;
    return true;
  }
  if (strcmp(token, "REV") == 0 || strcmp(token, "REVERSE") == 0) {
    direction = MotorController::Direction::Reverse;
    return true;
  }
  return false;
}

bool SerialProtocol::parseUint32(const char* token, uint32_t& value) {
  if (token == nullptr || *token == '\0' || *token == '-') {
    error("NUMBER", "expected an unsigned integer");
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const unsigned long parsed = strtoul(token, &end, 10);
  if (errno == ERANGE || end == token || *end != '\0' ||
      parsed > 0xFFFFFFFFUL) {
    error("NUMBER", token);
    return false;
  }
  value = static_cast<uint32_t>(parsed);
  return true;
}

bool SerialProtocol::noExtraTokens(char* context) {
  if (nextToken(nullptr, &context) == nullptr) {
    return true;
  }
  error("ARGUMENT", "too many arguments");
  return false;
}
