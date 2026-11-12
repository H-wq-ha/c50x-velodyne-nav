#include "motor_controller.h"

#include "app_config.h"

namespace {

bool timeReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

uint8_t oppositeLevel(uint8_t level) { return level == HIGH ? LOW : HIGH; }

}  // namespace

void MotorController::begin() {
  // Put command outputs into safe states before changing their pin modes.
  digitalWrite(AppConfig::kPulsePin, oppositeLevel(AppConfig::kPulseActiveLevel));
  digitalWrite(AppConfig::kDirectionPin,
               oppositeLevel(AppConfig::kForwardDirectionLevel));

  const uint8_t disabledLevel =
      AppConfig::kEnaSignalDisablesDrive
          ? AppConfig::kEnaAssertedLevel
          : oppositeLevel(AppConfig::kEnaAssertedLevel);
  digitalWrite(AppConfig::kEnablePin, disabledLevel);

  pinMode(AppConfig::kPulsePin, OUTPUT);
  pinMode(AppConfig::kDirectionPin, OUTPUT);
  pinMode(AppConfig::kEnablePin, OUTPUT);
  pinMode(AppConfig::kPendPin, INPUT_PULLUP);
  pinMode(AppConfig::kAlarmPin, INPUT_PULLUP);

  state_ = State::Disabled;
  driveEnabled_ = false;
  speedPps_ = AppConfig::kDefaultSpeedPps;
  setSpeed(speedPps_);
  watchdogMs_ = AppConfig::kDefaultWatchdogMs;
  lastKeepAliveMs_ = millis();
  lastStopReason_ = "POWER_ON";
}

void MotorController::update() {
  if (alarmAsserted() && state_ != State::Fault) {
    enterFault("DRIVE_ALARM");
    return;
  }

  const uint32_t nowMs = millis();
  if (watchdogExpired(nowMs)) {
    stopMotion("WATCHDOG");
    return;
  }

  const uint32_t nowUs = micros();
  if (state_ == State::Enabling) {
    if (!timeReached(nowUs, startAtUs_)) {
      return;
    }
    applyDirection();
    startAtUs_ = nowUs + AppConfig::kDirectionSetupUs;
    state_ = State::Starting;
    return;
  }

  if (state_ == State::Starting) {
    if (!timeReached(nowUs, startAtUs_)) {
      return;
    }
    state_ = State::Running;
    nextEdgeUs_ = nowUs;
  }

  if (state_ != State::Running || !timeReached(nowUs, nextEdgeUs_)) {
    return;
  }

  if (!pulseActive_) {
    setPulseActive(true);
    ++sentPulses_;
    if (!continuous_) {
      --remainingPulses_;
      stopAfterPulse_ = remainingPulses_ == 0U;
    }
    nextEdgeUs_ = nowUs + activeTimeUs_;
    return;
  }

  setPulseActive(false);
  if (stopAfterPulse_) {
    stopAfterPulse_ = false;
    state_ = State::Idle;
    lastStopReason_ = "MOVE_COMPLETE";
    return;
  }
  nextEdgeUs_ = nowUs + idleTimeUs_;
}

bool MotorController::enable() {
  if (!motionAllowed()) {
    return false;
  }
  setDriveEnabled(true);
  if (state_ == State::Disabled) {
    state_ = State::Idle;
  }
  lastStopReason_ = "NONE";
  keepAlive();
  return true;
}

void MotorController::disable() {
  stopMotion("DISABLED_BY_COMMAND");
  setDriveEnabled(false);
  if (state_ != State::Fault && state_ != State::EStop) {
    state_ = State::Disabled;
  }
}

void MotorController::stop() {
  if (state_ == State::Running || state_ == State::Starting ||
      state_ == State::Enabling) {
    stopMotion("STOP_COMMAND");
  }
  keepAlive();
}

void MotorController::emergencyStop() {
  stopMotion("EMERGENCY_STOP");
  setDriveEnabled(false);
  state_ = State::EStop;
}

bool MotorController::clearSafetyLatch() {
  if (alarmAsserted()) {
    return false;
  }
  setPulseActive(false);
  setDriveEnabled(false);
  state_ = State::Disabled;
  continuous_ = false;
  remainingPulses_ = 0;
  lastStopReason_ = "SAFETY_CLEARED";
  keepAlive();
  return true;
}

bool MotorController::run(Direction direction, uint32_t speedPps) {
  return requestMotion(direction, 0, speedPps, true);
}

bool MotorController::move(Direction direction, uint32_t pulses,
                           uint32_t speedPps) {
  if (pulses == 0U) {
    return false;
  }
  return requestMotion(direction, pulses, speedPps, false);
}

bool MotorController::setSpeed(uint32_t speedPps) {
  if (speedPps < AppConfig::kMinSpeedPps ||
      speedPps > AppConfig::kMaxSpeedPps) {
    return false;
  }

  speedPps_ = speedPps;
  const uint32_t periodUs = 1000000UL / speedPps_;
  activeTimeUs_ = periodUs / 2U;
  if (activeTimeUs_ < AppConfig::kMinPulseWidthUs) {
    activeTimeUs_ = AppConfig::kMinPulseWidthUs;
  }
  idleTimeUs_ = periodUs - activeTimeUs_;
  if (idleTimeUs_ < AppConfig::kMinPulseWidthUs) {
    idleTimeUs_ = AppConfig::kMinPulseWidthUs;
  }
  return true;
}

bool MotorController::setWatchdog(uint32_t timeoutMs) {
  if (timeoutMs > AppConfig::kMaxWatchdogMs) {
    return false;
  }
  watchdogMs_ = timeoutMs;
  keepAlive();
  return true;
}

void MotorController::keepAlive() { lastKeepAliveMs_ = millis(); }

bool MotorController::alarmAsserted() const {
  return digitalRead(AppConfig::kAlarmPin) == AppConfig::kAlarmAssertedLevel;
}

bool MotorController::inPosition() const {
  return digitalRead(AppConfig::kPendPin) == AppConfig::kPendAssertedLevel;
}

const char* MotorController::stateName(State state) {
  switch (state) {
    case State::Disabled:
      return "DISABLED";
    case State::Idle:
      return "IDLE";
    case State::Enabling:
      return "ENABLING";
    case State::Starting:
      return "STARTING";
    case State::Running:
      return "RUNNING";
    case State::Fault:
      return "FAULT";
    case State::EStop:
      return "ESTOP";
  }
  return "UNKNOWN";
}

const char* MotorController::directionName(Direction direction) {
  return direction == Direction::Forward ? "FWD" : "REV";
}

bool MotorController::requestMotion(Direction direction, uint32_t pulses,
                                    uint32_t speedPps, bool continuous) {
  if (!motionAllowed() || !setSpeed(speedPps)) {
    return false;
  }

  // A new RUN/MOVE command atomically replaces any current movement.
  stopMotion("REPLACED_BY_COMMAND");
  direction_ = direction;
  setDriveEnabled(true);
  continuous_ = continuous;
  remainingPulses_ = pulses;
  stopAfterPulse_ = false;
  lastStopReason_ = "NONE";
  keepAlive();

  const uint32_t nowUs = micros();
  if (timeReached(nowUs, enableReadyAtUs_)) {
    applyDirection();
    startAtUs_ = nowUs + AppConfig::kDirectionSetupUs;
    state_ = State::Starting;
  } else {
    // ENA is released first. DIR changes only after the conservative enable
    // lead time, then PUL waits for its own direction setup interval.
    startAtUs_ = enableReadyAtUs_;
    state_ = State::Enabling;
  }
  return true;
}

void MotorController::setDriveEnabled(bool enabled) {
  if (enabled && !driveEnabled_) {
    enableReadyAtUs_ = micros() + AppConfig::kEnableLeadUs;
  } else if (!enabled) {
    enableReadyAtUs_ = 0;
  }
  const bool assertEna = AppConfig::kEnaSignalDisablesDrive ? !enabled : enabled;
  digitalWrite(AppConfig::kEnablePin,
               assertEna ? AppConfig::kEnaAssertedLevel
                         : oppositeLevel(AppConfig::kEnaAssertedLevel));
  driveEnabled_ = enabled;
}

void MotorController::applyDirection() {
  digitalWrite(AppConfig::kDirectionPin,
               direction_ == Direction::Forward
                   ? AppConfig::kForwardDirectionLevel
                   : oppositeLevel(AppConfig::kForwardDirectionLevel));
}

void MotorController::setPulseActive(bool active) {
  digitalWrite(AppConfig::kPulsePin,
               active ? AppConfig::kPulseActiveLevel
                      : oppositeLevel(AppConfig::kPulseActiveLevel));
  pulseActive_ = active;
}

void MotorController::enterFault(const char* reason) {
  stopMotion(reason);
  setDriveEnabled(false);
  state_ = State::Fault;
}

void MotorController::stopMotion(const char* reason) {
  setPulseActive(false);
  continuous_ = false;
  remainingPulses_ = 0;
  stopAfterPulse_ = false;
  lastStopReason_ = reason;
  if (state_ == State::Running || state_ == State::Starting ||
      state_ == State::Enabling) {
    state_ = driveEnabled_ ? State::Idle : State::Disabled;
  }
}

bool MotorController::motionAllowed() const {
  return state_ != State::Fault && state_ != State::EStop && !alarmAsserted();
}

bool MotorController::watchdogExpired(uint32_t nowMs) const {
  if (watchdogMs_ == 0U ||
      (state_ != State::Running && state_ != State::Starting &&
       state_ != State::Enabling)) {
    return false;
  }
  return static_cast<uint32_t>(nowMs - lastKeepAliveMs_) > watchdogMs_;
}
