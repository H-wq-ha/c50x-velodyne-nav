#pragma once

#include <Arduino.h>

class MotorController {
 public:
  enum class Direction : uint8_t { Forward, Reverse };
  enum class State : uint8_t {
    Disabled,
    Idle,
    Enabling,
    Starting,
    Running,
    Fault,
    EStop
  };

  void begin();
  void update();

  bool enable();
  void disable();
  void stop();
  void emergencyStop();
  bool clearSafetyLatch();

  bool run(Direction direction, uint32_t speedPps);
  bool move(Direction direction, uint32_t pulses, uint32_t speedPps);
  bool setSpeed(uint32_t speedPps);
  bool setWatchdog(uint32_t timeoutMs);
  void keepAlive();

  State state() const { return state_; }
  Direction direction() const { return direction_; }
  bool driveEnabled() const { return driveEnabled_; }
  bool alarmAsserted() const;
  bool inPosition() const;
  uint32_t speedPps() const { return speedPps_; }
  uint32_t sentPulses() const { return sentPulses_; }
  uint32_t remainingPulses() const { return continuous_ ? 0U : remainingPulses_; }
  bool continuous() const { return continuous_; }
  uint32_t watchdogMs() const { return watchdogMs_; }
  const char* lastStopReason() const { return lastStopReason_; }

  static const char* stateName(State state);
  static const char* directionName(Direction direction);

 private:
  bool requestMotion(Direction direction, uint32_t pulses, uint32_t speedPps,
                     bool continuous);
  void setDriveEnabled(bool enabled);
  void applyDirection();
  void setPulseActive(bool active);
  void enterFault(const char* reason);
  void stopMotion(const char* reason);
  bool motionAllowed() const;
  bool watchdogExpired(uint32_t nowMs) const;

  State state_ = State::Disabled;
  Direction direction_ = Direction::Forward;
  bool driveEnabled_ = false;
  bool pulseActive_ = false;
  bool continuous_ = false;
  bool stopAfterPulse_ = false;
  uint32_t speedPps_ = 0;
  uint32_t remainingPulses_ = 0;
  uint32_t sentPulses_ = 0;
  uint32_t activeTimeUs_ = 0;
  uint32_t idleTimeUs_ = 0;
  uint32_t nextEdgeUs_ = 0;
  uint32_t startAtUs_ = 0;
  uint32_t enableReadyAtUs_ = 0;
  uint32_t watchdogMs_ = 0;
  uint32_t lastKeepAliveMs_ = 0;
  const char* lastStopReason_ = "POWER_ON";
};
