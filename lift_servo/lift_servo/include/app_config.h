#pragma once

#include <Arduino.h>

namespace AppConfig {

// STM32F103C8 <-> 3.3 V/5 V level-converter connections.
constexpr uint32_t kPulsePin = PA0;       // A0 -> B0 -> PUL+
constexpr uint32_t kDirectionPin = PA1;   // A1 -> B1 -> DIR+
constexpr uint32_t kEnablePin = PA2;      // A2 -> B2 -> ENA+
constexpr uint32_t kPendPin = PA3;        // A3 <- B3 <- PEND feedback
constexpr uint32_t kAlarmPin = PA4;       // A4 <- B4 <- ALM feedback

// USART1 is independent of PA2/PA3. UART signals must be crossed:
// PA9 (TX) -> module RX, PA10 (RX) <- module TX.
constexpr uint32_t kSerialBaud = 115200;

// Electrical polarity. Change these values to match the drive manual and its
// parameter settings before connecting the motor/load.
constexpr uint8_t kPulseActiveLevel = HIGH;
constexpr uint8_t kForwardDirectionLevel = HIGH;

// Many step-servo drives call ENA an "offline/free" input: asserting ENA
// removes motor current. This configuration therefore asserts ENA to DISABLE.
// Set false only if the exact drive manual says asserted ENA enables torque.
constexpr bool kEnaSignalDisablesDrive = true;
constexpr uint8_t kEnaAssertedLevel = HIGH;

// PEND/ALM are normally open-collector optocoupler outputs. With an external
// pull-up on the 5 V side and a suitable level converter, conduction reads LOW.
constexpr uint8_t kPendAssertedLevel = LOW;
constexpr uint8_t kAlarmAssertedLevel = LOW;

constexpr uint32_t kDefaultSpeedPps = 1000;
constexpr uint32_t kMinSpeedPps = 1;
constexpr uint32_t kMaxSpeedPps = 20000;
constexpr uint32_t kMinPulseWidthUs = 3;
constexpr uint32_t kDirectionSetupUs = 10;
constexpr uint32_t kEnableLeadUs = 200000;

// 0 disables the communication watchdog. It can be enabled at runtime with
// WATCHDOG <milliseconds>. On expiry, pulses stop but holding torque remains.
constexpr uint32_t kDefaultWatchdogMs = 0;
constexpr uint32_t kMaxWatchdogMs = 60000;

}  // namespace AppConfig
