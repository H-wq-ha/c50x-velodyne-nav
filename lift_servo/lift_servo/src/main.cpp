#include <Arduino.h>

#include "app_config.h"
#include "motor_controller.h"
#include "serial_protocol.h"

MotorController motor;
SerialProtocol protocol(Serial1, motor);

void setup() {
  motor.begin();
  Serial1.begin(AppConfig::kSerialBaud, SERIAL_8N1);
  protocol.begin();
}

void loop() {
  // Both calls are non-blocking so pulse timing, ALM and UART remain serviced.
  motor.update();
  protocol.update();
}
