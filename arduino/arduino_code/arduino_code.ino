#include "motor_regulator.h"
#include "ros2_communication.hpp"

// Форвард-декларации
class Regulator;
extern Regulator left_regulator;
extern Regulator right_regulator;

Motor right_motor(4, 5, true);
Motor left_motor(7, 6, true);

// Энкодеры
Encoder left_enc(3, 12, []{ left_regulator.encoder.encoder_int(); }, true);
Encoder right_enc(2, 13, []{ right_regulator.encoder.encoder_int(); }, true);

// PID-регуляторы
PID left_pid(4.0, 0.2, 0.04, 100);
PID right_pid(4.0, 0.2, 0.04, 100);

// Регуляторы
Regulator left_regulator(left_motor, left_enc, left_pid);
Regulator right_regulator(right_motor, right_enc, right_pid);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  set_velocity(0.0f, 0.0f);
}

void loop() {
  static uint32_t t = millis();

  command_spin();

  if (millis() - t >= 10) {
    t = millis();
    left_regulator.update();
    right_regulator.update();
  }

  check_move_complete();
  check_turn_complete();
}