#pragma once
#include "motor_regulator.h"

extern Regulator left_regulator;
extern Regulator right_regulator;

// ============================================================
//  Раздельные константы для левого и правого колеса.
//  Значения получены экспериментально: L = 442, R = 374.
//  Если такие же уже есть в motor_regulator.h — убери дубли там.
// ============================================================


enum Commands : uint8_t {
  SET_VELOCITY       = 0x10,
  GET_ODOM_DATA      = 0x11,
  HANDSHAKE_RESPONSE = 0x14,
  TURN_ROBOT         = 0x12,
  MOVE_DIST          = 0x13
};

// -------- Переменные для движения --------
long dist_rStart_ticks = 0;
long dist_lStart_ticks = 0;
long turn_rStart_ticks = 0;
long turn_lStart_ticks = 0;

// Раздельные цели по тикам для каждого колеса
long moveTicksLeft  = 0;
long moveTicksRight = 0;
long turnTicksLeft  = 0;
long turnTicksRight = 0;

bool move_flag = false;
bool turn_flag = false;

template<typename T>
void serial_read(T& value) {
  while (Serial.available() < static_cast<int>(sizeof(T))) {}
  Serial.readBytes(reinterpret_cast<uint8_t*>(&value), sizeof(T));
}

// ============================================================
//  Установка скорости.
//  linear  — м/с
//  angular — рад/с
//  Внутри каждая сторона пересчитывается через СВОЮ константу.
// ============================================================
void set_velocity(float linear, float angular) {
  float left_speed  = (linear - angular * WHEEL_BASE / 2.0f) * LEFT_TICKS_PER_METER;
  float right_speed = (linear + angular * WHEEL_BASE / 2.0f) * RIGHT_TICKS_PER_METER;

  left_regulator.set_speed(left_speed);
  right_regulator.set_speed(right_speed);
}

// ============================================================
//  Движение на заданную дистанцию.
//  dist_mm — мм (знак задаёт направление)
//  speed   — мм/с
// ============================================================
void move_distance(int32_t dist_mm, int32_t speed) {
  float dist_meters = dist_mm / 1000.0f;

  // Сколько тиков должно пройти каждое колесо
  float left_ticks_f  = (dist_meters / WHEEL_CIRCUMFERENCE) * LEFT_TICKS_PER_REV;
  float right_ticks_f = (dist_meters / WHEEL_CIRCUMFERENCE) * RIGHT_TICKS_PER_REV;

  dist_lStart_ticks = left_regulator.encoder.ticks;
  dist_rStart_ticks = right_regulator.encoder.ticks;

  moveTicksLeft  = (long)abs(left_ticks_f);
  moveTicksRight = (long)abs(right_ticks_f);
  move_flag = true;

  float speed_mps = speed / 1000.0f;
  float sign      = (dist_mm >= 0) ? 1.0f : -1.0f;

  left_regulator.set_speed(sign * speed_mps * LEFT_TICKS_PER_METER);
  right_regulator.set_speed(sign * speed_mps * RIGHT_TICKS_PER_METER);
}

// ============================================================
//  Поворот на угол.
//  angle_deg — градусы (знак = направление)
//  speed     — мм/с
// ============================================================
void turn_robot(int32_t angle_deg, int32_t speed) {
  float angle_rad    = angle_deg * PI / 180.0f;
  float arc_distance = (WHEEL_BASE / 2.0f) * angle_rad;

  float left_ticks_f  = (arc_distance / WHEEL_CIRCUMFERENCE) * LEFT_TICKS_PER_REV;
  float right_ticks_f = (arc_distance / WHEEL_CIRCUMFERENCE) * RIGHT_TICKS_PER_REV;

  turn_lStart_ticks = left_regulator.encoder.ticks;
  turn_rStart_ticks = right_regulator.encoder.ticks;

  turnTicksLeft  = (long)abs(left_ticks_f);
  turnTicksRight = (long)abs(right_ticks_f);
  turn_flag = true;

  float speed_mps = speed / 1000.0f;

  if (angle_deg > 0) {
    left_regulator.set_speed( speed_mps * LEFT_TICKS_PER_METER);
    right_regulator.set_speed(-speed_mps * RIGHT_TICKS_PER_METER);
  } else {
    left_regulator.set_speed(-speed_mps * LEFT_TICKS_PER_METER);
    right_regulator.set_speed( speed_mps * RIGHT_TICKS_PER_METER);
  }
}

// ============================================================
//  Проверка завершения движения — по КАЖДОМУ колесу отдельно.
// ============================================================
void check_move_complete() {
  if (!move_flag) return;

  long left_traveled  = abs(left_regulator.encoder.ticks  - dist_lStart_ticks);
  long right_traveled = abs(right_regulator.encoder.ticks - dist_rStart_ticks);

  if (left_traveled >= moveTicksLeft && right_traveled >= moveTicksRight) {
    left_regulator.set_speed(0);
    right_regulator.set_speed(0);
    move_flag = false;
  }
}

void check_turn_complete() {
  if (!turn_flag) return;

  long left_traveled  = abs(left_regulator.encoder.ticks  - turn_lStart_ticks);
  long right_traveled = abs(right_regulator.encoder.ticks - turn_rStart_ticks);

  if (left_traveled >= turnTicksLeft && right_traveled >= turnTicksRight) {
    left_regulator.set_speed(0);
    right_regulator.set_speed(0);
    turn_flag = false;
  }
}

// ============================================================
//  Отправка одометрии в ROS.
// ============================================================
struct EncoderData {
  int16_t left_ticks;
  int16_t right_ticks;
  float   left_speed;
  float   right_speed;
};

int16_t prev_left_ticks  = 0;
int16_t prev_right_ticks = 0;

void send_encoder_data() {
  left_regulator.encoder.calc_delta();
  right_regulator.encoder.calc_delta();

  int16_t left_delta  = left_regulator.encoder.ticks  - prev_left_ticks;
  int16_t right_delta = right_regulator.encoder.ticks - prev_right_ticks;

  prev_left_ticks  = left_regulator.encoder.ticks;
  prev_right_ticks = right_regulator.encoder.ticks;

  float left_speed_mps  = left_regulator.encoder.speed  * LEFT_METERS_PER_TICK;
  float right_speed_mps = right_regulator.encoder.speed * RIGHT_METERS_PER_TICK;

  EncoderData data{
    left_delta,
    right_delta,
    left_speed_mps,
    right_speed_mps
  };
  Serial.write((uint8_t*)&data, sizeof(data));
}

void handshake_response() {
  Serial.write("ARDUINO_OK");
  Serial.flush();
}

// ============================================================
//  Разбор команд по Serial.
// ============================================================
void command_spin() {
  if (Serial.available() > 0) {
    uint8_t cmd = Serial.read();
    switch (cmd) {
      case SET_VELOCITY: {
        float linear, angular;
        serial_read(linear);
        serial_read(angular);
        set_velocity(linear, angular);
        break;
      }
      case GET_ODOM_DATA:
        send_encoder_data();
        break;
      case HANDSHAKE_RESPONSE:
        handshake_response();
        break;
      case TURN_ROBOT: {
        int8_t angle, speed;
        serial_read(angle);
        serial_read(speed);
        turn_robot(angle, speed);
        break;
      }
      case MOVE_DIST: {
        int32_t dist, speed;
        serial_read(dist);
        serial_read(speed);
        move_distance(dist, speed);
        break;
      }
    }
  }
}