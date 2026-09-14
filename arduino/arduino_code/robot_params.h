#pragma once
#include <math.h>

constexpr float DT = 0.01f;

// ---------------- Железо робота ----------------
// ВНИМАНИЕ: PIN_ECHO не должен совпадать с пинами энкодеров!
// В arduino_code.ino: left_enc(3, 12), right_enc(2, 13).
// Поэтому ультразвук сажаем на свободные пины (например 8 и 9).
constexpr byte PIN_TRIG = 8;
constexpr byte PIN_ECHO = 9;

// ---------------- Колесная база ----------------
constexpr float WHEEL_DIAMETER = 0.067f;   // [m]
constexpr float WHEEL_BASE     = 0.18f;    // [m]

// ---------------- Ограничения ----------------
constexpr float MAX_LIN_SPEED = 0.4863f;   // [m/s]
constexpr float MAX_ANG_SPEED = 5.4f;      // [rad/s]
constexpr float MAX_LIN_ACCEL = 5.0f;      // [m/s²]
constexpr float MAX_ANG_ACCEL = 5.0f;      // [rad/s²]

// ---------------- Расчетные константы ----------------
constexpr float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;

// Экспериментально измеренные тики на оборот для каждого колеса
constexpr float LEFT_TICKS_PER_REV  = 442.0f;
constexpr float RIGHT_TICKS_PER_REV = 374.0f;

constexpr float LEFT_TICKS_PER_METER  = LEFT_TICKS_PER_REV  / WHEEL_CIRCUMFERENCE;
constexpr float RIGHT_TICKS_PER_METER = RIGHT_TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

constexpr float LEFT_METERS_PER_TICK  = WHEEL_CIRCUMFERENCE / LEFT_TICKS_PER_REV;
constexpr float RIGHT_METERS_PER_TICK = WHEEL_CIRCUMFERENCE / RIGHT_TICKS_PER_REV;

// Для лимита скорости берём худший (больший) вариант, чтобы ни одно колесо не превысило.
constexpr float MAX_TICKS_PER_METER =
    (LEFT_TICKS_PER_METER > RIGHT_TICKS_PER_METER) ? LEFT_TICKS_PER_METER : RIGHT_TICKS_PER_METER;

constexpr int MAX_DELTA_TICKS = static_cast<int>(MAX_LIN_SPEED * MAX_TICKS_PER_METER);