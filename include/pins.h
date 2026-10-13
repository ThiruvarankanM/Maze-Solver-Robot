#pragma once

#include <stdint.h>

// Encoders, motor drivers and ultrasonics match the wiring tables in docs/.
// IR array and user interface are still placeholders.

// Encoders (all four channels on interrupt pins)
constexpr uint8_t ENC_LEFT_A = 2;
constexpr uint8_t ENC_LEFT_B = 3;
constexpr uint8_t ENC_RIGHT_A = 18;
constexpr uint8_t ENC_RIGHT_B = 19;

// Motor drivers (HW-039 / BTS7960, one per motor; R_EN and L_EN bridged)
constexpr uint8_t MOTOR_LEFT_RPWM = 5;  // forward
constexpr uint8_t MOTOR_LEFT_LPWM = 6;  // reverse
constexpr uint8_t MOTOR_LEFT_EN = 24;
constexpr uint8_t MOTOR_RIGHT_RPWM = 7; // forward
constexpr uint8_t MOTOR_RIGHT_LPWM = 8; // reverse
constexpr uint8_t MOTOR_RIGHT_EN = 25;

// Ultrasonic sensors
constexpr uint8_t US_FRONT_TRIG = 30;
constexpr uint8_t US_FRONT_ECHO = 31;
constexpr uint8_t US_LEFT_TRIG = 32;
constexpr uint8_t US_LEFT_ECHO = 33;
constexpr uint8_t US_RIGHT_TRIG = 34;
constexpr uint8_t US_RIGHT_ECHO = 35;

// IR array (left to right)
constexpr uint8_t IR_PINS[8] = {54, 55, 56, 57, 58, 59, 60, 61}; // A0-A7

// MPU-6050 uses I2C: SDA 20, SCL 21

// User interface
// TODO: 30 and 31 are now the front ultrasonic - reassign once wired.
constexpr uint8_t MODE_SWITCH = 30;  // explore / fast run
constexpr uint8_t START_BUTTON = 31;
constexpr uint8_t STATUS_LED = 13;
