#pragma once

#include <stdint.h>

// Placeholder values - update once the robot is wired.

// Encoders (A channels must be interrupt pins: 2, 3, 18, 19, 20, 21)
constexpr uint8_t ENC_LEFT_A = 2;
constexpr uint8_t ENC_LEFT_B = 4;
constexpr uint8_t ENC_RIGHT_A = 3;
constexpr uint8_t ENC_RIGHT_B = 5;

// Motor driver
constexpr uint8_t MOTOR_LEFT_PWM = 6;
constexpr uint8_t MOTOR_LEFT_IN1 = 26;
constexpr uint8_t MOTOR_LEFT_IN2 = 27;
constexpr uint8_t MOTOR_RIGHT_PWM = 7;
constexpr uint8_t MOTOR_RIGHT_IN1 = 28;
constexpr uint8_t MOTOR_RIGHT_IN2 = 29;

// Ultrasonic sensors
constexpr uint8_t US_LEFT_TRIG = 36;
constexpr uint8_t US_LEFT_ECHO = 37;
constexpr uint8_t US_FRONT_TRIG = 38;
constexpr uint8_t US_FRONT_ECHO = 39;
constexpr uint8_t US_RIGHT_TRIG = 40;
constexpr uint8_t US_RIGHT_ECHO = 41;

// IR array (left to right)
constexpr uint8_t IR_PINS[8] = {54, 55, 56, 57, 58, 59, 60, 61}; // A0-A7

// MPU-6050 uses I2C: SDA 20, SCL 21

// User interface
constexpr uint8_t MODE_SWITCH = 30;  // explore / fast run
constexpr uint8_t START_BUTTON = 31;
constexpr uint8_t STATUS_LED = 13;
