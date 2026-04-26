#pragma once
#include <Arduino.h>

// =====================
// VBD Teensy Pin Map
// =====================
//
// Notes:
// Notes:
// - CAN2 uses pins 0 (RX) and 1 (TX) on Teensy 4.x
// - I2C1 uses pins 16 (SDA1) and 17 (SCL1) => Wire1
// - BMS UART on pins 7/8 (typical Teensy Serial2 mapping)
// - Proximity is pull-up (likely active-low when triggered)

// =====================
// Node identity
// =====================
#ifndef NODE_ID
#define NODE_ID 1 // fallback if not defined
#endif

// =====================
// CAN (Transceiver)
// =====================
// You said: 0 -> R, 1 -> D on CAN transceiver.
// Keep names generic: Teensy TX/RX to transceiver DI/RO depends on your transceiver.
// We'll just label what you told me.
// CAN2 uses TX=1, RX=0
constexpr uint8_t PIN_CAN_R = 0;   // connected to transceiver R (RO)
constexpr uint8_t PIN_CAN_D = 1;   // connected to transceiver D (DI)

// =====================
// Encoder (Maxon TTL -> 3.3V)
// =====================
constexpr uint8_t PIN_ENC_A = 2;   // CHA
constexpr uint8_t PIN_ENC_B = 3;   // CHB
constexpr uint8_t PIN_ENC_Z = 4;   // CHZ (index)

// =====================
// Proximity sensor (pull-up, active-low when triggered per your earlier note)
// =====================
constexpr uint8_t PIN_PROX  = 5;

// =====================
// BMS UART (TinyBMS)
// =====================
constexpr uint8_t PIN_BMS_TX = 7;  // Teensy TX -> BMS RX (check direction in wiring)
constexpr uint8_t PIN_BMS_RX = 8;  // Teensy RX <- BMS TX
constexpr uint32_t BMS_BAUD  = 115200;

// =====================
// VL6180X ToF (I2C)
// =====================
constexpr uint8_t PIN_TOF_SCL = 16;
constexpr uint8_t PIN_TOF_SDA = 17;

// =====================
// Leak sensor
// =====================
constexpr uint8_t PIN_LEAK = 18;

// =====================
// Motor driver (Pololu G2 24v13)
// =====================
constexpr uint8_t PIN_MOTOR_CS  = 19;  // ADC current sense
constexpr uint8_t PIN_MOTOR_FLT = 20;  // fault
constexpr uint8_t PIN_MOTOR_SLP = 21;  // sleep/enable
constexpr uint8_t PIN_MOTOR_PWM = 22;  // PWM
constexpr uint8_t PIN_MOTOR_DIR = 23;  // DIR