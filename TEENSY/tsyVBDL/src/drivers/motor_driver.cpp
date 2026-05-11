#include "drivers/motor_driver.hpp"
#include <Arduino.h>

MotorDriver::MotorDriver(const Config& cfg) : cfg_(cfg) {}

uint16_t MotorDriver::adcMax_() const {
  // e.g. 12-bit -> 4095
  return (1u << cfg_.adc_bits) - 1u;
}

void MotorDriver::begin() {
  pinMode(cfg_.pin_dir, OUTPUT);
  pinMode(cfg_.pin_pwm, OUTPUT);
  pinMode(cfg_.pin_slp, OUTPUT);
  pinMode(cfg_.pin_flt, INPUT_PULLUP); // safe default; FLT often open-drain
  pinMode(cfg_.pin_cs, INPUT);

  // ADC resolution
  analogReadResolution(cfg_.adc_bits);

  // PWM setup
  analogWriteFrequency(cfg_.pin_pwm, cfg_.pwm_freq_hz);
  analogWrite(cfg_.pin_pwm, 0);

  // Default safe state
  digitalWrite(cfg_.pin_dir, LOW);
  digitalWrite(cfg_.pin_slp, LOW);
  enabled_ = false;
  last_pwm_ = 0;
  dir_ = Direction::EXTEND;
}

void MotorDriver::enable() {
  digitalWrite(cfg_.pin_slp, HIGH);
  enabled_ = true;
}

void MotorDriver::disable() {
  // Always stop first
  analogWrite(cfg_.pin_pwm, 0);
  last_pwm_ = 0;
  digitalWrite(cfg_.pin_slp, LOW);
  enabled_ = false;
}

void MotorDriver::setDirection(Direction dir) {
  dir_ = dir;
  digitalWrite(cfg_.pin_dir, (dir_ == Direction::EXTEND) ? HIGH : LOW);
}

void MotorDriver::setPWM(uint16_t pwm) {
  if (pwm > cfg_.pwm_max) pwm = cfg_.pwm_max;
  last_pwm_ = pwm;

  // If disabled, don't drive
  if (!enabled_) {
    analogWrite(cfg_.pin_pwm, 0);
    return;
  }

  analogWrite(cfg_.pin_pwm, pwm);
}

void MotorDriver::setDuty(float duty) {
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 1.0f) duty = 1.0f;
  const uint16_t pwm = static_cast<uint16_t>(duty * static_cast<float>(cfg_.pwm_max));
  setPWM(pwm);
}

void MotorDriver::stop() {
  analogWrite(cfg_.pin_pwm, 0);
  last_pwm_ = 0;
}

bool MotorDriver::faultActive() const {
  const bool raw_high = (digitalRead(cfg_.pin_flt) == HIGH);
  // active-low => fault when pin is LOW
  return cfg_.flt_active_low ? !raw_high : raw_high;
}

float MotorDriver::readCurrentA() const {
  const uint16_t raw = analogRead(cfg_.pin_cs);
  const float volts = (static_cast<float>(raw) / static_cast<float>(adcMax_())) * cfg_.adc_ref_volts;

  // Current sense outputs volts proportional to motor current
  // Some boards output 0V at 0A. If yours has offset, we’ll add later.
  const float amps = volts / cfg_.cs_volts_per_amp;
  return amps;
}