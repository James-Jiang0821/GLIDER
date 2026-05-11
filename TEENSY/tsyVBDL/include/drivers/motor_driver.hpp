#pragma once
#include <stdint.h>

// We implemented a structured MotorDriver class that safely 
// controls PWM, direction, sleep (enable), fault detection, 
// and current sensing for the Pololu G2 driver. It initializes 
// pins, sets a 25 kHz PWM frequency, enforces safe behavior 
// when disabled, and converts the current sense voltage to 
// amps, giving higher-level controllers a clean and safe 
// interface to command motion without dealing with low-level 
// pin handling.

class MotorDriver {
public:
  enum class Direction : uint8_t {
    EXTEND = 0,
    RETRACT = 1
  };

  struct Config {
    uint8_t pin_pwm;
    uint8_t pin_dir;
    uint8_t pin_slp;
    uint8_t pin_flt;
    uint8_t pin_cs;

    // PWM
    float pwm_freq_hz = 25000.0f;    // 25 kHz default
    uint16_t pwm_max = 255;          // default analogWrite range (may be 255 or 1023 on Teensy)

    // Fault polarity (Pololu FLT is commonly active-low)
    bool flt_active_low = true;

    // Current sense scaling
    // Pololu G2 24v13: approx 0.04 V/A (confirm your exact board)
    float cs_volts_per_amp = 0.04f;

    // ADC reference voltage (Teensy uses 3.3V analog reference by default)
    float adc_ref_volts = 3.3f;

    // ADC resolution bits (Teensy default can be 10, 12, etc.)
    uint8_t adc_bits = 12;
  };

  explicit MotorDriver(const Config& cfg);

  void begin();

  void enable();     // SLP HIGH
  void disable();    // SLP LOW + PWM=0

  void setDirection(Direction dir);
  Direction direction() const { return dir_; }

  // PWM command
  void setPWM(uint16_t pwm);  // 0..pwm_max
  void setDuty(float duty);   // 0..1

  void stop();                // PWM=0 (does not disable)
  bool faultActive() const;   // reads FLT pin

  float readCurrentA() const; // reads CS pin and converts to amps

  uint16_t lastPwm() const { return last_pwm_; }
  bool enabled() const { return enabled_; }

private:
  Config cfg_;
  Direction dir_ = Direction::EXTEND;
  uint16_t last_pwm_ = 0;
  bool enabled_ = false;

  uint16_t adcMax_() const;
};