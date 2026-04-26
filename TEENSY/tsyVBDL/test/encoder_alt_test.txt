#include <Arduino.h>
#include <drivers/enc_alt.hpp>
#include "config/pin_map.hpp"
#include "config/constants.hpp"
#include "drivers/motor_driver.hpp"

// --- Encoder (uses config like your main system) ---
EncoderAlt encoder({
  .channel = 2,  // ⚠️ still must be set (Teensy hardware requirement)

  .pin_a = PIN_ENC_A,
  .pin_b = PIN_ENC_B,

  .counts_per_rev = config::COUNTS_PER_SCREW_REV,
  .lead_screw_pitch_mm = config::LEAD_SCREW_MM_PER_REV
});

// --- Motor ---
MotorDriver motor({
  .pin_pwm = PIN_MOTOR_PWM,
  .pin_dir = PIN_MOTOR_DIR,
  .pin_slp = PIN_MOTOR_SLP,
  .pin_flt = PIN_MOTOR_FLT,
  .pin_cs  = PIN_MOTOR_CS,

  .pwm_freq_hz = config::PWM_FREQ_HZ,
  .pwm_max = 255,

  .flt_active_low = true,

  .cs_volts_per_amp = config::CURRENT_SENSE_V_PER_A,
  .adc_ref_volts = 3.3f,
  .adc_bits = 12
});

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== MOTOR + ENCODER TEST (CONFIG-BASED) ===");

  encoder.begin();
  encoder.writeCounts(0);

  motor.begin();
  motor.enable();

  motor.setDirection(MotorDriver::Direction::EXTEND);

  // Spin motor slowly
  motor.setPWM(10);

  Serial.println("Motor spinning at PWM=10");
}

void loop() {
  static uint32_t lastPrint = 0;

  if (millis() - lastPrint > 200) {
    lastPrint = millis();

    int32_t counts = encoder.readCounts();
    float pos = encoder.readPositionMm();

    Serial.print("Counts: ");
    Serial.print(counts);

    Serial.print(" | Pos(mm): ");
    Serial.print(pos, 4);

    Serial.println();
  }
}