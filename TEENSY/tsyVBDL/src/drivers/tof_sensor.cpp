#include "drivers/tof_sensor.hpp"
#include <Adafruit_VL6180X.h>

static Adafruit_VL6180X g_vl;

bool ToFSensor::begin(TwoWire& wire) {
  wire.setSDA(sda_pin_);
  wire.setSCL(scl_pin_);

  wire.begin();
  wire.setClock(400000);

  if (!g_vl.begin(&wire)) {
    inited_ = false;
    continuous_started_ = false;
    last_status_ = 0xFF;
    no_ready_count_ = 0;
    return false;
  }

  // Continuous mode allows non-blocking reads from this wrapper.
  g_vl.startRangeContinuous(20);

  inited_ = true;
  continuous_started_ = true;
  last_status_ = 0;
  no_ready_count_ = 0;
  return true;
}

ToFSensor::Reading ToFSensor::read() {
  Reading out{};
  if (!inited_) {
    out.fresh = false;
    out.valid = false;
    out.status = 0xFF;
    out.mm = last_valid_mm_;
    return out;
  }

  if (!continuous_started_) {
    g_vl.startRangeContinuous(20);
    continuous_started_ = true;
  }

  // Small bounded wait so we can catch samples that are about to become ready
  // without introducing unbounded blocking behavior.
  uint32_t start_ms = millis();
  bool ready = g_vl.isRangeComplete();
  while (!ready && (millis() - start_ms) < 3) {
    ready = g_vl.isRangeComplete();
  }

  if (!ready) {
    no_ready_count_++;

    // Recover if "not ready" persists for too long.
    if (no_ready_count_ >= 100) {
      g_vl.startRangeContinuous(20);
      no_ready_count_ = 0;
    }

    // Read a real status code so we don't get stuck outputting 0xFF.
    const uint8_t status = g_vl.readRangeStatus();
    last_status_ = status;

    out.valid  = (last_valid_mm_ >= min_valid_mm_);
    out.fresh  = false;
    out.status = status;
    out.mm     = last_valid_mm_;
    return out;
  }
  no_ready_count_ = 0;

  // Follow the library's async flow: read result, then status.
  const uint8_t range  = g_vl.readRangeResult();
  const uint8_t status = g_vl.readRangeStatus();
  last_status_ = status;
  const uint16_t mm = static_cast<uint16_t>(range);

  // Accept only if status OK AND above your minimum reliable floor.
  if (status == 0 && mm >= min_valid_mm_) {
    const float corrected = static_cast<float>(mm) + offset_mm_;
    const uint16_t corrected_mm =
      (corrected < 0.0f) ? 0 : static_cast<uint16_t>(corrected);

    out.valid = true;
    out.fresh = true;
    out.mm = corrected_mm;
    out.status = status;

    last_valid_mm_ = corrected_mm;
    return out;
  }

  out.valid  = false;
  out.fresh  = true;
  out.status = status;
  out.mm     = last_valid_mm_;
  return out;
}
