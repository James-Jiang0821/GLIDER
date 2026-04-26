#pragma once

#include <stdint.h>

class Estimator {
public:
    struct Config {
        float mm_per_count;
        float tof_gain;
        float recovery_gain;
        float slip_detect_mm;
        float tof_min_valid_mm;
        // float tof_offset_mm;
    };

    struct Output {
        float pos_est_mm;
        float vel_est_mm_s;
        uint16_t tof_mm;
        bool tof_valid;
        bool slip_detected;
    };

    Estimator() = default;
    explicit Estimator(const Config& cfg) : cfg_(cfg) {}

    void configure(const Config& cfg);

    void reset();
    void setPositionMm(float pos_mm);

    void syncEncoder(int32_t encoder_count);

    Output update(int32_t encoder_count,
                  bool tof_valid_hw,
                  uint16_t tof_mm,
                  float dt);

    float positionMm() const;
    float velocityMmPerSec() const;

private:
    Config cfg_{};

    float pos_est_mm_ = 0.0f;
    float vel_est_mm_s_ = 0.0f;

    int32_t last_encoder_count_ = 0;
    bool encoder_initialized_ = false;

    uint16_t last_tof_mm_ = 0;
    bool last_tof_valid_ = false;
    bool slip_detected_ = false;
};