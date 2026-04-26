#include "controllers/estimator.hpp"
#include <cmath>

void Estimator::configure(const Config& cfg)
{
    cfg_ = cfg;
}

void Estimator::reset()
{
    pos_est_mm_ = 0.0f;
    vel_est_mm_s_ = 0.0f;
    last_encoder_count_ = 0;
    encoder_initialized_ = false;
    last_tof_mm_ = 0;
    last_tof_valid_ = false;
    slip_detected_ = false;
}

void Estimator::setPositionMm(float pos_mm)
{
    pos_est_mm_ = pos_mm;
    vel_est_mm_s_ = 0.0f;
    slip_detected_ = false;
}

Estimator::Output Estimator::update(int32_t encoder_count,
                                    bool tof_valid_hw,
                                    uint16_t tof_mm,
                                    float dt)
{
    // ---- Encoder integration ----
    if (!encoder_initialized_) {
        last_encoder_count_ = encoder_count;
        encoder_initialized_ = true;
    }

    int32_t delta_counts = encoder_count - last_encoder_count_;
    last_encoder_count_ = encoder_count;

    float delta_mm = static_cast<float>(delta_counts) * cfg_.mm_per_count;

    pos_est_mm_ += delta_mm;
    vel_est_mm_s_ = (dt > 0.0f) ? (delta_mm / dt) : 0.0f;

    // ---- ToF validation ----
    bool tof_valid = tof_valid_hw && (tof_mm >= cfg_.tof_min_valid_mm);

    last_tof_valid_ = tof_valid;
    if (tof_valid) {
        last_tof_mm_ = tof_mm;

        float tof_pos_mm = static_cast<float>(tof_mm);
        //float tof_pos_mm = static_cast<float>(tof_mm) + cfg_.tof_offset_mm;
        float error_mm = tof_pos_mm - pos_est_mm_;

        slip_detected_ = (std::fabs(error_mm) > cfg_.slip_detect_mm);

        float gain = slip_detected_ ? cfg_.recovery_gain
                                    : cfg_.tof_gain;

        pos_est_mm_ += gain * error_mm;
    } else {
        slip_detected_ = false;
    }

    return Output{
        pos_est_mm_,
        vel_est_mm_s_,
        last_tof_mm_,
        tof_valid,
        slip_detected_
    };
}

float Estimator::positionMm() const
{
    return pos_est_mm_;
}

float Estimator::velocityMmPerSec() const
{
    return vel_est_mm_s_;
}

void Estimator::syncEncoder(int32_t encoder_count) {
    last_encoder_count_ = encoder_count;
    encoder_initialized_ = true;
}
