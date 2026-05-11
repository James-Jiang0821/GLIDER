#include "controllers/piston_controller.hpp"
#include <algorithm>
#include <cmath>

// -----------------------------
// CONFIG
// -----------------------------
void PistonController::configure(const Config& cfg)
{
    cfg_ = cfg;
}

// -----------------------------
// PID
// -----------------------------
void PistonController::configurePID(const PIDController::Config& cfg)
{
    pid_.configure(cfg);
}

// -----------------------------
// RESET
// -----------------------------
void PistonController::reset()
{
    state_ = State::UNHOMED;
    pid_.reset();
    homing_timer_s_ = 0.0f;
    homing_backoff_ = false;
    target_mm_ = cfg_.stroke_min_mm;
}

// -----------------------------
// UTIL
// -----------------------------
float PistonController::percentToMm(float pct) const
{
    pct = std::clamp(pct, 0.0f, 100.0f);

    return cfg_.stroke_min_mm +
           (pct / 100.0f) *
           (cfg_.stroke_max_mm - cfg_.stroke_min_mm);
}

float PistonController::applySoftZone(float pos_mm) const
{
    if (pos_mm <= cfg_.soft_start_mm) {
        float t = (pos_mm - cfg_.stroke_min_mm) /
                  (cfg_.soft_start_mm - cfg_.stroke_min_mm);

        t = std::clamp(t, 0.0f, 1.0f);

        return cfg_.soft_zone_max_duty +
               t * (1.0f - cfg_.soft_zone_max_duty);
    }

    if (pos_mm >= cfg_.soft_end_mm) {
        float t = (cfg_.stroke_max_mm - pos_mm) /
                  (cfg_.stroke_max_mm - cfg_.soft_end_mm);

        t = std::clamp(t, 0.0f, 1.0f);

        return cfg_.soft_zone_max_duty +
               t * (1.0f - cfg_.soft_zone_max_duty);
    }

    return 1.0f;
}

// -----------------------------
// MAIN UPDATE
// -----------------------------
PistonController::Output
PistonController::update(const Inputs& in, float dt)
{
    Output out{};
    out.homing_failed = false;

    // safe defaults
    out.duty = 0.0f;
    out.dir_extend = true;

    // -----------------------------
    // HARD FAULT (latched)
    // -----------------------------
    if (in.hard_fault) {
        state_ = State::FAULT;
    }

    if (state_ == State::FAULT) {
        out.state = state_;
        out.target_mm = target_mm_;
        return out;
    }

    // -----------------------------
    // DISABLE → UNHOMED
    // -----------------------------
    if (!in.cmd_enable) {
        state_ = State::UNHOMED;

        pid_.reset();
        homing_timer_s_ = 0.0f;
        homing_backoff_ = false;

        target_mm_ = cfg_.stroke_min_mm;

        out.state = state_;
        out.target_mm = target_mm_;
        return out;
    }

    // -----------------------------
    // STATE MACHINE
    // -----------------------------
    switch (state_) {

    case State::UNHOMED:
        if (in.cmd_home) {
            state_ = State::HOMING;
            homing_backoff_ = false;
            homing_timer_s_ = 0.0f;
        }
        break;

    // -----------------------------
    // HOMING
    // -----------------------------
    case State::HOMING:

        homing_timer_s_ += dt;

        if (homing_timer_s_ > cfg_.homing_timeout_s) {
            state_ = State::FAULT;
            out.homing_failed = true;
            break;
        }

        // Phase 1: retract
        if (!homing_backoff_) {

            if (!in.prox_triggered) {
                out.dir_extend = false;
                out.duty = cfg_.homing_pwm;
            } else {
                homing_backoff_ = true;
            }
        }
        // Phase 2: forward
        else {

            if (in.prox_triggered) {
                out.dir_extend = true;
                out.duty = cfg_.homing_backoff_duty;
            } else {
                // homing complete
                pid_.reset();
                target_mm_ = cfg_.stroke_min_mm;

                homing_timer_s_ = 0.0f;
                homing_backoff_ = false;

                state_ = State::HOLD;
            }
        }

        break;

    // -----------------------------
    // RUN
    // -----------------------------
    case State::RUN:
    {
        target_mm_ = percentToMm(in.target_percent);

        float error = target_mm_ - in.pos_est_mm;

        if (std::fabs(error) <= cfg_.pos_tol_mm) {
            state_ = State::HOLD;
            break;
        }

        float effort = pid_.update(
            target_mm_,
            in.pos_est_mm,
            in.vel_est_mm_s,
            dt
        );

        out.dir_extend = effort > 0.0f;

        float duty = std::fabs(effort);

        // soft zone scaling
        duty *= applySoftZone(in.pos_est_mm);

        // min duty AFTER scaling (ensures torque)
        if (duty > 0.0f) {
            duty = cfg_.min_duty +
                   (1.0f - cfg_.min_duty) * duty;
        }

        out.duty = std::clamp(duty, 0.0f, 1.0f);

        break;
    }

    // -----------------------------
    // HOLD (includes READY)
    // -----------------------------
    case State::HOLD:
    {
        float commanded_target_mm = percentToMm(in.target_percent);
        float error = commanded_target_mm - in.pos_est_mm;

        target_mm_ = commanded_target_mm;

        if (std::fabs(error) > cfg_.drift_restart_mm) {
            pid_.reset();
            state_ = State::RUN;
        }

        break;
    }

    case State::FAULT:
        break;
    }

    out.state = state_;
    out.target_mm = target_mm_;

    return out;
}