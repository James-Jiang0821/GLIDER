#pragma once

#include "controllers/pid_controller.hpp"

class PistonController {
public:

    enum class State {
        UNHOMED,
        HOMING,
        RUN,
        HOLD,
        FAULT
    };

    // -----------------------------
    // CONFIG (injected from main)
    // -----------------------------
    struct Config {
        float stroke_min_mm;
        float stroke_max_mm;

        float soft_start_mm;
        float soft_end_mm;
        float soft_zone_max_duty;

        float min_duty;

        float pos_tol_mm;
        float drift_restart_mm;

        float homing_pwm;
        float homing_backoff_duty;
        float homing_timeout_s;
    };

    struct Inputs {
        float pos_est_mm;
        float vel_est_mm_s;

        bool prox_triggered;

        bool hard_fault;
        bool soft_fault;

        bool cmd_enable;
        bool cmd_home;

        float target_percent;
    };

    struct Output {
        bool dir_extend;
        float duty;
        State state;
        float target_mm;
        bool homing_failed;
    };

    void configure(const Config& cfg);
    void configurePID(const PIDController::Config& cfg);

    void reset();

    Output update(const Inputs& in, float dt);

private:

    float percentToMm(float pct) const;
    float applySoftZone(float pos_mm) const;

    Config cfg_{};

    PIDController pid_;

    State state_ = State::UNHOMED;

    float target_mm_ = 0.0f;

    bool homing_backoff_ = false;

    float homing_timer_s_ = 0.0f;
};