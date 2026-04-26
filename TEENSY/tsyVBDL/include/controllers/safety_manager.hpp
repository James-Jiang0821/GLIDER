#pragma once

#include <stdint.h>

/*
In your real `main.cpp`, the **SafetyManager sits between sensors/drivers 
and the controllers** as the single source of truth for system health. 
Each control cycle (ideally at your fixed 1 kHz loop), you gather all raw 
inputs—leak sensor, motor fault pin, motor current, estimator position, 
ToF validity, command/BMS timeouts, etc.—and pass them into 
`safety.update(in, dt)`. The returned output provides `hard_fault`, 
`soft_fault`, and fault bitfields; **you do not let controllers decide 
faults anymore**. The piston controller receives `hard_fault` 
(and optionally `soft_fault`) as inputs, and if `hard_fault` is true, 
the main loop immediately forces the motor to a safe state 
(PWM = 0, disable driver) and keeps it there since faults are latched. 
Separately, the same SafetyManager output is used to populate and transmit 
the `STATUS_FAULT (0x050)` CAN message. In short: **drivers → 
SafetyManager → controller + CAN + motor override**, ensuring all safety 
decisions are centralized, deterministic, and consistent.
*/

namespace can {
enum HardFaultA : uint8_t;
enum HardFaultB : uint8_t;
enum SoftFault  : uint8_t;
}

class SafetyManager {
public:
    struct Config {
        float overcurrent_hard_limit_a;
        float overcurrent_confirm_s;

        float stall_current_a;
        float stall_min_duty;
        float stall_pos_eps_mm;
        float stall_confirm_s;

        float bms_temp_confirm_s;

        float stroke_real_min_mm;
        float stroke_real_max_mm;

        float leak_debounce_s;
        float driver_fault_debounce_s;
    };

    struct Inputs {
        bool leak_raw;
        bool driver_fault_raw;

        float motor_current_a;
        bool motor_enabled;
        float motor_duty;

        float pos_est_mm;
        bool homing_failed;

        bool cmd_timeout_small;
        bool cmd_timeout_large;

        bool bms_timeout_small;
        bool bms_timeout_large;

        bool bms_temp_high;
        bool bms_overcurrent;
        bool bms_switch_fault;

        bool bms_low_voltage;
        bool bms_high_voltage;

        bool tof_valid;
        bool tof_out_of_range;
        bool slip_detected;
    };

    struct Output {
        bool hard_fault;
        bool soft_fault;

        uint8_t hard_fault_a;
        uint8_t hard_fault_b;
        uint8_t soft_fault_bits;

        uint8_t first_hard_fault;
    };

    void configure(const Config& cfg);
    void reset();

    Output update(const Inputs& in, float dt);

private:
    void latchFirstHardFault(uint8_t code);

    Config cfg_{};

    bool hard_fault_latched_ = false;

    uint8_t hard_fault_a_ = 0;
    uint8_t hard_fault_b_ = 0;
    uint8_t soft_fault_bits_ = 0;
    uint8_t first_hard_fault_ = 0;

    float overcurrent_timer_s_ = 0.0f;
    float stall_timer_s_ = 0.0f;
    float bms_temp_timer_s_ = 0.0f;
    float leak_timer_s_ = 0.0f;
    float driver_fault_timer_s_ = 0.0f;

    float last_pos_mm_ = 0.0f;
    bool pos_initialized_ = false;
};