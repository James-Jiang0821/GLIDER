#include "controllers/safety_manager.hpp"
#include "protocols/can_messages.hpp"
#include <cmath>

void SafetyManager::configure(const Config& cfg)
{
    cfg_ = cfg;
}

void SafetyManager::reset()
{
    hard_fault_latched_ = false;

    hard_fault_a_ = 0;
    hard_fault_b_ = 0;
    soft_fault_bits_ = 0;

    first_hard_fault_ = 0;

    overcurrent_timer_s_ = 0.0f;
    stall_timer_s_ = 0.0f;
    bms_temp_timer_s_ = 0.0f;
    leak_timer_s_ = 0.0f;
    driver_fault_timer_s_ = 0.0f;

    pos_initialized_ = false;
}

void SafetyManager::latchFirstHardFault(uint8_t code)
{
    if (!hard_fault_latched_ && first_hard_fault_ == 0) {
        first_hard_fault_ = code;
    }
}

SafetyManager::Output SafetyManager::update(const Inputs& in, float dt)
{
    Output out{};

    if (dt <= 0.0f) {
        out.hard_fault = hard_fault_latched_;
        out.soft_fault = false;
        out.hard_fault_a = hard_fault_a_;
        out.hard_fault_b = hard_fault_b_;
        out.soft_fault_bits = soft_fault_bits_;
        out.first_hard_fault = first_hard_fault_;
        return out;
    }

    // --------------------------------------------------
    // Position init (for stall detection)
    // --------------------------------------------------
    if (!pos_initialized_) {
        last_pos_mm_ = in.pos_est_mm;
        pos_initialized_ = true;
    }

    const float delta_pos = std::fabs(in.pos_est_mm - last_pos_mm_);
    last_pos_mm_ = in.pos_est_mm;

    // --------------------------------------------------
    // Debounce digital faults
    // --------------------------------------------------
    bool leak = false;
    if (in.leak_raw) {
        leak_timer_s_ += dt;
        if (leak_timer_s_ >= cfg_.leak_debounce_s) {
            leak = true;
        }
    } else {
        leak_timer_s_ = 0.0f;
    }

    bool driver_fault = false;
    if (in.driver_fault_raw) {
        driver_fault_timer_s_ += dt;
        if (driver_fault_timer_s_ >= cfg_.driver_fault_debounce_s) {
            driver_fault = true;
        }
    } else {
        driver_fault_timer_s_ = 0.0f;
    }

    // --------------------------------------------------
    // Soft faults (recomputed every cycle)
    // --------------------------------------------------
    uint8_t soft = 0;

    if (in.cmd_timeout_small) soft |= can::FAULT_CMD_TO;
    if (!in.tof_valid)        soft |= can::FAULT_TOF_INV;
    if (in.tof_out_of_range)  soft |= can::FAULT_TOF_OOR;
    if (in.slip_detected)     soft |= can::FAULT_SLIP;
    if (in.bms_timeout_small) soft |= can::FAULT_BMS_TO;
    if (in.bms_low_voltage)   soft |= can::FAULT_BMS_LOW;
    if (in.bms_high_voltage)  soft |= can::FAULT_BMS_HIGH;

    soft_fault_bits_ = soft;

    // --------------------------------------------------
    // Hard fault detection (non-latched stage)
    // --------------------------------------------------
    uint8_t hardA = 0;
    uint8_t hardB = 0;

    // 1. Leak
    if (leak) {
        hardA |= can::FAULT_LEAK;
        latchFirstHardFault(1);
    }

    // 2. Driver fault
    if (driver_fault) {
        hardA |= can::FAULT_DRIVER;
        latchFirstHardFault(2);
    }

    // 3. Overcurrent
    if (in.motor_current_a > cfg_.overcurrent_hard_limit_a) {
        overcurrent_timer_s_ += dt;
        if (overcurrent_timer_s_ >= cfg_.overcurrent_confirm_s) {
            hardA |= can::FAULT_OVERCURRENT;
            latchFirstHardFault(3);
        }
    } else {
        overcurrent_timer_s_ = 0.0f;
    }

    // 4. Stall
    bool stall_condition =
        in.motor_enabled &&
        (in.motor_duty >= cfg_.stall_min_duty) &&
        (in.motor_current_a >= cfg_.stall_current_a) &&
        (delta_pos <= cfg_.stall_pos_eps_mm);

    if (stall_condition) {
        stall_timer_s_ += dt;
        if (stall_timer_s_ >= cfg_.stall_confirm_s) {
            hardA |= can::FAULT_STALL;
            latchFirstHardFault(4);
        }
    } else {
        stall_timer_s_ = 0.0f;
    }

    // 5. Command timeout (large)
    if (in.cmd_timeout_large) {
        hardA |= can::FAULT_CMD_TO_L;
        latchFirstHardFault(5);
    }

    // 6. Homing failed
    if (in.homing_failed) {
        hardA |= can::FAULT_HOMING_FAILED; // your renamed meaning
        latchFirstHardFault(6);
    }

    // 7. Position out of bounds (only checked once homed/calibrated)
    if (in.pos_valid &&
        (in.pos_est_mm < cfg_.stroke_real_min_mm ||
         in.pos_est_mm > cfg_.stroke_real_max_mm)) {

        hardA |= can::FAULT_POS_LIMIT;
        latchFirstHardFault(7);
    }

    // 8. BMS temp high
    if (in.bms_temp_high) {
        bms_temp_timer_s_ += dt;
        if (bms_temp_timer_s_ >= cfg_.bms_temp_confirm_s) {
            hardA |= can::FAULT_BMS_TEMP;
            latchFirstHardFault(8);
        }
    } else {
        bms_temp_timer_s_ = 0.0f;
    }

    // --- B group ---
    if (in.bms_timeout_large) {
        hardB |= can::FAULT_BMS_TO_L;
        latchFirstHardFault(9);
    }

    if (in.bms_overcurrent) {
        hardB |= can::FAULT_BMS_OC;
        latchFirstHardFault(10);
    }

    if (in.bms_switch_fault) {
        hardB |= can::FAULT_BMS_SWITCH;
        latchFirstHardFault(11);
    }

    // --------------------------------------------------
    // Latch hard faults
    // --------------------------------------------------
    if (hardA != 0 || hardB != 0) {
        hard_fault_latched_ = true;
    }

    if (hard_fault_latched_) {
        hard_fault_a_ |= hardA;
        hard_fault_b_ |= hardB;
    }

    // --------------------------------------------------
    // Output
    // --------------------------------------------------
    out.hard_fault = hard_fault_latched_;
    out.soft_fault = (soft_fault_bits_ != 0);

    out.hard_fault_a = hard_fault_a_;
    out.hard_fault_b = hard_fault_b_;
    out.soft_fault_bits = soft_fault_bits_;
    out.first_hard_fault = first_hard_fault_;

    return out;
}