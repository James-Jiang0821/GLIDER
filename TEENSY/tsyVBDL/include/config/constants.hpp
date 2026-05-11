#pragma once

/*
------------------------------------------------------------
VBD TEENSY SYSTEM CONSTANTS
------------------------------------------------------------

All global system configuration values live here.

Rules:
- No hardware pins here (pin_map.hpp handles that)
- No driver-specific constants
- Only system behaviour parameters

Units are explicitly stated to avoid ambiguity.
------------------------------------------------------------
*/

#include <stdint.h>

namespace config
{

/* ----------------------------------------------------------
   NODE CONFIGURATION
---------------------------------------------------------- */

constexpr uint8_t NODE_ID_LEFT  = 1;
constexpr uint8_t NODE_ID_RIGHT = 2;


/* ----------------------------------------------------------
   CONTROL LOOP
---------------------------------------------------------- */

constexpr uint32_t CONTROL_HZ = 1000;        // Control loop frequency
constexpr float CONTROL_DT_S  = 1.0f / CONTROL_HZ;

constexpr uint32_t PWM_FREQ_HZ = 25000;      // Motor PWM frequency


/* ----------------------------------------------------------
   STROKE GEOMETRY
---------------------------------------------------------- */

constexpr float STROKE_REAL_MIN_MM = 8.0f;
constexpr float STROKE_REAL_MAX_MM = 150.0f;

// usable control range
constexpr float STROKE_HARD_MIN_MM = 10.0f;
constexpr float STROKE_HARD_MAX_MM = 140.0f;

// soft zones
constexpr float STROKE_SOFT_START_MM = 20.0f;
constexpr float STROKE_SOFT_END_MM   = 130.0f;

// stroke span
constexpr float STROKE_SPAN_MM = STROKE_HARD_MAX_MM - STROKE_HARD_MIN_MM;

/* ----------------------------------------------------------
   POSITION CONTROL
---------------------------------------------------------- */

constexpr float POS_TOL_MM = 0.1f;   // enter hold mode
constexpr float DRIFT_RESTART_MM = 2.0f;   // restart PID

constexpr float HOMING_PWM = 0.60f;          // slow homing
constexpr float HOMING_BACKOFF_DUTY = 0.10f;  //even slower correction
constexpr float SOFT_ZONE_MAX_DUTY = 0.80f;   // slower in soft zone
constexpr float MIN_DUTY = 0.30f; // ~10%


/* ----------------------------------------------------------
   PID GAINS
---------------------------------------------------------- */
constexpr float PID_KP = 0.10f;
constexpr float PID_KI = 0.0f;
constexpr float PID_KD = 0.003f;

constexpr float PID_I_LIMIT = 5.0f;
constexpr float PID_OUT_LIMIT = 1.0f;

// constexpr float TOF_OFFSET_MM = 0.0f;

/* ----------------------------------------------------------
   ESTIMATOR PARAMETERS
---------------------------------------------------------- */

constexpr float TOF_FUSION_GAIN = 0.02f;

// EMA alpha for tof_only position filter (lower = smoother but more lag).
// 0.3 at ~50 Hz ToF ≈ ~40 ms time constant — filters sensor noise at long range.
constexpr float TOF_FILTER_ALPHA = 0.2f;

constexpr float SLIP_DETECT_MM = 3.0f;
constexpr float SLIP_RECOVERY_GAIN = 0.2f;


/* ----------------------------------------------------------
   TOF SENSOR VALIDATION
---------------------------------------------------------- */

constexpr uint16_t TOF_MIN_VALID_MM = 8;

// Raw ToF backstop: only cross-checks against hard limit when filtered position
// is already within this many mm of the limit, preventing mid-range noise spikes
// from causing spurious motor cutoffs.
constexpr float TOF_BACKSTOP_APPROACH_MM = 5.0f;


/* ----------------------------------------------------------
   ENCODER + MECHANICS
---------------------------------------------------------- */

constexpr float LEAD_SCREW_MM_PER_REV = 4.0f;

constexpr uint16_t ENCODER_CPR = 1024;
constexpr uint8_t ENCODER_QUAD = 4;
constexpr uint16_t GEAR_RATIO = 156;

constexpr uint32_t ENCODER_COUNTS_PER_REV = ENCODER_CPR * ENCODER_QUAD;

constexpr uint32_t COUNTS_PER_SCREW_REV = ENCODER_COUNTS_PER_REV * GEAR_RATIO;

constexpr float MM_PER_COUNT =
    LEAD_SCREW_MM_PER_REV / COUNTS_PER_SCREW_REV;


/* ----------------------------------------------------------
   CURRENT SENSE
---------------------------------------------------------- */

// Motor driver current sense gain
constexpr float CURRENT_SENSE_V_PER_A = 0.04f;


/* ----------------------------------------------------------
   CAN POSITION SCALING
---------------------------------------------------------- */

// Estimated position sent in 0.1mm units
constexpr float CAN_POS_SCALE = 10.0f;


/* ----------------------------------------------------------
   SAFETY PLACEHOLDERS
---------------------------------------------------------- */

constexpr float OVERCURRENT_HARD_LIMIT_A = 6.0f;
constexpr float OVERCURRENT_CONFIRM_S = 0.50f;
constexpr float STALL_CURRENT_A = 3.0f;
constexpr float STALL_MIN_DUTY = 0.25f;
constexpr float STALL_POS_EPS_MM = 0.10f;
constexpr float STALL_CONFIRM_S = 2.0f;
constexpr float BMS_TEMP_CONFIRM_S = 2.0f;
constexpr float HOMING_TIMEOUT_S = 180.0f;

// debounce
constexpr float LEAK_DEBOUNCE_S = 0.05f;
constexpr float DRIVER_FAULT_DEBOUNCE_S = 0.02f;

// command timeout
constexpr uint32_t CMD_TIMEOUT_SMALL_MS = 300;
constexpr uint32_t CMD_TIMEOUT_LARGE_MS = 120000;


// BMS timeout
constexpr uint32_t BMS_TIMEOUT_SMALL_MS = 5000;
constexpr uint32_t BMS_TIMEOUT_LARGE_MS = 120000;

}