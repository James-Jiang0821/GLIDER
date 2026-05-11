#include "controllers/pid_controller.hpp"
#include <algorithm>

void PIDController::configure(const Config& cfg)
{
    cfg_ = cfg;
}

void PIDController::reset()
{
    integral_ = 0.0f;
}

float PIDController::update(float setpoint_mm,
                            float pos_mm,
                            float vel_mm_s,
                            float dt)
{
    float error = setpoint_mm - pos_mm;

    // ----- Integral -----
    integral_ += error * dt;

    integral_ = std::clamp(integral_,
                           -cfg_.i_limit,
                           cfg_.i_limit);

    // ----- Terms -----
    float p = cfg_.kp * error;
    float i = cfg_.ki * integral_;
    float d = -cfg_.kd * vel_mm_s;

    float effort = p + i + d;

    // ----- Output clamp -----
    effort = std::clamp(effort,
                        -cfg_.out_limit,
                        cfg_.out_limit);

    return effort;
}