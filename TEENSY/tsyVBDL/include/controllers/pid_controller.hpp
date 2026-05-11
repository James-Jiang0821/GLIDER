#pragma once

class PIDController {
public:

    struct Config {
        float kp;
        float ki;
        float kd;

        float i_limit;     // integral clamp
        float out_limit;   // effort clamp (1.0)
    };

    PIDController() = default;

    void configure(const Config& cfg);

    void reset();

    float update(float setpoint_mm,
                 float pos_mm,
                 float vel_mm_s,
                 float dt);

private:

    Config cfg_{};

    float integral_ = 0.0f;
};