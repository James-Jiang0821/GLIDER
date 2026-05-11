#pragma once

#include <Arduino.h>

class EncoderDriver {
public:
    struct Config {
        uint8_t pin_a;
        uint8_t pin_b;
        uint8_t pin_z;   // optional (255 if unused)

        int32_t counts_per_rev;
        float   lead_screw_pitch_mm;
    };

    explicit EncoderDriver(const Config& cfg);

    void begin();

    void update();  // call as fast as possible in loop()

    int32_t readCounts() const;
    void writeCounts(int32_t value);

    float readPositionMm() const;

private:
    Config cfg_;

    volatile int32_t count_ = 0;
    volatile uint8_t last_state_ = 0;

    // --- NEW ---
    static EncoderDriver* instance_;
    static void isrHandler();

    inline uint8_t readState_() const;
    float countsToMm_(int32_t counts) const;
};