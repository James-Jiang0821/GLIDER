#pragma once

#include <Arduino.h>
#include <QuadEncoder.h>

class EncoderAlt {
public:
    struct Config {
        uint8_t channel;   // ✅ REQUIRED (1–4)
        uint8_t pin_a;
        uint8_t pin_b;
        int32_t counts_per_rev;
        float   lead_screw_pitch_mm;
    };

    explicit EncoderAlt(const Config& cfg);

    void begin();

    int32_t readCounts();
    void writeCounts(int32_t value);

    float readPositionMm();

private:
    Config cfg_;

    QuadEncoder encoder_;

    float countsToMm_(int32_t counts) const;
};