#include <drivers/enc_alt.hpp>

EncoderAlt::EncoderAlt(const Config& cfg)
: cfg_(cfg),
  encoder_(cfg.channel, cfg.pin_a, cfg.pin_b)   // ✅ matches example
{}

void EncoderAlt::begin() {
    encoder_.setInitConfig();   // ✅ correct order
    encoder_.init();
    encoder_.write(0);          // reset position
}

int32_t EncoderAlt::readCounts() {
    return encoder_.read();     // ✅ correct API
}

void EncoderAlt::writeCounts(int32_t value) {
    encoder_.write(value);      // ✅ correct API
}

float EncoderAlt::countsToMm_(int32_t counts) const {
    if (cfg_.counts_per_rev == 0) return 0.0f;

    float revs = (float)counts / (float)cfg_.counts_per_rev;
    return revs * cfg_.lead_screw_pitch_mm;
}

float EncoderAlt::readPositionMm() {
    return countsToMm_(readCounts());
}