#include "drivers/encoder.hpp"

EncoderDriver* EncoderDriver::instance_ = nullptr;

EncoderDriver::EncoderDriver(const Config& cfg)
: cfg_(cfg) {}

void EncoderDriver::begin() {
    pinMode(cfg_.pin_a, INPUT_PULLUP);
    pinMode(cfg_.pin_b, INPUT_PULLUP);

    if (cfg_.pin_z != 255) {
        pinMode(cfg_.pin_z, INPUT_PULLUP);
    }

    instance_ = this;

    last_state_ = readState_();
    count_ = 0;

    attachInterrupt(digitalPinToInterrupt(cfg_.pin_a), isrHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(cfg_.pin_b), isrHandler, CHANGE);
}

inline uint8_t EncoderDriver::readState_() const {
    return (digitalRead(cfg_.pin_a) << 1) |
            digitalRead(cfg_.pin_b);
}

void EncoderDriver::isrHandler() {
    if (!instance_) return;

    EncoderDriver* self = instance_;

    uint8_t s =
        (digitalRead(self->cfg_.pin_a) << 1) |
         digitalRead(self->cfg_.pin_b);

    if (s == self->last_state_) return;

    // --- YOUR ORIGINAL LOGIC (UNCHANGED) ---
    if (
        (self->last_state_ == 3 && s == 2) ||
        (self->last_state_ == 2 && s == 0) ||
        (self->last_state_ == 0 && s == 1) ||
        (self->last_state_ == 1 && s == 3)
    ) {
        self->count_--;
    }
    else if (
        (self->last_state_ == 3 && s == 1) ||
        (self->last_state_ == 1 && s == 0) ||
        (self->last_state_ == 0 && s == 2) ||
        (self->last_state_ == 2 && s == 3)
    ) {
        self->count_++;
    }

    self->last_state_ = s;
}

int32_t EncoderDriver::readCounts() const {
    noInterrupts();
    int32_t c = count_;
    interrupts();
    return c;
}

void EncoderDriver::writeCounts(int32_t value) {
    noInterrupts();
    count_ = value;
    interrupts();
}


float EncoderDriver::countsToMm_(int32_t counts) const {
    if (cfg_.counts_per_rev == 0) return 0.0f;

    const float revs = (float)counts / (float)cfg_.counts_per_rev;
    return revs * cfg_.lead_screw_pitch_mm;
}

float EncoderDriver::readPositionMm() const {
    return countsToMm_(readCounts());
}