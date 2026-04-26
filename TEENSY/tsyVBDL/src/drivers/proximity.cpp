#include "drivers/proximity.hpp"
#include <Arduino.h>

ProximitySensor::ProximitySensor(uint8_t pin, bool active_low)
: pin_(pin), active_low_(active_low) {}

void ProximitySensor::begin(bool use_pullup) {
    pinMode(pin_, use_pullup ? INPUT_PULLUP : INPUT);
}

bool ProximitySensor::raw() const {
    return digitalRead(pin_) == HIGH;
}

bool ProximitySensor::isTriggered() const {
    bool r = raw();
    return active_low_ ? !r : r;
}