#pragma once
#include <stdint.h>

// We implemented a ProximitySensor driver that wraps the homing 
// switch as a clean digital input with configurable polarity 
// (active-low in your case). The driver initializes the pin with 
// an internal pull-up and provides both a raw read and a logical 
// isTriggered() function that abstracts the electrical behavior. 
// This keeps the hardware details isolated while allowing higher-
// level motion logic to reliably use the proximity sensor for homing 
// without worrying about signal polarity or pin handling.

class ProximitySensor {
public:
    // active_low = true → LOW means triggered (your case)
    explicit ProximitySensor(uint8_t pin, bool active_low = true);

    void begin(bool use_pullup = true);

    bool isTriggered() const;   // logical triggered state
    bool raw() const;           // raw pin state

private:
    uint8_t pin_;
    bool active_low_;
};