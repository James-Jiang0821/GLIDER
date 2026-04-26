#include <Arduino.h>
#include "config/pin_map.hpp"
#include "utils/logger.hpp"
#include "drivers/tof_sensor.hpp"

static ToFSensor tof(PIN_TOF_SDA, PIN_TOF_SCL);

void setup() {
    logger::begin(115200);
    LOGI(F("ToF Test, improved driver"));

    Wire1.setSDA(PIN_TOF_SDA);
    Wire1.setSCL(PIN_TOF_SCL);

    if (!tof.begin(Wire1)) {
        LOGE(F("ToF begin() failed."));
    } else {
        LOGI(F("ToF init OK."));
    }
}

void loop() {
    auto r = tof.read();

    if (r.fresh && r.valid) {
        LOGI("ToF valid new sample");
        Serial.print(" dist(mm)=");
        Serial.print(r.mm);
        Serial.print(" st=");
        Serial.println(r.status);
    } else if (!r.valid) {
        LOGW("ToF invalid, returning last valid");
        Serial.print(" last(mm)=");
        Serial.print(r.mm);
        Serial.print(" st=");
        Serial.println(r.status);
    }

    delay(500); // 2 Hz
}