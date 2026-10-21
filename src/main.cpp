#include <Arduino.h>
#include "CHPAPI.h"

CHPAPI::ProtocolMonitor* mon;

void setup() {
    Serial.begin(112500);
    
    mon = new CHPAPI::ProtocolMonitor(Serial);
}

void loop() {
    if (!mon->read()) {
        if (mon->getLastError() != CHPAPI::Error::NO_ERROR) {
            Serial.println("ERROR!");
        }
    } else {
         
    }
}
