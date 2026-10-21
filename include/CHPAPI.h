#pragma once
#include <cstdint>
#ifndef _CHPAPI_H
#define _CHPAPI_H

#include <Arduino.h>

// Protocol credits: https://github.com/zatakon/vevor_heater_control

namespace CHPAPI {

    enum class Error {
        NO_ERROR,
        READ_TIMEOUT,
        INVALID_FRAME_LENGTH,
        INVALID_CHECKSUM,
        UNKNOWN_DEVICE_ID
    };

    enum class RequestedState : uint8_t {
        OFF = 0x02,
        STOP = 0x05,
        START = 0x06,
        RUNNING = 0x08
    };

    enum class State : uint8_t {
        OFF = 0x00,
        GLOW_PLUG_PRE_HEAT = 0x01,
        IGNITED = 0x02,
        STABLE_COMBUSTION = 0x03,
        STOPPING = 0x04
    };

    enum class ErrorState : uint8_t {
        NO_ERROR = 0x00,
        STARTUP_FAIL = 0x01,
        CHECK_FUEL_LEVEL = 0x02,
        SUPPLY_VOLTAGE_OVERRUN = 0x03,
        FUEL_PUMP_FAILURE = 0x06,
        FAN_FAILURE = 0x07,
        CHECK_IGNITION = 0x08,
        OVERHEAT = 0x09
    };

    struct ControllerData {
        RequestedState reqState;
        uint8_t powerLevel; // 1-10
    };

    struct MainUnitData {
        State state;
        uint8_t powerLevel; // 1-10
        ErrorState errorState;
        float inputVoltage;
        float heatExchangerTemperature;
        uint16_t pumpFrequency;
        uint16_t fanRPM;
        uint16_t stateDuration;
    };

    class ProtocolMonitor {
    public:
        ProtocolMonitor(Stream& stream);
        ~ProtocolMonitor();

        bool read();
        Error getLastError() const;

        const ControllerData& getControllerData() const;
        const MainUnitData& getMainUnitData() const;
    private:
        class Impl;
        Impl* impl;
    };

}

#endif // _CHPAPI_H