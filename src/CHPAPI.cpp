#include "CHPAPI.h"

#include <stdint.h>
#include <string.h>

#define READ_BYTE(v) if (!readByte(v)) { lastError = Error::READ_TIMEOUT; return false; }

using namespace CHPAPI;

constexpr uint16_t BYTE_TIMEOUT_MS = 20;
constexpr uint8_t FRAME_SIZE = 128;

constexpr uint8_t PROT_IDX_IDENT = 0;
constexpr uint8_t PROT_IDX_DEVICE_ID = 1;
constexpr uint8_t PROT_IDX_CMD = 2;
constexpr uint8_t PROT_IDX_LEN = 3;
constexpr uint8_t PROT_IDX_DATA_BEGIN = 4;

constexpr uint8_t DEVICE_ID_CONTROLLER = 0x66;
constexpr uint8_t DEVICE_ID_MAIN_UNIT = 0x77;

class ProtocolMonitor::Impl {
public:
    Impl(Stream& stream) : stream(stream) {
        memset(&contrData, 0, sizeof(ControllerData));
        memset(&mainUnitData, 0, sizeof(MainUnitData));
    }

    bool read() {
        if (!stream.available()) return false;

        lastError = Error::NO_ERROR;
        memset(frame, 0, sizeof(frame));
        bool success = readFrame();

        return success;
    }

    Error getLastError() const {
        return lastError;
    }

    const ControllerData& getControllerData() const {
        return contrData;
    }

    const MainUnitData& getMainUnitData() const {
        return mainUnitData;
    }
private:
    Stream& stream;
    Error lastError = Error::NO_ERROR;
    uint8_t frame[FRAME_SIZE];
    
    ControllerData contrData{};
    MainUnitData mainUnitData{};

    void readControllerFrame() {
        contrData.powerLevel = frame[8];
        contrData.reqState = static_cast<RequestedState>(frame[9]);
    }

    void readMainUnitFrame() {
        mainUnitData.state = static_cast<State>(frame[5]);
        mainUnitData.powerLevel = frame[6];
        mainUnitData.errorState = static_cast<ErrorState>(frame[7]);
        mainUnitData.inputVoltage = frame[11] / 10.0f;
        mainUnitData.heatExchangerTemperature = readU16BE(16) / 10.0f;
        mainUnitData.stateDuration = readU16BE(20);
        mainUnitData.pumpFrequency = frame[23] / 10.0f;
        mainUnitData.fanRPM = readU16BE(28);
    }

    bool readFrame() {
        READ_BYTE(frame[PROT_IDX_IDENT]);

        if (frame[PROT_IDX_IDENT] != 0xAA) {
            return false;
        }

        READ_BYTE(frame[PROT_IDX_DEVICE_ID]);
        READ_BYTE(frame[PROT_IDX_CMD]);
        READ_BYTE(frame[PROT_IDX_LEN]);

        const size_t dataLength = frame[PROT_IDX_LEN];
        const size_t checksumIndex = PROT_IDX_DATA_BEGIN + dataLength;

        if (checksumIndex >= FRAME_SIZE) {
            lastError = Error::INVALID_FRAME_LENGTH;
            return false;
        }

        for (size_t i = 0; i < dataLength; i++) {
            READ_BYTE(frame[PROT_IDX_DATA_BEGIN + i]);
        }

        READ_BYTE(frame[checksumIndex]);

        uint8_t expectedChecksum = 0;
        for (size_t i = PROT_IDX_CMD; i < checksumIndex; i++) {
            expectedChecksum += frame[i];
        }

        const uint8_t receivedChecksum = frame[checksumIndex];
        if (expectedChecksum != receivedChecksum) {
            lastError = Error::INVALID_CHECKSUM;
            return false;
        }

        switch (frame[PROT_IDX_DEVICE_ID]) {
            case DEVICE_ID_CONTROLLER:
                readControllerFrame();
                break;

            case DEVICE_ID_MAIN_UNIT:
                readMainUnitFrame();
                break;

            default:
                lastError = Error::UNKNOWN_DEVICE_ID;
                return false;
        }

        return true;
    }

    bool skipBytes(uint8_t count) {
        for (uint8_t i = 1; i < count; i++) {
            if (!readByte(frame[i])) {
                lastError = Error::READ_TIMEOUT;
                return false;
            }
        }

        return true;
    }
    
    uint8_t calculateChecksum(const uint8_t* frame, size_t frameSize) {
        uint8_t checksum = 0;
        
        for (size_t i = 2; i < frameSize - 1; i++) {
            checksum += frame[i];
        }
        
        return checksum;
    }

    uint16_t readU16BE(size_t index) {
        return ((uint16_t)frame[index] << 8) |
            ((uint16_t)frame[index + 1]);
    }

    bool readByte(uint8_t& value) {
        uint32_t startTime = millis();

        while ((uint32_t)(millis() - startTime) < BYTE_TIMEOUT_MS) {
            if (stream.available()) {
                int received = stream.read();

                if (received >= 0) {
                    value = (uint8_t)received;
                    return true;
                }
            }

#ifdef ARDUINO_ARCH_ESP32
            yield();
#else
            delayMicroseconds(1);
#endif
        }

        return false;
    }
};

ProtocolMonitor::ProtocolMonitor(Stream& stream) {
    impl = new Impl(stream);
}

ProtocolMonitor::~ProtocolMonitor() {
    delete impl;
}

bool ProtocolMonitor::read() {
    return impl->read();
}

Error ProtocolMonitor::getLastError() const {
    return impl->getLastError();
}

const ControllerData& ProtocolMonitor::getControllerData() const {
    return impl->getControllerData();
}

const MainUnitData& ProtocolMonitor::getMainUnitData() const {
    return impl->getMainUnitData();
}
