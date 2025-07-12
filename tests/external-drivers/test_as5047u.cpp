#include "../src/AS5047U.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <unordered_map>

// -----------------------------------------------------------------------------
// Simulated AS5047U device that mimics SPI transactions according to the
// datasheet timing for 16-, 24- and 32-bit frame formats.
// -----------------------------------------------------------------------------

class SimulatedDevice {
  public:
    std::unordered_map<uint16_t, uint16_t> regs; // simple register map

    // state for current SPI transaction
    struct State {
        bool waiting = false; // expecting second frame
        bool read = false;    // true if read command
        uint16_t addr = 0;
    } s16, s24, s32;

    int progCountdown = 0; // simulate OTP programming delay

    SimulatedDevice() {
        // initialise a few registers with non-zero values for testing
        regs[AS5047U_REG::ANGLECOM::ADDRESS] = 0x1111;
        regs[AS5047U_REG::ANGLEUNC::ADDRESS] = 0x2222;
        regs[AS5047U_REG::VEL::ADDRESS] = 0x0100;
        regs[AS5047U_REG::AGC::ADDRESS] = 0x0055;
        regs[AS5047U_REG::MAG::ADDRESS] = 0x0AAA;
        regs[AS5047U_REG::DIA::ADDRESS] = 0x1234;
    }

    static uint8_t crc8(uint16_t dat) {
        uint8_t crc = 0xC4;
        for (int i = 0; i < 16; ++i) {
            bool bit = ((dat >> (15 - i)) & 1) ^ ((crc >> 7) & 1);
            crc = (crc << 1) ^ (bit ? 0x1D : 0);
        }
        return crc ^ 0xFF;
    }

    void handle_prog() {
        if (progCountdown > 0) {
            --progCountdown;
            if (progCountdown == 0) {
                regs[AS5047U_REG::PROG::ADDRESS] = 0x0001; // PROGEN=1 after done
            }
        }
    }

    void transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
        handle_prog();
        if (len == 2) {
            spi16(tx, rx);
        } else if (len == 3) {
            spi24(tx, rx);
        } else if (len == 4) {
            spi32(tx, rx);
        }
    }

  private:
    void spi16(const uint8_t *tx, uint8_t *rx) {
        if (!s16.waiting) {
            uint16_t cmd = (tx[0] << 8) | tx[1];
            s16.read = cmd & 0x4000;
            s16.addr = cmd & 0x3FFF;
            s16.waiting = true;
            if (rx) {
                rx[0] = 0;
                rx[1] = 0;
            }
        } else {
            if (s16.read) {
                uint16_t val = regs[s16.addr] & 0x3FFF;
                if (rx) {
                    rx[0] = val >> 8;
                    rx[1] = val & 0xFF;
                }
            } else {
                uint16_t val = ((tx[0] << 8) | tx[1]) & 0x3FFF;
                regs[s16.addr] = val;
                if (s16.addr == AS5047U_REG::PROG::ADDRESS && (val & 0x0008)) {
                    progCountdown = 2; // start programming
                }
                if (rx) {
                    rx[0] = 0;
                    rx[1] = 0;
                }
            }
            s16.waiting = false;
        }
    }

    void spi24(const uint8_t *tx, uint8_t *rx) {
        if (!s24.waiting) {
            uint16_t dat = ((tx[0] & 0x3F) << 8) | tx[1];
            s24.read = tx[0] & 0x40;
            s24.addr = dat;
            s24.waiting = true;
            if (rx) {
                rx[0] = 0;
                rx[1] = 0;
                rx[2] = 0;
            }
        } else {
            if (s24.read) {
                uint16_t val = regs[s24.addr] & 0x3FFF;
                uint8_t c = crc8(val);
                if (rx) {
                    rx[0] = val >> 8;
                    rx[1] = val & 0xFF;
                    rx[2] = c;
                }
            } else {
                uint16_t val = ((tx[0] << 8) | tx[1]) & 0x3FFF;
                regs[s24.addr] = val;
                if (s24.addr == AS5047U_REG::PROG::ADDRESS && (val & 0x0008)) {
                    progCountdown = 2;
                }
                if (rx) {
                    rx[0] = 0;
                    rx[1] = 0;
                    rx[2] = 0;
                }
            }
            s24.waiting = false;
        }
    }

    void spi32(const uint8_t *tx, uint8_t *rx) {
        if (!s32.waiting) {
            uint16_t dat = ((tx[1] & 0x3F) << 8) | tx[2];
            s32.read = tx[1] & 0x40;
            s32.addr = dat;
            s32.waiting = true;
            if (rx) {
                rx[0] = 0;
                rx[1] = 0;
                rx[2] = 0;
                rx[3] = 0;
            }
        } else {
            if (s32.read) {
                uint16_t val = regs[s32.addr] & 0x3FFF;
                uint8_t c = crc8(val);
                if (rx) {
                    rx[0] = tx[0];
                    rx[1] = val >> 8;
                    rx[2] = val & 0xFF;
                    rx[3] = c;
                }
            } else {
                uint16_t val = ((tx[1] << 8) | tx[2]) & 0x3FFF;
                regs[s32.addr] = val;
                if (s32.addr == AS5047U_REG::PROG::ADDRESS && (val & 0x0008)) {
                    progCountdown = 2;
                }
                if (rx) {
                    rx[0] = 0;
                    rx[1] = 0;
                    rx[2] = 0;
                    rx[3] = 0;
                }
            }
            s32.waiting = false;
        }
    }
};

// -----------------------------------------------------------------------------
// SPI bus that forwards transfers to the simulated device
// -----------------------------------------------------------------------------
class MockBus : public AS5047U::spiBus {
  public:
    SimulatedDevice dev;
    void transfer(const uint8_t *tx, uint8_t *rx, std::size_t len) override {
        dev.transfer(tx, rx, len);
    }
};

// -----------------------------------------------------------------------------
// Unit tests exercising the AS5047U API
// -----------------------------------------------------------------------------

int main() {
    MockBus bus;
    AS5047U enc(bus, FrameFormat::SPI_16);

    // Basic getters
    assert(enc.getAngle() == 0x1111);
    assert(enc.getRawAngle() == 0x2222);
    assert(enc.getVelocity() == 0x0100);
    assert(enc.getAGC() == 0x55);
    assert(enc.getMagnitude() == 0x0AAA);
    assert(enc.getErrorFlags() == 0);

    // Velocity conversions
    double dps = enc.getVelocityDegPerSec();
    double rps = enc.getVelocityRadPerSec();
    double rpm = enc.getVelocityRPM();
    assert(std::fabs(dps - (0x0100 * AS5047U::Velocity::DEG_PER_LSB)) < 1e-3);
    assert(std::fabs(rps - (0x0100 * AS5047U::Velocity::RAD_PER_LSB)) < 1e-3);
    assert(std::fabs(rpm - (0x0100 * AS5047U::Velocity::RPM_PER_LSB)) < 1e-3);

    // Configuration setters and getters
    assert(enc.setZeroPosition(0x3333));
    assert(bus.dev.regs[AS5047U_REG::ZPOSM::ADDRESS] == (0x3333 >> 6));
    assert(bus.dev.regs[AS5047U_REG::ZPOSL::ADDRESS] == (0x3333 & 0x3F));

    assert(enc.setDirection(false));
    auto s2 = bus.dev.regs[AS5047U_REG::SETTINGS2::ADDRESS];
    assert(((s2 >> 2) & 1) == 1);

    assert(enc.setABIResolution(12));
    auto s3 = bus.dev.regs[AS5047U_REG::SETTINGS3::ADDRESS];
    assert((s3 & 0xE0) >> 5 == (12 - 10));

    assert(enc.setUVWPolePairs(5));
    s3 = bus.dev.regs[AS5047U_REG::SETTINGS3::ADDRESS];
    assert((s3 & 0x07) == (5 - 1));

    assert(enc.setIndexPulseLength(1));
    s2 = bus.dev.regs[AS5047U_REG::SETTINGS2::ADDRESS];
    assert((s2 & 0x01) == 1);

    assert(enc.configureInterface(true, false, true));
    auto dis = bus.dev.regs[AS5047U_REG::DISABLE::ADDRESS];
    s2 = bus.dev.regs[AS5047U_REG::SETTINGS2::ADDRESS];
    assert(((dis >> 1) & 1) == 0 && ((dis >> 0) & 1) == 1);
    assert((s2 & 0x80) >> 7 == 1);

    assert(enc.setDynamicAngleCompensation(true));
    s2 = bus.dev.regs[AS5047U_REG::SETTINGS2::ADDRESS];
    assert(((s2 >> 4) & 1) == 0);

    assert(enc.setAdaptiveFilter(true));
    dis = bus.dev.regs[AS5047U_REG::DISABLE::ADDRESS];
    assert(((dis >> 6) & 1) == 0);

    assert(enc.setFilterParameters(2, 3));
    auto s1 = bus.dev.regs[AS5047U_REG::SETTINGS1::ADDRESS];
    // expect K_min=2 (bits3..5) and K_max=3 (bits0..2)
    assert((s1 & 0x7) == 3 && ((s1 >> 3) & 0x7) == 2);

    assert(enc.set150CTemperatureMode(true));
    s2 = bus.dev.regs[AS5047U_REG::SETTINGS2::ADDRESS];
    assert(((s2 >> 1) & 1) == 1);

    enc.setPad(0xAA); // just call

    assert(enc.setHysteresis(AS5047U_REG::SETTINGS3::Hysteresis::LSB_2));
    s3 = bus.dev.regs[AS5047U_REG::SETTINGS3::ADDRESS];
    assert(((s3 >> 3) & 0x3) == 1);
    assert(enc.getHysteresis() == AS5047U_REG::SETTINGS3::Hysteresis::LSB_2);

    assert(enc.setAngleOutputSource(AS5047U_REG::SETTINGS2::AngleOutputSource::UseANGLEUNC));
    assert(enc.getAngleOutputSource() == AS5047U_REG::SETTINGS2::AngleOutputSource::UseANGLEUNC);

    auto dia = enc.getDiagnostics();
    assert(dia.value == 0x1234);

    // Templated read/write
    auto readS2 = enc.readReg<AS5047U_REG::SETTINGS2>();
    assert(readS2.value == bus.dev.regs[AS5047U_REG::SETTINGS2::ADDRESS]);
    AS5047U_REG::SETTINGS2 wr = readS2;
    wr.bits.PWMon = 0;
    assert(enc.writeReg(wr));
    assert(bus.dev.regs[AS5047U_REG::SETTINGS2::ADDRESS] == (readS2.value & ~0x80));

    // Program OTP sequence (simulation always succeeds)
    assert(enc.programOTP());

    std::cout << "All tests passed\n";
    return 0;
}
