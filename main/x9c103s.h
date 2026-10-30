#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "esp_err.h"

/**
 * Driver for the X9C103S digitally controlled potentiometer (10 kOhm, 100 taps, 3-wire interface).
 *
 * Wiring (module pin -> ESP32):
 *   CS   -> any GPIO (active low)
 *   INC  -> any GPIO (wiper moves on HIGH->LOW transition)
 *   U/D  -> any GPIO (HIGH = up, LOW = down)
 *   VCC  -> 5 V (the X9C103S is a 5 V part; see the note about level shifting in main.cpp)
 *   VSS  -> GND
 *   VH / VL / VW -> the potentiometer terminals
 */
class X9C103S {
public:
    static constexpr int     kTaps       = 100;      // positions 0..99
    static constexpr float   kTotalOhms  = 10000.0f; // nominal end-to-end resistance
    static constexpr float   kWiperOhms  = 40.0f;    // typical wiper resistance

    X9C103S(gpio_num_t cs, gpio_num_t inc, gpio_num_t ud);

    /** Configure the GPIOs and leave the bus idle (CS=1, INC=1). */
    esp_err_t init();

    /**
     * Drive the wiper to 0 (VL side) by stepping down kTaps times.
     * Use this once at boot to synchronize the software position counter,
     * because the chip restores the last stored wiper position at power-up
     * and offers no way to read it back.
     */
    void home(bool store = false);

    /**
     * Calibration: drive the wiper to the maximum position (99, VH side) by stepping up
     * kTaps + 5 times. Since the end stop is a known mechanical limit, the software
     * counter is then exactly in sync with the chip (position = 99).
     * Call this at every power-up.
     */
    void calibrateToMax(bool store = false);

    /** Move the wiper to an absolute position 0..99. Optionally save it to non-volatile memory. */
    void setPosition(uint8_t position, bool store = false);

    /** Move the wiper relative to the current one (clamped to 0..99). */
    void step(int delta, bool store = false);

    /** Save the current wiper position to non-volatile memory. */
    void store();

    /** Set a resistance between VW and VL (approximate). Returns the position used. */
    uint8_t setResistanceOhms(float ohms, bool store = false);

    /** Software-tracked wiper position (0..99). */
    uint8_t position() const { return position_; }

    /** Approximate resistance between VW and VL for the current position. */
    float resistanceOhms() const;

private:
    void pulses(bool up, int count, bool store);

    gpio_num_t cs_, inc_, ud_;
    uint8_t    position_ = 0;
    bool       initialized_ = false;
};
