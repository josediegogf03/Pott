#include "x9c103s.h"

#include <algorithm>
#include <cstdlib>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "X9C103S";

// Timing from the datasheet's A.C. operating characteristics (rounded up).
static constexpr uint32_t kDiUs  = 3;    // tDI:  U/D to INC setup        >= 2.9 us
static constexpr uint32_t kCiUs  = 1;    // tCI:  CS to INC setup         >= 100 ns
static constexpr uint32_t kIlUs  = 2;    // tIL:  INC LOW period          >= 1 us (tCYC >= 4 us)
static constexpr uint32_t kIhUs  = 2;    // tIH:  INC HIGH period         >= 1 us
static constexpr uint32_t kIwUs  = 500;  // tIW:  INC to Vw change        <= 500 us
static constexpr uint32_t kIcUs  = 1;    // tIC:  INC inactive to CS inactive >= 1 us
static constexpr uint32_t kCphMs = 20;   // tCPH: CS deselect time        >= 20 ms

X9C103S::X9C103S(gpio_num_t cs, gpio_num_t inc, gpio_num_t ud, float total_ohms, bool reversed,
                 int taps)
    : cs_(cs), inc_(inc), ud_(ud), total_ohms_(total_ohms), reversed_(reversed), taps_(taps) {}

esp_err_t X9C103S::init()
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << cs_) | (1ULL << inc_) | (1ULL << ud_);
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }

    // Standby: CS HIGH (deselected), INC HIGH.
    gpio_set_level(cs_, 1);
    gpio_set_level(inc_, 1);
    gpio_set_level(ud_, 0);

    initialized_ = true;
    ESP_LOGI(TAG, "Initialized (CS=%d INC=%d U/D=%d)", cs_, inc_, ud_);
    return ESP_OK;
}

void X9C103S::pulses(bool up, int count, bool store)
{
    if (!initialized_ || count <= 0) {
        return;
    }

    // Mode selection table:
    //   CS L, INC falling, U/D H -> wiper up      CS rising, INC H -> store wiper position
    //   CS L, INC falling, U/D L -> wiper down    CS rising, INC L -> no store, back to standby
    gpio_set_level(ud_, (up != reversed_) ? 1 : 0);
    esp_rom_delay_us(kDiUs);

    gpio_set_level(cs_, 0);
    esp_rom_delay_us(kCiUs);

    // One INC falling edge per step. INC is left LOW after the last one.
    for (int i = 0; i < count; i++) {
        if (i > 0) {
            gpio_set_level(inc_, 1);
            esp_rom_delay_us(kIhUs);
        }
        gpio_set_level(inc_, 0);
        esp_rom_delay_us(kIlUs);
    }
    esp_rom_delay_us(kIwUs);

    if (store) {
        gpio_set_level(inc_, 1);
        esp_rom_delay_us(kIcUs);
        gpio_set_level(cs_, 1);          // CS rising, INC HIGH: store
    } else {
        gpio_set_level(cs_, 1);          // CS rising, INC LOW: no store
        gpio_set_level(inc_, 1);
    }
    vTaskDelay(pdMS_TO_TICKS(kCphMs));
}

void X9C103S::home(bool store)
{
    // Extra pulses beyond taps_ are harmless: the wiper stops at the end of the array.
    pulses(false, taps_ + 5, store);
    position_ = 0;
}

void X9C103S::calibrateToMax(bool store)
{
    // Extra pulses beyond taps_ are harmless: the wiper stops at the end of the array.
    pulses(true, taps_ + 5, store);
    position_ = taps_ - 1;
}

void X9C103S::setPosition(uint8_t position, bool store)
{
    position = std::min<uint8_t>(position, static_cast<uint8_t>(taps_ - 1));
    int delta = static_cast<int>(position) - static_cast<int>(position_);
    if (delta == 0) {
        if (store) {
            this->store();
        }
        return;
    }
    pulses(delta > 0, std::abs(delta), store);
    position_ = position;
}

void X9C103S::step(int delta, bool store)
{
    int target = std::clamp(static_cast<int>(position_) + delta, 0, taps_ - 1);
    setPosition(static_cast<uint8_t>(target), store);
}

void X9C103S::store()
{
    if (!initialized_) {
        return;
    }
    // Select the chip, then deselect it with INC HIGH: store.
    gpio_set_level(inc_, 1);
    gpio_set_level(cs_, 0);
    esp_rom_delay_us(kIcUs);
    gpio_set_level(cs_, 1);
    vTaskDelay(pdMS_TO_TICKS(kCphMs));
}

float X9C103S::resistanceOhms() const
{
    return (total_ohms_ / (taps_ - 1)) * position_ + kWiperOhms;
}

uint8_t X9C103S::setResistanceOhms(float ohms, bool store)
{
    float steps = (ohms - kWiperOhms) / (total_ohms_ / (taps_ - 1));
    int target = std::clamp(static_cast<int>(steps + 0.5f), 0, taps_ - 1);
    setPosition(static_cast<uint8_t>(target), store);
    return static_cast<uint8_t>(target);
}
