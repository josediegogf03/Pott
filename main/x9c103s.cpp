#include "x9c103s.h"

#include <algorithm>
#include <cstdlib>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "X9C103S";

// Timing (datasheet minimums are lower; these leave margin).
static constexpr uint32_t kSetupUs   = 5;   // CS->INC and U/D->INC setup (tCI >= 100 ns, tDI >= 2.9 us)
static constexpr uint32_t kPulseUs   = 5;   // INC high/low time (tIL/tIH >= 1 us)
static constexpr uint32_t kSettleUs  = 100; // wiper settle after the last step (tIW ~ 100 us)
static constexpr uint32_t kStoreMs   = 20;  // non-volatile write time (tWR max 10 ms)

X9C103S::X9C103S(gpio_num_t cs, gpio_num_t inc, gpio_num_t ud, float total_ohms, bool reversed)
    : cs_(cs), inc_(inc), ud_(ud), total_ohms_(total_ohms), reversed_(reversed) {}

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

    // Idle state: chip deselected, INC high. Deselecting with INC high would store the
    // wiper, so set the levels in this order: INC high first, then CS high only after
    // making sure no store is triggered unintentionally (CS is high already => no-op).
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

    gpio_set_level(ud_, (up != reversed_) ? 1 : 0);
    esp_rom_delay_us(kSetupUs);

    gpio_set_level(cs_, 0);          // select the chip (INC is high, so nothing moves)
    esp_rom_delay_us(kSetupUs);

    // Every HIGH->LOW edge of INC while CS is low moves the wiper, so the sequence must
    // contain exactly `count` falling edges. INC is left LOW after the last one.
    for (int i = 0; i < count; i++) {
        if (i > 0) {
            gpio_set_level(inc_, 1);     // rising edge: no movement
            esp_rom_delay_us(kPulseUs);
        }
        gpio_set_level(inc_, 0);         // falling edge = one step
        esp_rom_delay_us(kPulseUs);
    }
    esp_rom_delay_us(kSettleUs);

    if (store) {
        // CS rising while INC is high => wiper position saved to non-volatile memory.
        gpio_set_level(inc_, 1);
        esp_rom_delay_us(kSetupUs);
        gpio_set_level(cs_, 1);
        vTaskDelay(pdMS_TO_TICKS(kStoreMs));
    } else {
        // CS rising while INC is low => no store. Then restore INC high
        // (a rising INC while deselected is ignored).
        gpio_set_level(cs_, 1);
        esp_rom_delay_us(kSetupUs);
        gpio_set_level(inc_, 1);
    }
}

void X9C103S::home(bool store)
{
    // Extra pulses beyond kTaps are harmless: the wiper stops at the end of the array.
    pulses(false, kTaps + 5, store);
    position_ = 0;
}

void X9C103S::calibrateToMax(bool store)
{
    // Extra pulses beyond kTaps are harmless: the wiper stops at the end of the array.
    pulses(true, kTaps + 5, store);
    position_ = kTaps - 1;
}

void X9C103S::setPosition(uint8_t position, bool store)
{
    position = std::min<uint8_t>(position, kTaps - 1);
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
    int target = std::clamp(static_cast<int>(position_) + delta, 0, kTaps - 1);
    setPosition(static_cast<uint8_t>(target), store);
}

void X9C103S::store()
{
    if (!initialized_) {
        return;
    }
    // Select the chip and deselect it again with INC high => store.
    gpio_set_level(inc_, 1);
    esp_rom_delay_us(kSetupUs);
    gpio_set_level(cs_, 0);
    esp_rom_delay_us(kSetupUs);
    gpio_set_level(cs_, 1);
    vTaskDelay(pdMS_TO_TICKS(kStoreMs));
}

float X9C103S::resistanceOhms() const
{
    return (total_ohms_ / (kTaps - 1)) * position_ + kWiperOhms;
}

uint8_t X9C103S::setResistanceOhms(float ohms, bool store)
{
    float steps = (ohms - kWiperOhms) / (total_ohms_ / (kTaps - 1));
    int target = std::clamp(static_cast<int>(steps + 0.5f), 0, kTaps - 1);
    setPosition(static_cast<uint8_t>(target), store);
    return static_cast<uint8_t>(target);
}
