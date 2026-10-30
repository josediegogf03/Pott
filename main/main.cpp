#include <algorithm>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "x9c103s.h"

static const char *TAG = "main";

// ---- Pin configuration (change to match your wiring) ----
// X9C103S
static constexpr gpio_num_t PIN_CS  = GPIO_NUM_27;
static constexpr gpio_num_t PIN_INC = GPIO_NUM_19;
static constexpr gpio_num_t PIN_UD  = GPIO_NUM_18;

// Push buttons: wired between the GPIO and GND (internal pull-up is used, active LOW).
static constexpr gpio_num_t PIN_BTN_UP    = GPIO_NUM_32;  // increase resistance
static constexpr gpio_num_t PIN_BTN_DOWN  = GPIO_NUM_33;  // decrease resistance
static constexpr gpio_num_t PIN_BTN_STORE = GPIO_NUM_14;  // save position to non-volatile memory

// ---- Behaviour ----
static constexpr int      STEP_PER_PRESS   = 1;    // taps per press
static constexpr uint32_t DEBOUNCE_MS      = 30;
static constexpr uint32_t REPEAT_DELAY_MS  = 400;  // hold time before auto-repeat starts
static constexpr uint32_t REPEAT_PERIOD_MS = 80;   // auto-repeat interval while held
static constexpr uint32_t POLL_MS          = 5;

// ---- Buck converter (LM2596 module, X9C103S replaces its trimmer) ----
// Wiring: FB -> VL, OUT+ -> VW and VH. The module's fixed resistor stays between FB and GND.
// Vout = Vref * (1 + R(VL-VW) / R_low), capped at Vin minus the LM2596 dropout.
// Keep Vin <= 5 V with this wiring: VW/VH sit at Vout and the X9C103S terminals are 5 V max.
static constexpr float BUCK_VREF_V    = 1.25f;   // measured on the FB pin
static constexpr float BUCK_R_LOW_OHM = 820.0f;  // measured between FB and GND
static constexpr float BUCK_VIN_V     = 5.0f;    // buck input voltage
static constexpr float BUCK_DROPOUT_V = 1.5f;    // approximate LM2596 dropout

// NOTE: the X9C103S is a 5 V part. The ESP32's 3.3 V outputs are usually accepted as logic high,
// but if your module misbehaves, add a level shifter.

// Debounced button with press + auto-repeat events.
class Button {
public:
    explicit Button(gpio_num_t pin, bool repeat) : pin_(pin), repeat_(repeat) {}

    void init() const
    {
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << pin_;
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&cfg));
    }

    // Returns true once per press, and repeatedly while held (if repeat is enabled).
    bool update(uint32_t now_ms)
    {
        bool raw = (gpio_get_level(pin_) == 0);  // active low

        if (raw != candidate_) {
            candidate_ = raw;
            changed_at_ = now_ms;
        }

        if ((now_ms - changed_at_) >= DEBOUNCE_MS && candidate_ != pressed_) {
            pressed_ = candidate_;
            if (pressed_) {
                next_repeat_ = now_ms + REPEAT_DELAY_MS;
                return true;  // new press
            }
        }

        if (repeat_ && pressed_ && now_ms >= next_repeat_) {
            next_repeat_ = now_ms + REPEAT_PERIOD_MS;
            return true;  // held: auto-repeat
        }
        return false;
    }

private:
    gpio_num_t pin_;
    bool       repeat_;
    bool       candidate_   = false;
    bool       pressed_     = false;
    uint32_t   changed_at_  = 0;
    uint32_t   next_repeat_ = 0;
};

// Estimated (not measured) buck output for the current wiper position.
static void logOutput(const char *event, const X9C103S &pot)
{
    float vout = BUCK_VREF_V * (1.0f + pot.resistanceOhms() / BUCK_R_LOW_OHM);
    float vmax = BUCK_VIN_V - BUCK_DROPOUT_V;
    if (vout > vmax) {
        ESP_LOGI(TAG, "%s position=%u (~%.0f ohm) -> Vout ~%.2f V (limited by Vin)",
                 event, pot.position(), pot.resistanceOhms(), vmax);
    } else {
        ESP_LOGI(TAG, "%s position=%u (~%.0f ohm) -> Vout ~%.2f V",
                 event, pot.position(), pot.resistanceOhms(), vout);
    }
}

extern "C" void app_main(void)
{
    X9C103S pot(PIN_CS, PIN_INC, PIN_UD);
    ESP_ERROR_CHECK(pot.init());

    // Calibration: the chip restores the last stored wiper position at power-up and it cannot
    // be read back. Driving the wiper to the maximum end stop on every boot guarantees the
    // software counter matches the real resistance (position 99, ~10 kOhm).
    ESP_LOGI(TAG, "Calibrating: moving wiper to maximum...");
    pot.calibrateToMax();
    logOutput("Calibration done.", pot);

    Button btnUp(PIN_BTN_UP, true);
    Button btnDown(PIN_BTN_DOWN, true);
    Button btnStore(PIN_BTN_STORE, false);
    btnUp.init();
    btnDown.init();
    btnStore.init();

    while (true) {
        uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);

        if (btnUp.update(now)) {
            pot.step(+STEP_PER_PRESS);
            logOutput("UP  ", pot);
        }
        if (btnDown.update(now)) {
            pot.step(-STEP_PER_PRESS);
            logOutput("DOWN", pot);
        }
        if (btnStore.update(now)) {
            pot.store();
            ESP_LOGI(TAG, "Stored position=%u in non-volatile memory", pot.position());
        }

        // At the default 100 Hz tick, 5 ms rounds down to 0 ticks and the loop would never
        // yield, starving the idle task (task watchdog). Always sleep at least one tick.
        vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(POLL_MS)));
    }
}
