#include <algorithm>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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

// Measured on this module: ~0.5 kOhm per step (X9C503, 50 kOhm), and "down" raised the
// resistance between VL and VW, i.e. the direction is reversed.
static constexpr float POT_TOTAL_OHM = 50000.0f;
static constexpr bool  POT_REVERSED  = true;

// Push buttons: wired between the GPIO and GND (internal pull-up is used, active LOW).
static constexpr gpio_num_t PIN_BTN_UP    = GPIO_NUM_32;  // increase resistance
static constexpr gpio_num_t PIN_BTN_DOWN  = GPIO_NUM_33;  // decrease resistance
static constexpr gpio_num_t PIN_BTN_STORE = GPIO_NUM_14;  // save position to non-volatile memory

// ---- Behaviour ----
static constexpr int      STEP_PER_PRESS   = 1;    // taps per press
static constexpr uint32_t DEBOUNCE_MS      = 30;
static constexpr uint32_t REPEAT_DELAY_MS  = 400;  // hold time before auto-repeat starts
static constexpr uint32_t REPEAT_PERIOD_MS = 80;   // auto-repeat interval while held

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

// Button edges are reported by a GPIO interrupt; the ISR only queues the pin number and
// all debouncing happens in the main task.
static QueueHandle_t s_btn_queue;

static void IRAM_ATTR button_isr(void *arg)
{
    uint32_t pin = reinterpret_cast<uint32_t>(arg);
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_btn_queue, &pin, &woken);
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

// Interrupt-driven button with press + auto-repeat events.
class Button {
public:
    explicit Button(gpio_num_t pin, bool repeat) : pin_(pin), repeat_(repeat) {}

    // Requires gpio_install_isr_service() to have been called.
    void init() const
    {
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << pin_;
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_ANYEDGE;
        ESP_ERROR_CHECK(gpio_config(&cfg));
        ESP_ERROR_CHECK(gpio_isr_handler_add(pin_, button_isr, reinterpret_cast<void *>(pin_)));
    }

    // Call after edges have settled. Returns true on a new press.
    bool settle(uint32_t now_ms)
    {
        bool pressed = (gpio_get_level(pin_) == 0);  // active low
        if (pressed == pressed_) {
            return false;
        }
        pressed_ = pressed;
        if (pressed_) {
            next_repeat_ = now_ms + REPEAT_DELAY_MS;
            return true;
        }
        return false;
    }

    // Returns true when an auto-repeat step is due while the button is held.
    bool repeatDue(uint32_t now_ms)
    {
        if (repeat_ && pressed_ && static_cast<int32_t>(now_ms - next_repeat_) >= 0) {
            next_repeat_ = now_ms + REPEAT_PERIOD_MS;
            return true;
        }
        return false;
    }

    // Milliseconds until the next auto-repeat step, or UINT32_MAX if none is pending.
    uint32_t msToRepeat(uint32_t now_ms) const
    {
        if (!repeat_ || !pressed_) {
            return UINT32_MAX;
        }
        int32_t left = static_cast<int32_t>(next_repeat_ - now_ms);
        return left > 0 ? static_cast<uint32_t>(left) : 0;
    }

private:
    gpio_num_t pin_;
    bool       repeat_;
    bool       pressed_     = false;
    uint32_t   next_repeat_ = 0;
};

static uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

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
    X9C103S pot(PIN_CS, PIN_INC, PIN_UD, POT_TOTAL_OHM, POT_REVERSED);
    ESP_ERROR_CHECK(pot.init());

    // Calibration: the chip restores the last stored wiper position at power-up and it cannot
    // be read back. Driving the wiper to the maximum end stop on every boot guarantees the
    // software counter matches the real resistance (position 99, ~POT_TOTAL_OHM).
    ESP_LOGI(TAG, "Calibrating: moving wiper to maximum...");
    pot.calibrateToMax();
    logOutput("Calibration done.", pot);

    s_btn_queue = xQueueCreate(16, sizeof(uint32_t));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    Button btnUp(PIN_BTN_UP, true);
    Button btnDown(PIN_BTN_DOWN, true);
    Button btnStore(PIN_BTN_STORE, false);
    btnUp.init();
    btnDown.init();
    btnStore.init();

    while (true) {
        // Sleep until a button edge arrives, or until the next auto-repeat step of a held button.
        uint32_t now  = nowMs();
        uint32_t wait = std::min(btnUp.msToRepeat(now), btnDown.msToRepeat(now));
        TickType_t timeout = (wait == UINT32_MAX)
                                 ? portMAX_DELAY
                                 : std::max<TickType_t>(1, pdMS_TO_TICKS(wait));

        uint32_t pin;
        if (xQueueReceive(s_btn_queue, &pin, timeout) == pdTRUE) {
            // Debounce: let the contacts settle, then drop the bounce edges queued meanwhile.
            vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(DEBOUNCE_MS)));
            xQueueReset(s_btn_queue);
        }

        now = nowMs();
        if (btnUp.settle(now) || btnUp.repeatDue(now)) {
            pot.step(+STEP_PER_PRESS);
            logOutput("UP  ", pot);
        }
        if (btnDown.settle(now) || btnDown.repeatDue(now)) {
            pot.step(-STEP_PER_PRESS);
            logOutput("DOWN", pot);
        }
        if (btnStore.settle(now)) {
            pot.store();
            ESP_LOGI(TAG, "Stored position=%u in non-volatile memory", pot.position());
        }
    }
}
