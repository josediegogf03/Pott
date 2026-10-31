#include <algorithm>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#include "x9c103s.h"

static const char *TAG = "main";

// ---- Pin configuration (change to match your wiring) ----
// X9C103S
static constexpr gpio_num_t PIN_CS  = GPIO_NUM_27;
static constexpr gpio_num_t PIN_INC = GPIO_NUM_19;
static constexpr gpio_num_t PIN_UD  = GPIO_NUM_18;

// Measured on this module (marked X9C103S): the ADC sweep moves ~106 mV per step and goes
// from bottom to top in 31 steps, and the multimeter shows ~0.35 kOhm per step, ~10.9 kOhm at
// the top. So it is a 32-position pot, not the datasheet's 100.
// Set POT_REVERSED if UP lowers the resistance between VL and VW.
static constexpr int   POT_TAPS      = 32;
static constexpr float POT_TOTAL_OHM = 31 * 350.0f;
static constexpr bool  POT_REVERSED  = false;

// Push buttons: wired between the GPIO and GND (internal pull-up is used, active LOW).
static constexpr gpio_num_t PIN_BTN_UP    = GPIO_NUM_32;  // increase resistance
static constexpr gpio_num_t PIN_BTN_DOWN  = GPIO_NUM_33;  // decrease resistance
static constexpr gpio_num_t PIN_BTN_STORE = GPIO_NUM_14;  // save position to non-volatile memory

// ---- Behaviour ----
static constexpr int      STEP_PER_PRESS   = 1;    // taps per press
static constexpr uint32_t DEBOUNCE_MS      = 30;
static constexpr uint32_t REPEAT_DELAY_MS  = 400;  // hold time before auto-repeat starts
static constexpr uint32_t REPEAT_PERIOD_MS = 80;   // auto-repeat interval while held

// ---- Wiper ADC test ----
// Test wiring: VH -> 3V3, VL -> GND, VW -> PIN_VW.
// GPIO4 is on ADC2, which cannot be used while Wi-Fi is on (this project does not use Wi-Fi).
static constexpr gpio_num_t    PIN_VW        = GPIO_NUM_4;     // ADC2 channel 0
static constexpr adc_unit_t    VW_UNIT       = ADC_UNIT_2;
static constexpr adc_channel_t VW_CHANNEL    = ADC_CHANNEL_0;
// Expected wiper voltage, fitted to the measured readings:
//   VW_OFFSET_V + position * VW_STEP_V, clamped to what the ADC can read (VW_MIN_V..VW_MAX_V).
static constexpr float         VW_OFFSET_V   = 0.032f;         // line extended to position 0
static constexpr float         VW_STEP_V     = 0.109f;         // per position
static constexpr float         VW_MIN_V      = 0.128f;         // lowest ADC reading (position 0)
static constexpr float         VW_MAX_V      = 3.144f;         // highest ADC reading (positions 29-31)
static constexpr int           ADC_SAMPLES   = 32;             // readings averaged per measurement
static constexpr uint32_t      SWEEP_STEP_MS = 200;            // pause between sweep steps
static constexpr int           STEP_SEEN_MV  = 15;             // a bigger change counts as a real move

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

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;

static void adcInit()
{
    adc_oneshot_unit_init_cfg_t unit = {};
    unit.unit_id = VW_UNIT;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &s_adc));

    // 12 dB attenuation reads up to ~3.1 V; the ESP32 ADC is most accurate below ~2.5 V.
    adc_oneshot_chan_cfg_t chan = {};
    chan.atten = ADC_ATTEN_DB_12;
    chan.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, VW_CHANNEL, &chan));

    adc_cali_line_fitting_config_t cali = {};
    cali.unit_id = VW_UNIT;
    cali.atten = ADC_ATTEN_DB_12;
    cali.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali, &s_cali));
    ESP_LOGI(TAG, "Reading VW on GPIO%d (ADC%d channel %d)", PIN_VW, VW_UNIT + 1, VW_CHANNEL);
}

// Wiper voltage in mV, averaged over ADC_SAMPLES readings.
static int readWiperMv()
{
    int sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        int raw = 0;
        int mv = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc, VW_CHANNEL, &raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(s_cali, raw, &mv));
        sum += mv;
    }
    return sum / ADC_SAMPLES;
}

// Logs the expected and measured wiper voltage; returns the measured value in mV.
static int logWiper(const char *event, const X9C103S &pot)
{
    int mv = readWiperMv();
    float expected = std::clamp(VW_OFFSET_V + pot.position() * VW_STEP_V, VW_MIN_V, VW_MAX_V);
    ESP_LOGI(TAG, "%s position=%2u  expected %.3f V  measured %.3f V",
             event, pot.position(), expected, mv / 1000.0f);
    return mv;
}

// Steps down one position at a time from the top and counts how many single-step commands
// really moved the wiper. Every step should change it by ~VW_STEP_V.
static void sweepDown(X9C103S &pot)
{
    ESP_LOGI(TAG, "Sweep: stepping down one position every %lu ms...", SWEEP_STEP_MS);
    int prev = readWiperMv();
    int start = prev;
    int moved = 0;
    while (pot.position() > 0) {
        pot.step(-1);
        vTaskDelay(pdMS_TO_TICKS(SWEEP_STEP_MS));
        int mv = logWiper("sweep", pot);
        if (prev - mv > STEP_SEEN_MV) {
            moved++;
        }
        prev = mv;
    }
    ESP_LOGI(TAG, "Sweep done: %d of %d step commands moved the wiper (%.3f V -> %.3f V, "
             "~%d mV per moving step)", moved, POT_TAPS - 1, start / 1000.0f, prev / 1000.0f,
             moved > 0 ? (start - prev) / moved : 0);
}

extern "C" void app_main(void)
{
    X9C103S pot(PIN_CS, PIN_INC, PIN_UD, POT_TOTAL_OHM, POT_REVERSED, POT_TAPS);
    ESP_ERROR_CHECK(pot.init());

    // Calibration: the chip restores the last stored wiper position at power-up and it cannot
    // be read back. Driving the wiper to the maximum end stop on every boot guarantees the
    // software counter matches the real resistance (position POT_TAPS - 1, ~POT_TOTAL_OHM).
    ESP_LOGI(TAG, "Calibrating: moving wiper to maximum...");
    pot.calibrateToMax();
    adcInit();
    logWiper("Calibration done.", pot);
    sweepDown(pot);

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
            logWiper("UP  ", pot);
        }
        if (btnDown.settle(now) || btnDown.repeatDue(now)) {
            pot.step(-STEP_PER_PRESS);
            logWiper("DOWN", pot);
        }
        if (btnStore.settle(now)) {
            pot.store();
            ESP_LOGI(TAG, "Stored position=%u in non-volatile memory", pot.position());
        }
    }
}
