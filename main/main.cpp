#include <chrono>
#include <thread>

#include <driver/gpio.h>

#include "butterworth_filter.hpp"
#include "high_resolution_timer.hpp"
#include "logger.hpp"
#include "oneshot_adc.hpp"
#include "simple_lowpass_filter.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static auto elapsed = [&]() {
    uint64_t now = esp_timer_get_time();
    return now / 1e6f;
  };

  espp::Logger logger({.tag = "esp-latency-test", .level = espp::Logger::Verbosity::DEBUG});

  static constexpr gpio_num_t button_pin = (gpio_num_t)CONFIG_BUTTON_GPIO;
  static constexpr gpio_num_t extra_gnd_pin = (gpio_num_t)CONFIG_EXTRA_GND_GPIO;
  static constexpr int BUTTON_PRESSED_LEVEL = CONFIG_BUTTON_PRESS_LEVEL;
  static constexpr int BUTTON_RELEASED_LEVEL = !BUTTON_PRESSED_LEVEL;
  static constexpr int UPPER_THRESHOLD = CONFIG_UPPER_THRESHOLD;
  static constexpr int LOWER_THRESHOLD = CONFIG_LOWER_THRESHOLD;
  static constexpr adc_unit_t ADC_UNIT = CONFIG_SENSOR_ADC_UNIT == 1 ? ADC_UNIT_1 : ADC_UNIT_2;
  static constexpr adc_channel_t ADC_CHANNEL = (adc_channel_t)CONFIG_SENSOR_ADC_CHANNEL;

  std::vector<espp::AdcConfig> channels{
    // A0 on QtPy ESP32S3
    {.unit = ADC_UNIT, .channel = ADC_CHANNEL, .attenuation = ADC_ATTEN_DB_12}};
  espp::OneshotAdc adc({
      .unit = ADC_UNIT,
      .channels = channels,
    });

  gpio_set_direction(button_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(button_pin, BUTTON_RELEASED_LEVEL);
  gpio_set_direction(extra_gnd_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(extra_gnd_pin, 0);

  logger.info("Bootup");

  espp::SimpleLowpassFilter lp_filter{{.time_constant = 0.001f}};

  static constexpr float sample_freq_hz = 100.0f;
  static constexpr float filter_cutoff_freq_hz = 50.0f;
  static constexpr float normalized_cutoff_frequency =
      2.0f * filter_cutoff_freq_hz / sample_freq_hz;
  static constexpr size_t ORDER = 2;
  // NOTE: using the Df2 since it's hardware accelerated :)
  using Filter = espp::ButterworthFilter<ORDER, espp::BiquadFilterDf2>;
  Filter bw_filter({.normalized_cutoff_frequency = normalized_cutoff_frequency});

  enum class TestState : uint8_t {
    IDLE = 0,
    RELEASE_DETECTED = 1,
    BUTTON_RELEASED = 2,
    PRESS_DETECTED = 3,
    BUTTON_PRESSED = 4,
  };

  fmt::print("% time (s), state * 50, adc (mV), latency (ms)\n");
  espp::HighResolutionTimer timer({
      .name = "logging timer",
      .callback = [&]() {
        auto voltages = adc.read_all_mv();
        auto mv = voltages[0];
        auto now_us = esp_timer_get_time();

        static auto state = TestState::IDLE;
        static auto button_press_start = now_us;
        static auto button_release_start = now_us;
        static uint64_t latency_us = 0;
        static constexpr uint64_t PERIOD_US = CONFIG_TRIGGER_PERIOD_MS * 1000;
        static constexpr uint64_t MIN_IDLE_US = 10 * 1000; // 10ms
        static constexpr uint64_t HOLD_TIME_US = CONFIG_BUTTON_HOLD_TIME_MS * 1000;

        uint64_t t = now_us % PERIOD_US;

        static bool button_pressed = false;
        // randomly shift the button press time within the 1s period
        static int shift = 0;

        // reset the state at the beginning of the period
        if (t < MIN_IDLE_US) {
          shift = (rand() % 400) * 1000; // 0-400ms
          button_pressed = false;
          state = TestState::IDLE;
          button_press_start = 0;
          button_release_start = 0;
        }

        // trigger a button press
        if (state == TestState::IDLE && t >= (MIN_IDLE_US + shift)) {
          state = TestState::BUTTON_PRESSED;
          button_pressed = true;
          gpio_set_level(button_pin, BUTTON_PRESSED_LEVEL);
          button_press_start = now_us;
        }

        // try to detect the button press
        if (state == TestState::BUTTON_PRESSED && mv > UPPER_THRESHOLD) {
          state = TestState::PRESS_DETECTED;
          latency_us = now_us - button_press_start;
        }

        // trigger a button release if it's held long enough
        if (button_pressed && t > (MIN_IDLE_US + HOLD_TIME_US + shift)) {
          state = TestState::BUTTON_RELEASED;
          gpio_set_level(button_pin, BUTTON_RELEASED_LEVEL);
          button_release_start = now_us;
          button_pressed = false;
        }

        // try to detect the button release
        if (state == TestState::BUTTON_RELEASED && mv < LOWER_THRESHOLD) {
          state = TestState::RELEASE_DETECTED;
          latency_us = now_us - button_release_start;
        }

        fmt::print("{:.3f}, {}, {}, {:.3f}\n", elapsed(), (int)state * 50, mv, latency_us / 1e3f);
      }
    });
  uint64_t timer_period_us = 5 * 1000; // 5ms
  timer.periodic(timer_period_us);

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
