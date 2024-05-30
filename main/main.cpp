#include <chrono>
#include <thread>

#include <driver/gpio.h>

#include "high_resolution_timer.hpp"
#include "logger.hpp"
#include "oneshot_adc.hpp"
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

  static uint64_t button_press_start = 0;
  static uint64_t button_release_start = 0;
  static uint64_t latency_us = 0;
  static constexpr uint64_t IDLE_US = 50 * 1000; // time between button presses
  static constexpr uint64_t HOLD_TIME_US = CONFIG_BUTTON_HOLD_TIME_MS * 1000;

  static bool button_pressed = false;
  // randomly shift the button press time within the 1s period
  static int shift = 0;

#if CONFIG_DEBUG_PLOT_ALL
  enum class TestState : uint8_t {
    IDLE = 0,
    RELEASE_DETECTED = 1,
    BUTTON_RELEASED = 2,
    PRESS_DETECTED = 3,
    BUTTON_PRESSED = 4,
  };

  static auto state = TestState::IDLE;
  static constexpr uint64_t PERIOD_US = CONFIG_TRIGGER_PERIOD_MS * 1000;

  logger.info("Starting latency test, DEBUG_PLOT_ALL enabled");

  fmt::print("% time (s), state * 50, adc (mV), latency (ms)\n");
  espp::HighResolutionTimer timer({
      .name = "logging timer",
      .callback = [&]() {
        auto voltages = adc.read_all_mv();
        auto mv = voltages[0];
        auto now_us = esp_timer_get_time();
        uint64_t t = now_us % PERIOD_US;

        // reset the state at the beginning of the period
        if (t < IDLE_US) {
          shift = (rand() % 400) * 1000; // 0-400ms
          button_pressed = false;
          state = TestState::IDLE;
          button_press_start = 0;
          button_release_start = 0;
        }

        // trigger a button press
        if (state == TestState::IDLE && t >= (IDLE_US + shift)) {
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
        if (button_pressed && t > (IDLE_US + HOLD_TIME_US + shift)) {
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

#else // CONFIG_DEBUG_PLOT_ALL

  logger.info("Starting latency test");

  auto get_mv = [&]() {
    auto voltages = adc.read_all_mv();
    return voltages[0];
  };

  fmt::print("% time (s), latency (ms)\n");

  // Let's actually measure latency instead of debugging / logging
  while (true) {
    // reset the state at the beginning of the loop
    shift = (rand() % 400) * 1000; // 0-400ms
    button_press_start = 0;
    button_release_start = 0;
    static constexpr uint64_t MAX_lATENCY_US = 200 * 1000; // 200ms

    // wait for (IDLE_US + shift) microseconds
    std::this_thread::sleep_for(std::chrono::microseconds(IDLE_US + shift));

    // trigger a button press
    button_press_start = esp_timer_get_time();
    gpio_set_level(button_pin, BUTTON_PRESSED_LEVEL);

    // wait for the button press to be detected
    while (true) {
      auto now_us = esp_timer_get_time();
      latency_us = now_us - button_press_start;
      // read the ADC
      if (get_mv() > UPPER_THRESHOLD) {
        break;
      }
      // timeout
      if (latency_us > MAX_lATENCY_US) {
        break;
      }
    }

    // log the latency
    fmt::print("{:.3f}, {:.3f}\n", elapsed(), latency_us / 1e3f);

    // latency reached, release the button after hold time
    std::this_thread::sleep_for(std::chrono::microseconds(HOLD_TIME_US - latency_us));

    // release the button
    button_release_start = esp_timer_get_time();
    gpio_set_level(button_pin, BUTTON_RELEASED_LEVEL);

    // wait for the button release to be detected
    while (true) {
      auto now_us = esp_timer_get_time();
      latency_us = now_us - button_release_start;
      // read the ADC
      if (get_mv() < LOWER_THRESHOLD) {
        break;
      }
      // timeout
      if (latency_us > MAX_lATENCY_US) {
        break;
      }
    }

    // log the latency
    fmt::print("{:.3f}, {:.3f}\n", elapsed(), latency_us / 1e3f);
  }

#endif // CONFIG_DEBUG_PLOT_ALL
}