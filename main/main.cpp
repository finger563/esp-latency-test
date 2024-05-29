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

  auto button_pin = GPIO_NUM_17; // A1 on QtPy ESP32S3
  auto extra_gnd_pin = GPIO_NUM_9; // A2 on QtPy ESP32S3
  static constexpr int BUTTON_PRESSED_LEVEL = 0;
  static constexpr int BUTTON_RELEASED_LEVEL = 1;

  std::vector<espp::AdcConfig> channels{
    // A0 on QtPy ESP32S3
    {.unit = ADC_UNIT_2, .channel = ADC_CHANNEL_7, .attenuation = ADC_ATTEN_DB_11}};
  espp::OneshotAdc adc({
      .unit = ADC_UNIT_2,
      .channels = channels,
    });

  gpio_set_direction(button_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(button_pin, BUTTON_RELEASED_LEVEL);
  gpio_set_direction(extra_gnd_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(extra_gnd_pin, 0);

  logger.info("Bootup");

  // espp::SimpleLowpassFilter filter{{.time_constant = 0.01f}};

  // static constexpr float sample_freq_hz = 100.0f;
  // static constexpr float filter_cutoff_freq_hz = 10.0f;
  // static constexpr float normalized_cutoff_frequency =
  //     2.0f * filter_cutoff_freq_hz / sample_freq_hz;
  // static constexpr size_t ORDER = 2;
  // // NOTE: using the Df2 since it's hardware accelerated :)
  // using Filter = espp::ButterworthFilter<ORDER, espp::BiquadFilterDf2>;
  // Filter filter({.normalized_cutoff_frequency = normalized_cutoff_frequency});


  fmt::print("% time (s), gpio, adc (mV), latency (ms)\n");
  espp::HighResolutionTimer timer({
      .name = "logging timer",
      .callback = [&]() {
        static constexpr int UPPER_THRESHOLD = 260;
        static constexpr int LOWER_THRESHOLD = 200;
        auto voltages = adc.read_all_mv();
        const auto &mv = voltages[0];
        auto now_us = esp_timer_get_time();

        static auto button_press_start = now_us;
        static auto button_release_start = now_us;
        static uint64_t latency_us = 0;

        static int button_value = 0;
        // randomly shift the button press time within the 1s period
        static int shift = 0;
        if (now_us % 1000000 < (10 * 1000)) {
          // update the shift
          shift = (rand() % 400) * 1000;
        }
        if (button_value == 0 && now_us % 1000000 < (10 * 1000 + shift)) {
          gpio_set_level(button_pin, BUTTON_PRESSED_LEVEL);
          button_value = 200;
          button_press_start = now_us;
        }
        if (button_value == 200 && mv > UPPER_THRESHOLD) {
          latency_us = now_us - button_press_start;
          button_value = 100;
        }
        if (button_value == 100 && now_us % 1000000 > (500 * 1000 + shift)) {
          gpio_set_level(button_pin, BUTTON_RELEASED_LEVEL);
          button_release_start = now_us;
          button_value = 50;
        }
        if (button_value == 50 && mv < LOWER_THRESHOLD) {
          latency_us = now_us - button_release_start;
          button_value = 0;
        }
        fmt::print("{:.3f}, {}, {}, {:.3f}\n", elapsed(), button_value, mv, latency_us / 1e3f);
      }
    });
  uint64_t timer_period_us = 5 * 1000; // 5ms
  timer.periodic(timer_period_us);

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
