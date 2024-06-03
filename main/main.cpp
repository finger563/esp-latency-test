#include <chrono>
#include <thread>

#include <driver/gpio.h>
#include <esp_hidh.h>

#include "cli.hpp"
#include "high_resolution_timer.hpp"
#include "logger.hpp"
#include "nvs.hpp"
#include "oneshot_adc.hpp"
#include "simple_lowpass_filter.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

#if CONFIG_IDF_TARGET_ESP32
#include "esp_hid_gap.h"

static std::atomic<bool> connected{false};
static std::mutex mutex;
static std::condition_variable cv;

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
  esp_hidh_event_t event = (esp_hidh_event_t)id;
  esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;
  switch (event) {
  case ESP_HIDH_OPEN_EVENT:
    if (param->open.status == ESP_OK) {
      connected = true;
    }
    break;
  case ESP_HIDH_CLOSE_EVENT:
    connected = false;
    break;
  case ESP_HIDH_INPUT_EVENT:
    cv.notify_all();
    break;
  default:
    break;
  }
}
#endif // CONFIG_IDF_TARGET_ESP32

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

  std::error_code ec;
  espp::Nvs nvs;
  nvs.init(ec);
  if (ec) {
    logger.error("Failed to initialize NVS: {}", ec.message());
    return;
  }

  bool use_hid_host = false;

#if CONFIG_IDF_TARGET_ESP32
  // Initialize the bluetooth / HID Host
  logger.info("setting hid gap, mode: {}", HID_HOST_MODE);
  ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));

#if CONFIG_BT_BLE_ENABLED
  ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));
#endif // CONFIG_BT_BLE_ENABLED

  esp_hidh_config_t config = {
      .callback = hidh_callback,
      .event_stack_size = 4096,
      .callback_arg = NULL,
  };
  ESP_ERROR_CHECK(esp_hidh_init(&config));

#if CONFIG_BT_NIMBLE_ENABLED
  ble_store_config_init();

  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  // Starting nimble task after gatts is initialized
  ret = esp_nimble_enable(ble_hid_host_task);
  if (ret) {
    ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
  }
#endif // CONFIG_BT_NIMBLE_ENABLED

  // load previous settings (if any) from NVS
  // - HID Host or PhotoDiode (ADC)
  // - if using HID Host, select the device to connect to
  std::string device_name;
  nvs.get_or_set_var("latency", "use_hid_host", use_hid_host, use_hid_host, ec);
  if (ec) {
    logger.error("Failed to get use_hid_host from NVS: {}", ec.message());
    return;
  }
  logger.info("Loaded use_hid_host: {}", use_hid_host);
  if (use_hid_host) {
    nvs.get_var("latency", "device_name", device_name, ec);
    if (ec) {
      logger.error("Failed to get device_name from NVS: {}", ec.message());
      return;
    }
    logger.info("Loaded device_name: '{}'", device_name);
  }

  auto root_menu = std::make_unique<cli::Menu>("latency", "Latency Measurement Menu");
  // make some items for configuring the latency test:
  // - select whether to use HID Host or PhotoDiode (ADC)
  // - if using HID Host, select the device to connect to
  // - save the configuration to NVS

  std::unordered_map<std::string, esp_hid_scan_result_t *> devices;
  static esp_hid_scan_result_t *results = NULL;
  auto scan = [&](int seconds = 5) {
    size_t results_len = 0;
    if (results) {
      esp_hid_scan_results_free(results);
      results = NULL;
      devices.clear();
    }
    esp_hid_scan(seconds, &results_len, &results);
    // loop through the results and print them
    if (results_len > 0) {
      esp_hid_scan_result_t *r = results;
      while (r) {
        if (r->name == nullptr) {
          r = r->next;
          continue;
        }
        devices[r->name] = r;
        r = r->next;
      }
    }
  };

  auto scan_and_print = [&](std::ostream &out, int seconds = 5) {
    scan(seconds);
    if (devices.empty()) {
      out << "No devices found\n";
      return;
    }
    out << "Devices found:\n";
    for (const auto &[name, result] : devices) {
      out << "  '" << name << "'\n";
    }
  };

  auto scan_and_connect = [&](std::string device_name) -> bool {
    scan(5);
    if (devices.empty()) {
      logger.error("No devices found");
      return false;
    }
    auto it = devices.find(device_name);
    if (it == devices.end()) {
      logger.error("Device '{}' not found", device_name);
      return false;
    }
    auto result = it->second;
    logger.info("Connecting to device '{}'", device_name);
    esp_hidh_dev_open(result->bda, result->transport, result->ble.addr_type);
    return true;
  };

  root_menu->Insert(
      "use_hid_host", [&](std::ostream &out) { out << "use_hid_host: " << use_hid_host << "\n"; },
      "Get the current value of use_hid_host");
  root_menu->Insert(
      "use_hid_host", {"Use HID Host (true/false)"},
      [&](std::ostream &out, bool value) {
        use_hid_host = value;
        nvs.set_var("latency", "use_hid_host", use_hid_host, ec);
        out << "use_hid_host: " << use_hid_host << "\n";
      },
      "Set the value of use_hid_host");

  root_menu->Insert(
      "device_name", [&](std::ostream &out) { out << "device_name: " << device_name << "\n"; },
      "Get the current value of device_name");
  root_menu->Insert(
      "device_name", {"Device Name"},
      [&](std::ostream &out, std::string value) {
        device_name = value;
        nvs.set_var("latency", "device_name", (const std::string)device_name, ec);
        out << "device_name: " << device_name << "\n";
      },
      "Set the value of device_name");

  root_menu->Insert(
      "scan", [&](std::ostream &out) { scan_and_print(out); }, "Scan for HID devices (5 seconds)");

  root_menu->Insert(
      "scan", {"seconds"}, [&](std::ostream &out, int seconds) { scan_and_print(out, seconds); },
      "Scan for HID devices");

  // wait 5 seconds for the user to send input over stdin. If receive input, run
  // the cli, else start the test according to the defaults loaded from NVS.

  // if we've gotten here, either the CLI wasn't entered or the user exited it
  if (use_hid_host) {
    while (!scan_and_connect(device_name)) {
      logger.warn("Failed to connect to device '{}', retrying...", device_name);
    }
  }
#endif // CONFIG_IDF_TARGET_ESP32

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
  static constexpr uint64_t MAX_SHIFT_MS = CONFIG_MAX_BUTTON_DELAY_MS;

  static bool button_pressed = false;
  // randomly shift the button press time within the 1s period
  static int shift = 0;

#if CONFIG_FILTER_ADC
  // filter the ADC readings
  static constexpr float ALPHA = CONFIG_FILTER_ALPHA / 1e3f;
  espp::SimpleLowpassFilter filter({.time_constant = ALPHA});
#endif // CONFIG_FILTER_ADC

  auto get_mv = [&]() {
    auto voltages = adc.read_all_mv();
#if CONFIG_FILTER_ADC
    return filter(voltages[0]);
#else  // CONFIG_FILTER_ADC
    return voltages[0];
#endif // CONFIG_FILTER_ADC
  };

#if CONFIG_DEBUG_PLOT_ALL
  if (use_hid_host) {
    logger.warning("DEBUG_PLOT_ALL is not supported with HID Host, will log ADC values instead");
  }
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
  espp::HighResolutionTimer timer(
      {.name = "logging timer", .callback = [&]() {
         auto mv = get_mv();
         auto now_us = esp_timer_get_time();
         uint64_t t = now_us % PERIOD_US;

         // reset the state at the beginning of the period
         if (t < IDLE_US) {
           shift = (rand() % MAX_SHIFT_MS) * 1000;
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
       }});
  uint64_t timer_period_us = 5 * 1000; // 5ms
  timer.periodic(timer_period_us);

  while (true) {
    std::this_thread::sleep_for(1s);
  }

#else // CONFIG_DEBUG_PLOT_ALL

  logger.info("Starting latency test");

  fmt::print("% time (s), latency (ms)\n");

  // Let's actually measure latency instead of debugging / logging
  while (true) {
    // reset the state at the beginning of the loop
    shift = (rand() % MAX_SHIFT_MS) * 1000;
    button_press_start = 0;
    button_release_start = 0;
    static constexpr uint64_t MAX_lATENCY_US = 200 * 1000; // 200ms

    // wait for (IDLE_US + shift) microseconds
    std::this_thread::sleep_for(std::chrono::microseconds(IDLE_US + shift));

    // trigger a button press
    button_press_start = esp_timer_get_time();
    gpio_set_level(button_pin, BUTTON_PRESSED_LEVEL);

    // wait for the button press to be detected
    if (use_hid_host) {
      std::unique_lock<std::mutex> lock(mutex);
      auto retval = cv.wait_for(lock, 500ms);
      if (retval == std::cv_status::timeout) {
        logger.error("Timeout waiting for button press to be detected");
        continue;
      }
      latency_us = esp_timer_get_time() - button_press_start;
    } else {
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
    }

    // log the latency
    fmt::print("{:.3f}, {:.3f}\n", elapsed(), latency_us / 1e3f);

    // latency reached, release the button after hold time
    std::this_thread::sleep_for(std::chrono::microseconds(HOLD_TIME_US - latency_us));

    // release the button
    button_release_start = esp_timer_get_time();
    gpio_set_level(button_pin, BUTTON_RELEASED_LEVEL);

    // wait for the button release to be detected
    if (use_hid_host) {
      std::unique_lock<std::mutex> lock(mutex);
      auto retval = cv.wait_for(lock, 500ms);
      if (retval == std::cv_status::timeout) {
        logger.error("Timeout waiting for button release to be detected");
        continue;
      }
      latency_us = esp_timer_get_time() - button_release_start;
    } else {
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
    }

    // log the latency
    fmt::print("{:.3f}, {:.3f}\n", elapsed(), latency_us / 1e3f);
  }

#endif // CONFIG_DEBUG_PLOT_ALL
}
