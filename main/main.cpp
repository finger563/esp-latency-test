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

static bool parse_input{false};
static uint8_t input_report_id{1};
static bool is_ble{false};
static std::atomic<bool> connected{false};
static std::mutex mutex;
static std::condition_variable cv;

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
  esp_hidh_event_t event = (esp_hidh_event_t)id;
  esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;
  switch (event) {
  case ESP_HIDH_OPEN_EVENT:
    if (param->open.status == ESP_OK) {
      fmt::print("Connected to device\n");
      connected = true;
      const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
#if CONFIG_BT_CLASSIC_ENABLED
      if (!is_ble) {
        // if BT, update the connection parameters
        // max time between packets = 60ms
        fmt::print("Setting QoS to 60ms\n");
        esp_bt_gap_set_qos((uint8_t *)bda, 96); // * 0.625ms = 60ms
      }
#endif // CONFIG_BT_CLASSIC_ENABLED
#if CONFIG_BT_BLE_ENABLED
      if (is_ble) {
        // if BLE, update the connection parameters
        esp_ble_conn_update_params_t conn_params = {
            .bda = {0},
            .min_int = 12, // * 1.25ms = 15ms
            .max_int = 24, // * 1.25ms = 30ms
            .latency = 0,
            .timeout = 400,
        };
        memcpy(conn_params.bda, bda, ESP_BD_ADDR_LEN);
        fmt::print("Setting BLE connection parameters\n");
        esp_ble_gap_update_conn_params(&conn_params);
      }
#endif // CONFIG_BT_BLE_ENABLED
    } else {
      fmt::print("Failed to connect to device\n");
    }
    break;
  case ESP_HIDH_CLOSE_EVENT:
    fmt::print("Disconnected from device\n");
    connected = false;
    break;
  case ESP_HIDH_INPUT_EVENT: {
    // for most controllers, they only send input reports when the state changes
    if (!parse_input) {
      std::unique_lock<std::mutex> lock(mutex);
      cv.notify_all();
      break;
    }
    // if we get here, we have to parse the input -.-
    // pull out the report id from the HID report
    uint8_t report_id = param->input.report_id;
    // ignore if it isn't the report we're looking for
    if (report_id != input_report_id) {
      break;
    }
    // for now we'll just hardcode the report parsing...
    // buttons are bytes 4 and 5 (0 index)
    // pull out the button state from the HID report
    uint16_t button_state = param->input.data[4] | (param->input.data[5] << 8);
    // initialize it to something it _shouldn't_ be so that first report comes
    // through as well.
    static auto last_button_state = 0;
    // if the button is pressed, notify the main thread
    if (button_state != last_button_state) {
      std::unique_lock<std::mutex> lock(mutex);
      cv.notify_all();
    }
    last_button_state = button_state;
  } break;
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
  nvs.get_or_set_var("latency", "use_hid_host", use_hid_host, use_hid_host, ec);
  if (ec) {
    logger.error("Failed to get use_hid_host from NVS: {}", ec.message());
    return;
  }
  logger.info("Loaded use_hid_host: {}", use_hid_host);

  std::string device_name;
  std::string device_address;
  if (use_hid_host) {
    // get the name
    nvs.get_var("latency", "device_name", device_name, ec);
    if (ec) {
      logger.warn("Failed to get device_name from NVS: {}", ec.message());
    } else {
      logger.info("Loaded device_name: '{}'", device_name);
    }
    ec.clear();
    // get device address (since the name may not show up...)
    nvs.get_var("latency", "device_address", device_address, ec);
    if (ec) {
      logger.warn("Failed to get device_address from NVS: {}", ec.message());
    } else {
      logger.info("Loaded device_address: '{}'", device_address);
    }
    ec.clear();
    // get whether or not we have to parse the input report
    nvs.get_or_set_var("latency", "parse_input", parse_input, parse_input, ec);
    if (ec) {
      logger.error("Failed to get parse_input from NVS: {}", ec.message());
      return;
    }
    logger.info("Loaded parse_input: {}", parse_input);
    // get the input report id
    nvs.get_or_set_var("latency", "input_report_id", input_report_id, input_report_id, ec);
    if (ec) {
      logger.error("Failed to get input_report_id from NVS: {}", ec.message());
      return;
    }
    logger.info("Loaded input_report_id: {}", input_report_id);
  }

  auto root_menu = std::make_unique<cli::Menu>("latency", "Latency Measurement Menu");
  // make some items for configuring the latency test:
  // - select whether to use HID Host or PhotoDiode (ADC)
  // - if using HID Host, select the device to connect to
  // - save the configuration to NVS

  std::unordered_map<std::string, esp_hid_scan_result_t *> devices;
  static esp_hid_scan_result_t *results = NULL;
  auto scan = [&](int seconds = 5) {
    logger.info("Scanning for devices for {} seconds", seconds);
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
        bool has_name = r->name != nullptr;
        bool is_gamepad = r->usage == ESP_HID_USAGE_GAMEPAD;
        std::string address =
            fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", ESP_BD_ADDR_HEX(r->bda));
        std::string name = has_name ? std::string(r->name) : address;
        logger.info("Found device: {} ({}), addr: {}", name, is_gamepad ? "Gamepad" : "Unknown",
                    address);
        // use either the name (if it exists) or the address as the key
        logger.info("Found device '{}'", name);
        devices[name] = r;
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

  auto connect = [&](const std::string &remote_name,
                     const std::string &remote_address = "") -> bool {
    esp_hid_scan_result_t *result = nullptr;
    auto it = std::find_if(devices.begin(), devices.end(), [&](const auto &pair) {
      const auto &name = pair.first;
      // check both remote name and remote address since the name may not show up
      return remote_name.contains(name) || name.contains(remote_name) ||
             remote_address.contains(name) || name.contains(remote_address);
    });
    if (it != devices.end()) {
      result = it->second;
      logger.info("Found device '{}'", result->name ? result->name : "Unknown");
    } else {
      logger.error("Device '{}' not found", remote_name);
      return false;
    }
    logger.info("Connecting to device '{}'", remote_name);
    logger.info("          at address {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                ESP_BD_ADDR_HEX(result->bda));
#if CONFIG_BT_CLASSIC_ENABLED && CONFIG_BT_BLE_ENABLED
    if (result->transport == ESP_HID_TRANSPORT_BLE) {
      is_ble = true;
#endif // CONFIG_BT_CLASSIC_ENABLED
      logger.info("          using BLE, addr_type: {}", (int)result->ble.addr_type);

      // if BLE, update the connection parameters
      static constexpr uint16_t min_interval = 12; // 15ms
      static constexpr uint16_t max_interval = 80; // 100ms
      esp_ble_gap_set_prefer_conn_params(result->bda, min_interval, max_interval, 0, 400);

      // now connect
      esp_hidh_dev_open(result->bda, ESP_HID_TRANSPORT_BLE, result->ble.addr_type);
#if CONFIG_BT_CLASSIC_ENABLED && CONFIG_BT_BLE_ENABLED
    } else {
      is_ble = false;
      logger.info("          using BT");
      esp_hidh_dev_open(result->bda, ESP_HID_TRANSPORT_BT, 0);
    }
#endif // CONFIG_BT_CLASSIC_ENABLED
    // now free the results
    esp_hid_scan_results_free(results);
    return true;
  };

  auto scan_and_connect = [&](int num_seconds = 5) -> bool {
    scan(num_seconds);
    if (devices.empty()) {
      logger.error("No devices found");
      return false;
    }
    return connect(device_name, device_address);
  };

  // menu functions for getting / setting use_hid_host
  root_menu->Insert(
      "use_hid_host", [&](std::ostream &out) { out << "use_hid_host: " << use_hid_host << "\n"; },
      "Get the current value of use_hid_host");
  root_menu->Insert(
      "use_hid_host", {"Use HID Host (true/false)"},
      [&](std::ostream &out, bool value) {
        use_hid_host = value;
        std::error_code ec;
        nvs.set_var("latency", "use_hid_host", use_hid_host, ec);
        if (ec) {
          out << "Failed to set use_hid_host: " << ec.message() << "\n";
        } else {
          out << "use_hid_host: " << use_hid_host << "\n";
        }
      },
      "Set the value of use_hid_host");

  // menu functions for getting / setting parse_input
  root_menu->Insert(
      "parse_input", [&](std::ostream &out) { out << "parse_input: " << parse_input << "\n"; },
      "Get the current value of parse_input");
  root_menu->Insert(
      "parse_input", {"Parse Input (true/false)"},
      [&](std::ostream &out, bool value) {
        parse_input = value;
        std::error_code ec;
        nvs.set_var("latency", "parse_input", parse_input, ec);
        if (ec) {
          out << "Failed to set parse_input: " << ec.message() << "\n";
        } else {
          out << "parse_input: " << parse_input << "\n";
        }
      },
      "Set the value of parse_input");

  // menu functions for getting / setting input_report_id
  root_menu->Insert(
      "input_report_id",
      [&](std::ostream &out) { out << "input_report_id: " << (int)input_report_id << "\n"; },
      "Get the current value of input_report_id");
  root_menu->Insert(
      "input_report_id", {"Input Report ID (0-255)"},
      [&](std::ostream &out, int value) {
        input_report_id = value;
        std::error_code ec;
        nvs.set_var("latency", "input_report_id", input_report_id, ec);
        if (ec) {
          out << "Failed to set input_report_id: " << ec.message() << "\n";
        } else {
          out << "input_report_id: " << (int)input_report_id << "\n";
        }
      },
      "Set the value of input_report_id");

  // add menu functions for getting / setting device_name
  root_menu->Insert(
      "device_name", [&](std::ostream &out) { out << "device_name: " << device_name << "\n"; },
      "Get the current value of device_name");
  root_menu->Insert(
      "device_name", {"Device Name"},
      [&](std::ostream &out, const std::string &value) {
        device_name = value;
        std::error_code ec;
        nvs.set_var("latency", "device_name", (const std::string)device_name, ec);
        if (ec) {
          out << "Failed to set device_name: " << ec.message() << "\n";
        } else {
          out << "device_name: " << device_name << "\n";
        }
        if (devices.contains(device_name)) {
          auto result = devices[device_name];
          device_address = fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                                       ESP_BD_ADDR_HEX(result->bda));
          out << "device_address: " << device_address << "\n";
          nvs.set_var("latency", "device_address", (const std::string)device_address, ec);
        }
      },
      "Set the value of device_name");

  root_menu->Insert(
      "scan", [&](std::ostream &out) { scan_and_print(out); }, "Scan for HID devices (5 seconds)");

  root_menu->Insert(
      "connect",
      [&](std::ostream &out) {
        if (connect(device_name, device_address)) {
          out << "Connected to device '" << device_name << "'\n";
        } else {
          out << "Failed to connect to device\n";
        }
      },
      "Connect to the device with the given name");
  root_menu->Insert(
      "connect", {"device_name"},
      [&](std::ostream &out, const std::string &name) {
        if (connect(name)) {
          out << "Connected to device '" << name << "'\n";
        } else {
          out << "Failed to connect to device\n";
        }
      },
      "Connect to the device with the given name");

  root_menu->Insert(
      "scan", {"seconds"}, [&](std::ostream &out, int seconds) { scan_and_print(out, seconds); },
      "Scan for HID devices");

  // wait 5 seconds for the user to send input over stdin. If receive input, run
  // the cli, else start the test according to the defaults loaded from NVS.
  bool run_menu = false;
  auto start = esp_timer_get_time();
  static constexpr int num_seconds = 3;
  logger.info("Press any key to enter the CLI menu, or wait {} seconds to start the test",
              num_seconds);
  while (true) {
    // delay a little bit
    std::this_thread::sleep_for(500ms);
    char c;
    // see if there is any data available on stdin
    if (read(fileno(stdin), &c, 1) > 0) {
      // if there is, run the menu
      run_menu = true;
      break;
    }
    if (esp_timer_get_time() - start > num_seconds * 1e6) {
      // if 5 seconds have passed, start the test
      break;
    }
  }

  if (run_menu) {
    logger.info("Entering CLI menu");
    cli::SetColor();
    cli::Cli cli(std::move(root_menu));
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start();
  }

  // if we've gotten here, either the CLI wasn't entered or the user exited it
  if (use_hid_host) {
    logger.info("Configured to use HID Host, connecting to device '{}'", device_name);
    while (!connected && !scan_and_connect(3)) {
      logger.warn("Failed to connect to device '{}', retrying...", device_name);
    }
    logger.info("Connected!");
  }
#endif                 // CONFIG_IDF_TARGET_ESP32
  if (!use_hid_host) { // cppcheck-suppress knownConditionTrueFalse
    logger.info("Configured to use PhotoDiode (ADC)");
  }

  std::vector<espp::AdcConfig> channels{
      // A0 on QtPy ESP32S3
      {.unit = ADC_UNIT, .channel = ADC_CHANNEL, .attenuation = ADC_ATTEN_DB_12}};
  espp::OneshotAdc adc({
      .unit = ADC_UNIT,
      .channels = channels,
  });

  logger.info("Setting up GPIO pins");
  logger.info("Button pin: {}, extra GND pin: {}", (int)button_pin, (int)extra_gnd_pin);
  logger.info("Button pressed level: {}, released level: {}", BUTTON_PRESSED_LEVEL,
              BUTTON_RELEASED_LEVEL);
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
  if (use_hid_host) { // cppcheck-suppresss knownConditionTrueFalse
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
         static bool button_pressed = false;
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
