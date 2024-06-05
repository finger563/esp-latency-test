#include <chrono>
#include <condition_variable>
#include <string>
#include <thread>
#include <unordered_map>

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

#include "esp_hid_gap.h"

// logger for the latency test
static espp::Logger logger({.tag = "esp-latency-test", .level = espp::Logger::Verbosity::DEBUG});

// variables loaded from menuconfig
static constexpr gpio_num_t button_pin = (gpio_num_t)CONFIG_BUTTON_GPIO;
static constexpr gpio_num_t extra_gnd_pin = (gpio_num_t)CONFIG_EXTRA_GND_GPIO;
static int BUTTON_PRESSED_LEVEL = 1;
static int BUTTON_RELEASED_LEVEL = !BUTTON_PRESSED_LEVEL;
static constexpr int UPPER_THRESHOLD = CONFIG_UPPER_THRESHOLD;
static constexpr int LOWER_THRESHOLD = CONFIG_LOWER_THRESHOLD;
static constexpr adc_unit_t ADC_UNIT = CONFIG_SENSOR_ADC_UNIT == 1 ? ADC_UNIT_1 : ADC_UNIT_2;
static constexpr adc_channel_t ADC_CHANNEL = (adc_channel_t)CONFIG_SENSOR_ADC_CHANNEL;
static std::vector<espp::AdcConfig> channels{
    // A0 on QtPy ESP32S3
    {.unit = ADC_UNIT, .channel = ADC_CHANNEL, .attenuation = ADC_ATTEN_DB_12}};
static std::shared_ptr<espp::OneshotAdc> adc{nullptr};

// things we'll load from NVS
static bool use_hid_host{false};
static std::string device_name{};
static std::string device_address{};
static bool parse_input{false};
static uint8_t input_report_id{1};
static uint8_t ble_min_interval_units{12}; // 15ms
static uint8_t ble_max_interval_units{80}; // 100ms
static uint8_t bt_qos_units{96};           // 60ms

// runtime state for HID mode
static bool is_ble{false};
static std::atomic<bool> connected{false};
static std::mutex mutex;
static std::condition_variable cv;
static std::unordered_map<std::string, esp_hid_scan_result_t *> devices;
static esp_hid_scan_result_t *results = NULL;

// utility functions
float get_mv();
void init_hid();
void load_nvs(espp::Nvs &nvs);
void build_menu(std::unique_ptr<cli::Menu> &root_menu, espp::Nvs &nvs);

// HID / BT / BLE functions
uint8_t ble_interval_ms_to_units(uint16_t interval_ms) { return interval_ms / 1.25f; }
uint8_t ble_interval_units_to_ms(uint8_t interval_units) { return interval_units * 1.25f; }
uint8_t bt_qos_ms_to_units(uint16_t qos_ms) { return qos_ms / 0.625f; }
uint16_t bt_qos_units_to_ms(uint8_t qos_units) { return qos_units * 0.625f; }
void scan(int seconds = 5);
void scan_and_print(std::ostream &out, int seconds = 5);
bool connect(const std::string &remote_name, const std::string &remote_address = "");
bool scan_and_connect(int num_seconds = 5);
void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);

extern "C" void app_main(void) {
  static auto elapsed = [&]() {
    uint64_t now = esp_timer_get_time();
    return now / 1e6f;
  };

  logger.info("Bootup");

  std::error_code ec;
  espp::Nvs nvs;
  nvs.init(ec);
  if (ec) {
    logger.error("Failed to initialize NVS: {}", ec.message());
    return;
  }

  // Initialize the bluetooth / HID Host
  init_hid();

  // load previous settings (if any) from NVS
  load_nvs(nvs);

  // make the ADC regardless of whether we're using it or not, esp. because
  // DEBUG_PLOT_ALL uses it
  adc = std::make_shared<espp::OneshotAdc>(espp::OneshotAdc::Config{
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

  auto root_menu = std::make_unique<cli::Menu>("latency", "Latency Measurement Menu");
  // make some items for configuring the latency test:
  // - select whether to use HID Host or PhotoDiode (ADC)
  // - if using HID Host, select the device to connect to
  // - save the configuration to NVS
  build_menu(root_menu, nvs);

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
    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start();
    logger.info("CLI complete, starting test");
  }

  // if we've gotten here, either the CLI wasn't entered or the user exited it
  if (use_hid_host) {
    logger.info("Configured to use HID Host, connecting to device '{}'", device_name);
    while (!connected && !scan_and_connect(3)) {
      logger.warn("Failed to connect to device '{}', retrying...", device_name);
    }
    logger.info("Connected!");
  } else {
    logger.info("Configured to use PhotoDiode (ADC)");
  }

  static uint64_t button_press_start = 0;
  static uint64_t button_release_start = 0;
  static uint64_t latency_us = 0;
  static constexpr uint64_t IDLE_US = 50 * 1000; // time between button presses
  static constexpr uint64_t HOLD_TIME_US = CONFIG_BUTTON_HOLD_TIME_MS * 1000;
  static constexpr uint64_t MAX_SHIFT_MS = CONFIG_MAX_BUTTON_DELAY_MS;

  // randomly shift the button press time within the 1s period
  static int shift = 0;

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

float get_mv() {
  auto voltages = adc->read_all_mv();
#if CONFIG_FILTER_ADC
  // filter the ADC readings
  static constexpr float ALPHA = CONFIG_FILTER_ALPHA / 1e3f;
  static espp::SimpleLowpassFilter filter({.time_constant = ALPHA});
  return filter(voltages[0]);
#else  // CONFIG_FILTER_ADC
  return voltages[0];
#endif // CONFIG_FILTER_ADC
}

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
  esp_hidh_event_t event = (esp_hidh_event_t)id;
  esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;
  switch (event) {
  case ESP_HIDH_OPEN_EVENT:
    if (param->open.status == ESP_OK) {
      fmt::print("Connected to device\n");
      connected = true;
      [[maybe_unused]] const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
#if CONFIG_BT_CLASSIC_ENABLED
      if (!is_ble) {
        // if BT, update the connection parameters
        // max time between packets
        fmt::print("Setting QoS\n");
        esp_bt_gap_set_qos((uint8_t *)bda, bt_qos_units);
      }
#endif // CONFIG_BT_CLASSIC_ENABLED
#if CONFIG_BT_BLE_ENABLED
      if (is_ble) {
        // if BLE, update the connection parameters
        esp_ble_conn_update_params_t conn_params = {
            .bda = {0},
            .min_int = ble_min_interval_units, // * 1.25ms
            .max_int = ble_max_interval_units, // * 1.25ms
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

void scan(int seconds) {
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
}

void scan_and_print(std::ostream &out, int seconds) {
  scan(seconds);
  if (devices.empty()) {
    out << "No devices found\n";
    return;
  }
  out << "Devices found:\n";
  for (const auto &[name, result] : devices) {
    out << "  '" << name << "'\n";
  }
}

bool connect(const std::string &remote_name, const std::string &remote_address) {
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
    esp_ble_gap_set_prefer_conn_params(result->bda, ble_min_interval_units, ble_max_interval_units,
                                       0, 400);

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
}

bool scan_and_connect(int num_seconds) {
  scan(num_seconds);
  if (devices.empty()) {
    logger.error("No devices found");
    return false;
  }
  return connect(device_name, device_address);
}

void init_hid() {
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
  esp_nimble_enable(ble_hid_host_task);
#endif // CONFIG_BT_NIMBLE_ENABLED
}

static const std::string ble_min_nvs_key = "ble_min";
static const std::string ble_max_nvs_key = "ble_max";
static const std::string bt_qos_nvs_key = "bt_qos";

void load_nvs(espp::Nvs &nvs) {
  std::error_code ec;
  // - HID Host or PhotoDiode (ADC)
  // - if using HID Host, select the device to connect to
  nvs.get_or_set_var("latency", "use_hid_host", use_hid_host, use_hid_host, ec);
  if (ec) {
    logger.error("Failed to get use_hid_host from NVS: {}", ec.message());
    return;
  }
  logger.info("Loaded use_hid_host: {}", use_hid_host);

  // button press level
  nvs.get_or_set_var("latency", "pressed_level", BUTTON_PRESSED_LEVEL, BUTTON_PRESSED_LEVEL, ec);
  if (ec) {
    logger.error("Failed to get pressed_level from NVS: {}", ec.message());
    return;
  }
  // set RELEASED level based on PRESSED level
  BUTTON_RELEASED_LEVEL = !BUTTON_PRESSED_LEVEL;

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
    // get the BLE connection parameters (min interval)
    nvs.get_or_set_var("latency", ble_min_nvs_key, ble_min_interval_units, ble_min_interval_units,
                       ec);
    if (ec) {
      logger.error("Failed to get ble_min_interval_units from NVS: {}", ec.message());
      return;
    }
    logger.info("Loaded ble_min_interval_units: {} = {} ms", ble_min_interval_units,
                ble_interval_units_to_ms(ble_min_interval_units));
    // get the BLE connection parameters (max interval)
    nvs.get_or_set_var("latency", ble_max_nvs_key, ble_max_interval_units, ble_max_interval_units,
                       ec);
    if (ec) {
      logger.error("Failed to get ble_max_interval_units from NVS: {}", ec.message());
      return;
    }
    logger.info("Loaded ble_max_interval_units: {} = {} ms", ble_max_interval_units,
                ble_interval_units_to_ms(ble_max_interval_units));
    // get the BT connection parameters
    nvs.get_or_set_var("latency", bt_qos_nvs_key, bt_qos_units, bt_qos_units, ec);
    if (ec) {
      logger.error("Failed to get bt_qos_units from NVS: {}", ec.message());
      return;
    }
    logger.info("Loaded bt_qos_units: {} = {} ms", bt_qos_units, bt_qos_units_to_ms(bt_qos_units));
  }
}

void build_menu(std::unique_ptr<cli::Menu> &root_menu, espp::Nvs &nvs) {
  // menu function for reading the current adc value
  root_menu->Insert(
      "adc", [&](std::ostream &out) { out << fmt::format("ADC value: {:.2f} mV\n", get_mv()); },
      "Read the current ADC value");

  // menu function for setting the button output value
  root_menu->Insert(
      "button", {"Button Value (0/1)"},
      [&](std::ostream &out, int value) {
        gpio_set_level(button_pin, value);
        out << "Button set to " << value << "\n";
      },
      "Set the button output value");

  // menu functions for getting / setting the button pressed level
  root_menu->Insert(
      "pressed_level",
      [&](std::ostream &out) { out << "pressed_level: " << BUTTON_PRESSED_LEVEL << "\n"; },
      "Get the current value of pressed_level");
  root_menu->Insert(
      "pressed_level", {"Button Pressed Level (0/1)"},
      [&](std::ostream &out, int value) {
        BUTTON_PRESSED_LEVEL = value;
        BUTTON_RELEASED_LEVEL = !BUTTON_PRESSED_LEVEL;
        std::error_code ec;
        nvs.set_var("latency", "pressed_level", BUTTON_PRESSED_LEVEL, ec);
        if (ec) {
          out << "Failed to set pressed_level: " << ec.message() << "\n";
        } else {
          out << "pressed_level: " << BUTTON_PRESSED_LEVEL << "\n";
        }
      },
      "Set the value of pressed_level");

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

  // add menu functions for getting / setting ble connection parameters
  root_menu->Insert(
      "ble_min_interval",
      [&](std::ostream &out) {
        out << "ble_min_interval_units: " << (int)ble_min_interval_units << " = "
            << (int)ble_interval_units_to_ms(ble_min_interval_units) << " ms\n";
      },
      "Get the current value of ble_min_interval");
  root_menu->Insert(
      "ble_min_interval", {"BLE Min interval (ms)"},
      [&](std::ostream &out, int value) {
        ble_min_interval_units = ble_interval_ms_to_units(value);
        std::error_code ec;
        nvs.set_var("latency", ble_min_nvs_key, ble_min_interval_units, ec);
        if (ec) {
          out << "Failed to set ble_min_interval_units: " << ec.message() << "\n";
        } else {
          out << "ble_min_interval_units: " << (int)ble_min_interval_units << " = "
              << (int)ble_interval_units_to_ms(ble_min_interval_units) << " ms\n";
        }
      },
      "Set the value of ble_min_interval");

  root_menu->Insert(
      "ble_max_interval",
      [&](std::ostream &out) {
        out << "ble_max_interval_units: " << (int)ble_max_interval_units << " = "
            << (int)ble_interval_units_to_ms(ble_max_interval_units) << " ms\n";
      },
      "Get the current value of ble_max_interval");
  root_menu->Insert(
      "ble_max_interval", {"BLE max interval (ms)"},
      [&](std::ostream &out, int value) {
        ble_max_interval_units = ble_interval_ms_to_units(value);
        std::error_code ec;
        nvs.set_var("latency", ble_max_nvs_key, ble_max_interval_units, ec);
        if (ec) {
          out << "Failed to set ble_max_interval_units: " << ec.message() << "\n";
        } else {
          out << "ble_max_interval_units: " << (int)ble_max_interval_units << " = "
              << (int)ble_interval_units_to_ms(ble_max_interval_units) << " ms\n";
        }
      },
      "Set the value of ble_max_interval");

  // add menu functions for getting / setting bt connection parameters
  root_menu->Insert(
      "bt_qos",
      [&](std::ostream &out) {
        out << "bt_qos_units: " << (int)bt_qos_units << " = "
            << (int)bt_qos_units_to_ms(bt_qos_units) << " ms\n";
      },
      "Get the current value of bt_qos");
  root_menu->Insert(
      "bt_qos", {"BT QoS (ms)"},
      [&](std::ostream &out, int value) {
        bt_qos_units = bt_qos_ms_to_units(value);
        std::error_code ec;
        nvs.set_var("latency", bt_qos_nvs_key, bt_qos_units, ec);
        if (ec) {
          out << "Failed to set bt_qos_units: " << ec.message() << "\n";
        } else {
          out << "bt_qos_units: " << (int)bt_qos_units << " = "
              << (int)bt_qos_units_to_ms(bt_qos_units) << " ms\n";
        }
      },
      "Set the value of bt_qos");

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
}
