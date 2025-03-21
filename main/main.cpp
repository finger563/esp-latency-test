#include <chrono>
#include <condition_variable>
#include <string>
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

#include "esp_hid_gap.h"

// logger for the latency test
static espp::Logger logger({.tag = "esp-latency-test", .level = espp::Logger::Verbosity::DEBUG});

// variables loaded from menuconfig
static gpio_num_t button_pin = (gpio_num_t)CONFIG_BUTTON_GPIO;
static gpio_num_t extra_gnd_pin = (gpio_num_t)CONFIG_EXTRA_GND_GPIO;
static int BUTTON_PRESSED_LEVEL = 1;
static int BUTTON_RELEASED_LEVEL = !BUTTON_PRESSED_LEVEL;
static constexpr adc_unit_t ADC_UNIT = CONFIG_SENSOR_ADC_UNIT == 1 ? ADC_UNIT_1 : ADC_UNIT_2;
static constexpr adc_channel_t ADC_CHANNEL = (adc_channel_t)CONFIG_SENSOR_ADC_CHANNEL;
static std::vector<espp::AdcConfig> channels{
    // A0 on QtPy ESP32S3
    {.unit = ADC_UNIT, .channel = ADC_CHANNEL, .attenuation = ADC_ATTEN_DB_6}};
static std::shared_ptr<espp::OneshotAdc> adc{nullptr};

// things we'll load from NVS
static int upper_threshold = 80;
static int lower_threshold = 30;
static bool use_hid_host{false};
static std::string device_name{};
static std::string device_address{};
static bool parse_input{false};    // false for most controllers, true for pro controller and ps5
static uint8_t input_report_id{1}; // 63 for pro controller, 1 for basically everything else
static uint8_t input_report_button_byte_0{
    0}; // 0 for pro controller, 4 for ps5, unneeded for most controllers
static uint8_t input_report_button_byte_1{
    1}; // 1 for pro controller, 5 for ps5, unneeded for most controllers
static uint8_t ble_min_interval_units{12}; // 15ms
static uint8_t ble_max_interval_units{12}; // 15ms
static uint8_t bt_qos_units{0};            // disabled by default

// runtime state for HID mode
static bool is_ble{false};
static std::atomic<bool> connected{false};
static std::mutex mutex;
static std::condition_variable cv;
static size_t devices_len = 0;
static esp_hid_scan_result_t *devices = NULL;

// utility functions
std::string config_to_string();
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
bool connect(const std::string &remote_name, const std::string &remote_address = "");
bool scan_and_connect(int num_seconds = 5);
void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
esp_hid_scan_result_t *get_device_by_name(const std::string &name);
esp_hid_scan_result_t *get_device_by_address(const std::string &address);

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

  // print the configuration
  logger.info("\n{}", config_to_string());

  // make the ADC regardless of whether we're using it or not, esp. because
  // DEBUG_PLOT_ALL uses it
  adc = std::make_shared<espp::OneshotAdc>(espp::OneshotAdc::Config{
      .unit = ADC_UNIT,
      .channels = channels,
  });

  logger.info("Setting up GPIO pins");
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
    logger.warn("DEBUG_PLOT_ALL is not supported with HID Host, will log ADC values instead");
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
         if (state == TestState::BUTTON_PRESSED && mv > upper_threshold) {
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
         if (state == TestState::BUTTON_RELEASED && mv < lower_threshold) {
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

  fmt::print("% time (s), latency (ms), num missed inputs\n");

  bool previous_iteration_failed = false;
  uint32_t num_missed_inputs = 0;

  // Let's actually measure latency instead of debugging / logging
  while (true) {
    // reset the state at the beginning of the loop
    shift = (rand() % MAX_SHIFT_MS) * 1000;
    button_press_start = 0;
    button_release_start = 0;
    static constexpr uint64_t MAX_lATENCY_US = 200 * 1000; // 200ms

    // ensure the button is released if we missed an input / timed out from a
    // previous iteration
    if (previous_iteration_failed) {
      previous_iteration_failed = false;
      gpio_set_level(button_pin, BUTTON_RELEASED_LEVEL);
      // ignore this value, we're just waiting for the button to be released
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait_for(lock, 500ms);
    }

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
        previous_iteration_failed = true;
        num_missed_inputs++;
        continue;
      }
      latency_us = esp_timer_get_time() - button_press_start;
    } else {
      while (true) {
        auto now_us = esp_timer_get_time();
        latency_us = now_us - button_press_start;
        // read the ADC
        if (get_mv() > upper_threshold) {
          break;
        }
        // timeout
        if (latency_us > MAX_lATENCY_US) {
          previous_iteration_failed = true;
          num_missed_inputs++;
          break;
        }
      }
    }

    // log the latency
    fmt::print("{:.3f}, {:.3f}, {}\n", elapsed(), latency_us / 1e3f, num_missed_inputs);

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
        previous_iteration_failed = true;
        num_missed_inputs++;
        continue;
      }
      latency_us = esp_timer_get_time() - button_release_start;
    } else {
      while (true) {
        auto now_us = esp_timer_get_time();
        latency_us = now_us - button_release_start;
        // read the ADC
        if (get_mv() < lower_threshold) {
          break;
        }
        // timeout
        if (latency_us > MAX_lATENCY_US) {
          previous_iteration_failed = true;
          num_missed_inputs++;
          break;
        }
      }
    }

    // log the latency
    fmt::print("{:.3f}, {:.3f}, {}\n", elapsed(), latency_us / 1e3f, num_missed_inputs);
  }

#endif // CONFIG_DEBUG_PLOT_ALL
}

std::string config_to_string() {
  std::string config = "Current configuration:\n"
                       "----------------------\n";
  config += fmt::format("Button pin: {}, extra GND pin: {}\n", (int)button_pin, (int)extra_gnd_pin);
  config += fmt::format("Button pressed level: {}, released level: {}\n", BUTTON_PRESSED_LEVEL,
                        BUTTON_RELEASED_LEVEL);
  config += fmt::format("Use HID Host: {}\n", use_hid_host);
  if (use_hid_host) {
    config += fmt::format("\tDevice name: '{}'\n", device_name);
    config += fmt::format("\tDevice address: '{}'\n", device_address);
    config += fmt::format("\tParse input: {}\n", parse_input);
    if (parse_input) {
      config += fmt::format("\t\tInput report ID: {}\n", (int)input_report_id);
      config += fmt::format("\t\tInput report button bytes: {}, {}\n",
                            (int)input_report_button_byte_0, (int)input_report_button_byte_1);
    }
    config += fmt::format("\tBLE min interval: {} = {} ms\n", ble_min_interval_units,
                          ble_interval_units_to_ms(ble_min_interval_units));
    config += fmt::format("\tBLE max interval: {} = {} ms\n", ble_max_interval_units,
                          ble_interval_units_to_ms(ble_max_interval_units));
    config += fmt::format("\tBT QoS: {} = {} ms\n", bt_qos_units, bt_qos_units_to_ms(bt_qos_units));
  } else {
    config += fmt::format("ADC unit: {}, channel: {}\n", ADC_UNIT, ADC_CHANNEL);
    config +=
        fmt::format("Upper threshold: {}, lower threshold: {}\n", upper_threshold, lower_threshold);
  }
  return config;
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
      const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
      fmt::print("Connected to device, addr: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}\n",
                 ESP_BD_ADDR_HEX(bda));
      connected = true;
#if CONFIG_BT_CLASSIC_ENABLED
      if (!is_ble && bt_qos_units != 0) {
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
            .latency = 4,                      // 4 packets allowed to be skipped
            .timeout = 100,                    // 1000ms timeout
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
    // pull out the button state from the HID report
    uint16_t button_state = param->input.data[input_report_button_byte_0] |
                            (param->input.data[input_report_button_byte_1] << 8);
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
  if (devices) {
    esp_hid_scan_results_free(devices);
    devices = nullptr;
    devices_len = 0;
  }
  esp_hid_scan(seconds, &devices_len, &devices);
  // loop through the devices and print them
  if (devices_len > 0) {
    esp_hid_scan_result_t *r = devices;
    while (r) {
      bool has_name = r->name != nullptr;
      std::string address =
          fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", ESP_BD_ADDR_HEX(r->bda));
      std::string name = has_name ? std::string(r->name) : "Unknown";
      logger.info("Found device: '{}', addr: {}", name, address);
      r = r->next;
    }
  } else {
    logger.info("No devices found");
  }
}

bool connect(const std::string &remote_name, const std::string &remote_address) {
  auto device_by_name = get_device_by_name(remote_name);
  auto device_by_address = get_device_by_address(remote_address);
  if (device_by_name == nullptr && device_by_address == nullptr) {
    logger.error("Device '{}' not found", remote_name);
    return false;
  }
  esp_hid_scan_result_t *device = device_by_name ? device_by_name : device_by_address;
  logger.info("Connecting to device '{}'", remote_name);
  logger.info("          at address {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
              ESP_BD_ADDR_HEX(device->bda));
#if CONFIG_BT_CLASSIC_ENABLED && CONFIG_BT_BLE_ENABLED
  if (device->transport == ESP_HID_TRANSPORT_BLE) {
#endif // CONFIG_BT_CLASSIC_ENABLED
    is_ble = true;
    logger.info("          using BLE, addr_type: {}", (int)device->ble.addr_type);

    // if BLE, update the connection parameters
    esp_ble_gap_set_prefer_conn_params(device->bda, ble_min_interval_units, ble_max_interval_units,
                                       0, 400);

    // now connect
    esp_hidh_dev_open(device->bda, ESP_HID_TRANSPORT_BLE, device->ble.addr_type);
#if CONFIG_BT_CLASSIC_ENABLED && CONFIG_BT_BLE_ENABLED
  } else {
    is_ble = false;
    logger.info("          using BT");
    esp_hidh_dev_open(device->bda, ESP_HID_TRANSPORT_BT, 0);
  }
#endif // CONFIG_BT_CLASSIC_ENABLED
  return true;
}

bool scan_and_connect(int num_seconds) {
  scan(num_seconds);
  if (devices_len == 0) {
    logger.error("No devices found");
    return false;
  }
  if (connected) {
    return true;
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
      .event_stack_size = 6192,
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

  // adc thresholds
  nvs.get_or_set_var("latency", "upper_threshold", upper_threshold, upper_threshold, ec);
  if (ec) {
    logger.error("Failed to get upper_threshold from NVS: {}", ec.message());
    return;
  }
  logger.info("Loaded upper_threshold: {}", upper_threshold);

  nvs.get_or_set_var("latency", "lower_threshold", lower_threshold, lower_threshold, ec);
  if (ec) {
    logger.error("Failed to get lower_threshold from NVS: {}", ec.message());
    return;
  }
  logger.info("Loaded lower_threshold: {}", lower_threshold);

  // button gpio
  nvs.get_or_set_var("latency", "button_pin", (int &)button_pin, (int)button_pin, ec);
  if (ec) {
    logger.error("Failed to get button_pin from NVS: {}", ec.message());
    return;
  }
  logger.info("Loaded button_pin: {}", (int)button_pin);

  // extra gnd gpio
  nvs.get_or_set_var("latency", "extra_gnd_pin", (int &)extra_gnd_pin, (int)extra_gnd_pin, ec);
  if (ec) {
    logger.error("Failed to get extra_gnd_pin from NVS: {}", ec.message());
    return;
  }
  logger.info("Loaded extra_gnd_pin: {}", (int)extra_gnd_pin);

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
    // get the input report button byte 0
    nvs.get_or_set_var("latency", "rp_byte0", input_report_button_byte_0,
                       input_report_button_byte_0, ec);
    if (ec) {
      logger.error("Failed to get input_report_button_byte_0 from NVS: {}", ec.message());
      return;
    }
    logger.info("Loaded input_report_button_byte_0: {}", input_report_button_byte_0);
    // get the input report button byte 1
    nvs.get_or_set_var("latency", "rp_byte1", input_report_button_byte_1,
                       input_report_button_byte_1, ec);
    if (ec) {
      logger.error("Failed to get input_report_button_byte_1 from NVS: {}", ec.message());
      return;
    }
    logger.info("Loaded input_report_button_byte_1: {}", input_report_button_byte_1);
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
  // menu function for printing current configuration
  root_menu->Insert(
      "config", [&](std::ostream &out) { out << config_to_string(); },
      "Print the current configuration");

  // menu function for reading the current adc value
  root_menu->Insert(
      "adc", [&](std::ostream &out) { out << fmt::format("ADC value: {:.2f} mV\n", get_mv()); },
      "Read the current ADC value");

  // menu function for setting the button pin
  root_menu->Insert(
      "button_pin", [&](std::ostream &out) { out << "button_pin: " << (int)button_pin << "\n"; },
      "Get the current value of button_pin");

  root_menu->Insert(
      "button_pin", {"Button Pin (0-39)"},
      [&](std::ostream &out, int value) {
        button_pin = (gpio_num_t)value;
        std::error_code ec;
        nvs.set_var("latency", "button_pin", (int)button_pin, ec);
        if (ec) {
          out << "Failed to set button_pin: " << ec.message() << "\n";
        } else {
          out << "button_pin: " << (int)button_pin << "\n";
        }
        // now ensure the pin is set to output
        gpio_set_direction(button_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(button_pin, BUTTON_RELEASED_LEVEL);
      },
      "Set the value of button_pin");

  // menu function for setting the extra gnd pin
  root_menu->Insert(
      "extra_gnd_pin",
      [&](std::ostream &out) { out << "extra_gnd_pin: " << (int)extra_gnd_pin << "\n"; },
      "Get the current value of extra_gnd_pin");

  root_menu->Insert(
      "extra_gnd_pin", {"Extra GND Pin (0-39)"},
      [&](std::ostream &out, int value) {
        extra_gnd_pin = (gpio_num_t)value;
        std::error_code ec;
        nvs.set_var("latency", "extra_gnd_pin", (int)extra_gnd_pin, ec);
        if (ec) {
          out << "Failed to set extra_gnd_pin: " << ec.message() << "\n";
        } else {
          out << "extra_gnd_pin: " << (int)extra_gnd_pin << "\n";
        }
        // now ensure the pin is set to output
        gpio_set_direction(extra_gnd_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(extra_gnd_pin, 0);
      },
      "Set the value of extra_gnd_pin");

  // menu function for setting the button output value
  root_menu->Insert(
      "button_out", {"Button Value (0/1)"},
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

  // menu functions for getting / setting the upper threshold
  root_menu->Insert(
      "upper_threshold",
      [&](std::ostream &out) { out << "upper_threshold: " << upper_threshold << "\n"; },
      "Get the current value of upper_threshold");
  root_menu->Insert(
      "upper_threshold", {"Upper Threshold (0-4095 mV)"},
      [&](std::ostream &out, int value) {
        upper_threshold = value;
        std::error_code ec;
        nvs.set_var("latency", "upper_threshold", upper_threshold, ec);
        if (ec) {
          out << "Failed to set upper_threshold: " << ec.message() << "\n";
        } else {
          out << "upper_threshold: " << upper_threshold << "\n";
        }
      },
      "Set the value of upper_threshold");

  // menu functions for getting / setting the lower threshold
  root_menu->Insert(
      "lower_threshold",
      [&](std::ostream &out) { out << "lower_threshold: " << lower_threshold << "\n"; },
      "Get the current value of lower_threshold");
  root_menu->Insert(
      "lower_threshold", {"Lower Threshold (0-4095 mV)"},
      [&](std::ostream &out, int value) {
        lower_threshold = value;
        std::error_code ec;
        nvs.set_var("latency", "lower_threshold", lower_threshold, ec);
        if (ec) {
          out << "Failed to set lower_threshold: " << ec.message() << "\n";
        } else {
          out << "lower_threshold: " << lower_threshold << "\n";
        }
      },
      "Set the value of lower_threshold");

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

  // add menu functions for getting / setting input_report_button_byte_0
  root_menu->Insert(
      "rp_byte0",
      [&](std::ostream &out) {
        out << "input_report_button_byte_0: " << (int)input_report_button_byte_0 << "\n";
      },
      "Get the current value of input_report_button_byte_0");
  root_menu->Insert(
      "rp_byte0", {"Input Report Button Byte 0 (0-255)"},
      [&](std::ostream &out, int value) {
        input_report_button_byte_0 = value;
        std::error_code ec;
        nvs.set_var("latency", "rp_byte0", input_report_button_byte_0, ec);
        if (ec) {
          out << "Failed to set input_report_button_byte_0: " << ec.message() << "\n";
        } else {
          out << "input_report_button_byte_0: " << (int)input_report_button_byte_0 << "\n";
        }
      },
      "Set the value of input_report_button_byte_0");

  // add menu functions for getting / setting input_report_button_byte_1
  root_menu->Insert(
      "rp_byte1",
      [&](std::ostream &out) {
        out << "input_report_button_byte_1: " << (int)input_report_button_byte_1 << "\n";
      },
      "Get the current value of input_report_button_byte_1");
  root_menu->Insert(
      "rp_byte1", {"Input Report Button Byte 1 (0-255)"},
      [&](std::ostream &out, int value) {
        input_report_button_byte_1 = value;
        std::error_code ec;
        nvs.set_var("latency", "rp_byte1", input_report_button_byte_1, ec);
        if (ec) {
          out << "Failed to set input_report_button_byte_1: " << ec.message() << "\n";
        } else {
          out << "input_report_button_byte_1: " << (int)input_report_button_byte_1 << "\n";
        }
      },
      "Set the value of input_report_button_byte_1");

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
        auto device = get_device_by_name(device_name);
        if (device != nullptr) {
          device_address = fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                                       ESP_BD_ADDR_HEX(device->bda));
          out << "device_address: " << device_address << "\n";
          nvs.set_var("latency", "device_address", (const std::string)device_address, ec);
        }
      },
      "Set the value of device_name");

  // add menu functions for getting / setting device_address
  root_menu->Insert(
      "device_address",
      [&](std::ostream &out) { out << "device_address: " << device_address << "\n"; },
      "Get the current value of device_address");
  root_menu->Insert(
      "device_address", {"Device Address"},
      [&](std::ostream &out, const std::string &value) {
        device_address = value;
        std::error_code ec;
        nvs.set_var("latency", "device_address", (const std::string)device_address, ec);
        if (ec) {
          out << "Failed to set device_address: " << ec.message() << "\n";
        } else {
          out << "device_address: " << device_address << "\n";
        }
      },
      "Set the value of device_address");

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
      "scan", [&](std::ostream &out) { scan(); }, "Scan for HID devices (5 seconds)");

  root_menu->Insert(
      "connect",
      [&](std::ostream &out) {
        if (connect(device_name, device_address)) {
          out << "Connecting to device '" << device_name << "'\n";
        } else {
          out << "Failed to connect to device\n";
        }
      },
      "Connect to the device with the given name");
  root_menu->Insert(
      "connect", {"device_name"},
      [&](std::ostream &out, const std::string &name) {
        if (connect(name)) {
          out << "Connecting to device '" << name << "'\n";
        } else {
          out << "Failed to connect to device\n";
        }
      },
      "Connect to the device with the given name");

  root_menu->Insert(
      "scan", {"seconds"}, [&](std::ostream &out, int seconds) { scan(seconds); },
      "Scan for HID devices");
}

static bool compare_string(const std::string &a, const std::string &b) {
  return strcmp(a.c_str(), b.c_str()) == 0;
}

static bool is_substring(const std::string &a, const std::string &b) {
  return strcmp(a.c_str(), b.c_str()) == 0;
}

esp_hid_scan_result_t *get_device_by_name(const std::string &name) {
  if (devices == nullptr) {
    logger.error("No devices found, run 'scan' first");
    return nullptr;
  }
  // go through the devices and find the device with the name
  esp_hid_scan_result_t *r = devices;
  while (r) {
    std::string this_address =
        fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", ESP_BD_ADDR_HEX(r->bda));
    logger.info("Checking name '{}' against name of device with address '{}'", name, this_address);
    if (r->name == nullptr) {
      logger.warn("Device has no name, skipping");
      r = r->next;
      continue;
    }
    // see if the provided name is a substring of the device name
    auto this_name = std::string(r->name);
    logger.info("Checking name '{}' against '{}'", name, this_name);
    if (this_name.contains(name) || name.contains(this_name) || this_name == name ||
        compare_string(name, this_name) || is_substring(name, this_name)) {
      logger.info("Matched name '{}' to '{}'", name, this_name);
      return r;
    }
    r = r->next;
  }
  // we didn't find the device, return nullptr
  return nullptr;
}

esp_hid_scan_result_t *get_device_by_address(const std::string &address) {
  if (devices == nullptr) {
    logger.error("No devices found, run 'scan' first");
    return nullptr;
  }
  // go through the devices and find the device with the address
  esp_hid_scan_result_t *r = devices;
  while (r) {
    std::string this_address =
        fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", ESP_BD_ADDR_HEX(r->bda));
    logger.info("Checking address '{}' against '{}'", address, this_address);
    if (this_address == address || this_address.contains(address) ||
        compare_string(address, this_address) || is_substring(address, this_address)) {
      logger.info("Matched address '{}' to '{}'", address, this_address);
      return r;
    }
    r = r->next;
  }
  // we didn't find the device, return nullptr
  return nullptr;
}
