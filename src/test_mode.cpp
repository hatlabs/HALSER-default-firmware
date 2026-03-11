#include "test_mode.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <driver/twai.h>

#include "halser_const.h"

static String input_buffer;
static bool twai_installed = false;
static Adafruit_NeoPixel* led = nullptr;

static bool twai_init() {
  if (twai_installed) return true;

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(kCANTxPin, kCANRxPin, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }
  twai_installed = true;
  return true;
}

static void twai_deinit() {
  if (!twai_installed) return;
  twai_stop();
  twai_driver_uninstall();
  twai_installed = false;
}

/// CAN bus ping-pong test. Sends a frame with ID 0x123 and waits for the
/// test jig MCU1 to respond with a frame with ID 0x456 (inverted data).
static void handle_can_test() {
  if (!twai_init()) {
    Serial.println("=== ERROR: CAN TWAI_INIT_FAIL");
    return;
  }

  // Flush any stale frames
  twai_message_t flush_msg;
  while (twai_receive(&flush_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
  }

  twai_message_t tx_msg = {};
  tx_msg.extd = 1;
  tx_msg.identifier = kCANTestTxId;
  tx_msg.data_length_code = 8;
  uint8_t tx_data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  memcpy(tx_msg.data, tx_data, 8);

  if (twai_transmit(&tx_msg, pdMS_TO_TICKS(1000)) != ESP_OK) {
    Serial.println("=== ERROR: CAN TX_FAIL");
    twai_deinit();
    return;
  }

  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(2000)) != ESP_OK) {
    Serial.println("=== ERROR: CAN RX_TIMEOUT");
    twai_deinit();
    return;
  }

  if (!rx_msg.extd || rx_msg.identifier != kCANTestRxId) {
    Serial.printf("=== ERROR: CAN RX_WRONG_ID 0x%lX\n", rx_msg.identifier);
    twai_deinit();
    return;
  }

  uint8_t expected[] = {0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8, 0xF7};
  if (memcmp(rx_msg.data, expected, 8) != 0) {
    Serial.println("=== ERROR: CAN RX_DATA_MISMATCH");
    twai_deinit();
    return;
  }

  Serial.println("=== OK: CAN_TEST PASS");
  twai_deinit();
}

/// Serial loopback test. Sends a pattern on UART1 TX and reads it back on RX.
static void handle_serial_loopback(const char* label) {
  Serial1.begin(kDefaultNMEA0183Baud, SERIAL_8N1, kUART1RxPin, kUART1TxPin);

  while (Serial1.available()) Serial1.read();

  const uint8_t pattern[] = {0x55, 0xAA, 0x0F, 0xF0, 0x12, 0x34, 0x56, 0x78};
  constexpr size_t len = sizeof(pattern);

  Serial1.write(pattern, len);
  Serial1.flush();

  uint8_t rx_buf[len] = {};
  size_t rx_count = 0;
  unsigned long deadline = millis() + 200;

  while (rx_count < len && millis() < deadline) {
    if (Serial1.available()) {
      rx_buf[rx_count++] = Serial1.read();
    }
  }

  Serial1.end();

  if (rx_count != len) {
    Serial.printf("=== ERROR: %s RX_SHORT %u/%u\n", label,
                  (unsigned)rx_count, (unsigned)len);
    return;
  }

  if (memcmp(rx_buf, pattern, len) != 0) {
    Serial.printf("=== ERROR: %s DATA_MISMATCH\n", label);
    return;
  }

  Serial.printf("=== OK: %s PASS\n", label);
}

/// GPIO output command: "GPIO_OUT <pin> <0|1>"
static void handle_gpio_out(const String& args) {
  int pin = -1, value = -1;
  if (sscanf(args.c_str(), "%d %d", &pin, &value) != 2 || pin < 0 ||
      pin > 21) {
    Serial.println("=== ERROR: GPIO_OUT BAD_ARGS");
    return;
  }
  pinMode(pin, OUTPUT);
  digitalWrite(pin, value ? HIGH : LOW);
  Serial.printf("=== OK: GPIO %d=%d\n", pin, value);
}

/// GPIO release command: "GPIO_RELEASE <pin>"
static void handle_gpio_release(const String& args) {
  int pin = -1;
  if (sscanf(args.c_str(), "%d", &pin) != 1 || pin < 0 || pin > 21) {
    Serial.println("=== ERROR: GPIO_RELEASE BAD_ARGS");
    return;
  }
  pinMode(pin, INPUT);
  Serial.printf("=== OK: GPIO %d RELEASED\n", pin);
}

/// Hall effect sensor test. Reads GPIO 1 with pull-down; the KTH1601TL
/// push-pull sensor drives HIGH when no magnet is present.
static void handle_hall_test() {
  pinMode(kHallSensorPin, INPUT_PULLDOWN);
  delay(10);
  int val = digitalRead(kHallSensorPin);
  pinMode(kHallSensorPin, INPUT);

  if (val == HIGH) {
    Serial.println("=== OK: HALL HIGH");
  } else {
    Serial.println("=== ERROR: HALL LOW");
  }
}

static void handle_command(const String& cmd) {
  String trimmed = cmd;
  trimmed.trim();

  if (trimmed == "PING") {
    Serial.println("=== OK: PONG");
  } else if (trimmed == "CAN_TEST") {
    handle_can_test();
  } else if (trimmed == "RS485_TEST") {
    handle_serial_loopback("RS485");
  } else if (trimmed == "RS232_TEST") {
    handle_serial_loopback("RS232");
  } else if (trimmed == "UART_TEST") {
    handle_serial_loopback("UART");
  } else if (trimmed.startsWith("GPIO_OUT ")) {
    handle_gpio_out(trimmed.substring(9));
  } else if (trimmed.startsWith("GPIO_RELEASE ")) {
    handle_gpio_release(trimmed.substring(13));
  } else if (trimmed == "HALL_TEST") {
    handle_hall_test();
  } else {
    Serial.print("=== ERROR: UNKNOWN_COMMAND ");
    Serial.println(trimmed);
  }
}

void run_test_mode() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  input_buffer.reserve(128);

  led = new Adafruit_NeoPixel(1, kRGBLEDPin, NEO_GRB + NEO_KHZ800);
  led->begin();
  led->setBrightness(30);

  while (true) {
    // Process serial commands
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (input_buffer.length() > 0) {
          handle_command(input_buffer);
          input_buffer = "";
        }
      } else if (input_buffer.length() < 128) {
        input_buffer += c;
      }
    }

    // Rainbow LED animation at 1 Hz
    uint16_t hue = (uint16_t)((millis() % 1000) * 65536UL / 1000);
    led->setPixelColor(0, led->ColorHSV(hue));
    led->show();
  }
}
