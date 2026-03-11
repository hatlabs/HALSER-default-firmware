#ifndef HALSER_SRC_HALSER_CONST_H_
#define HALSER_SRC_HALSER_CONST_H_

#include <driver/gpio.h>

// ESP32-C3 pin assignments for HALSER board
constexpr int kTestJigPin = 0;
constexpr int kHallSensorPin = 1;
constexpr gpio_num_t kUART1TxPin = GPIO_NUM_2;
constexpr gpio_num_t kUART1RxPin = GPIO_NUM_3;
constexpr gpio_num_t kCANTxPin = GPIO_NUM_4;
constexpr gpio_num_t kCANRxPin = GPIO_NUM_5;
constexpr int kI2CSDAPin = 6;
constexpr int kI2CSCLPin = 7;
constexpr int kRGBLEDPin = 8;
constexpr int kButtonPin = 9;
constexpr int kOneWirePin = 10;

constexpr int kDefaultNMEA0183Baud = 4800;

// CAN test frame IDs (extended 29-bit)
constexpr uint32_t kCANTestTxId = 0x123;
constexpr uint32_t kCANTestRxId = 0x456;

// NMEA 2000 device identity
constexpr uint16_t kManufacturerCode = 2046;  // Hat Labs
constexpr uint8_t kDeviceFunction = 130;      // PC Gateway
constexpr uint8_t kDeviceClass = 25;          // Inter/Intranetwork Device

#endif  // HALSER_SRC_HALSER_CONST_H_
