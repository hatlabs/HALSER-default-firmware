#include "nmea_gateway.h"

#include <Adafruit_NeoPixel.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <esp_mac.h>

#include "halser_const.h"
#include "n2k_senders.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/gnss_sentence_parser.h"
#include "sensesp_nmea0183/sentence_parser/navigation_sentence_parser.h"
#include "sensesp_nmea0183/sentence_parser/wind_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static tNMEA2000* nmea2000 = nullptr;
static Adafruit_NeoPixel* led = nullptr;
static unsigned long led_off_until = 0;

static uint32_t GetBoardSerialNumber() {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  return (mac[3] << 16) | (mac[4] << 8) | mac[5];
}

/// Format the WiFi MAC address as a serial number string for N2K product info.
static String GetProductSerialNumber() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X%02X%02X%02X%02X%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

void run_nmea_gateway() {
  Serial.setTxTimeoutMs(0);
  SetupLogging(ESP_LOG_DEBUG);

  // SensESP application
  SensESPAppBuilder builder;
  auto sensesp_app = (&builder)
                         ->set_hostname("halser")
                         ->set_button_pin(kButtonPin)
                         ->enable_ota("halser")
                         ->get_app();

  // RGB LED for activity indication
  led = new Adafruit_NeoPixel(1, kRGBLEDPin, NEO_GRB + NEO_KHZ800);
  led->begin();
  led->setBrightness(30);

  // NMEA 0183 bit rate (configurable via web UI, requires restart)
  auto bit_rate = std::make_shared<PersistingObservableValue<int>>(
      kDefaultNMEA0183Baud, "/serial/bit_rate");
  ConfigItem(bit_rate)
      ->set_title("NMEA 0183 Bit Rate")
      ->set_description("Serial bit rate for the NMEA 0183 input (default 4800)")
      ->set_sort_order(100)
      ->set_requires_restart(true)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Bit rate (bit/s)","type":"integer"}}})schema");

  // UART1 for NMEA 0183 input
  Serial1.begin(bit_rate->get(), SERIAL_8N1, kUART1RxPin, kUART1TxPin);

  // NMEA 2000 (CAN bus via TWAI)
  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);
  auto product_serial = GetProductSerialNumber();
  nmea2000->SetProductInformation(
      product_serial.c_str(),     // Serial number (from WiFi MAC)
      100,                        // Product code
      "HALSER NMEA 0183-N2K GW", // Model ID
      "1.0.0",                    // Software version
      "1.0.0"                     // Model version
  );
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),
      kDeviceFunction,    // 130 = PC Gateway
      kDeviceClass,       // 25 = Inter/Intranetwork Device
      kManufacturerCode   // 2046 = Hat Labs
  );
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 73);
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // Process N2K messages (address claim, heartbeat, etc.)
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  // NMEA 0183 I/O task (runs on dedicated FreeRTOS task)
  auto nmea0183_io = new NMEA0183IOTask(&Serial1);

  // --- Sentence Parsers ---

  auto gga_parser = new GGASentenceParser(&nmea0183_io->parser_);
  auto rmc_parser = new RMCSentenceParser(&nmea0183_io->parser_);
  auto vtg_parser = new VTGSentenceParser(&nmea0183_io->parser_);
  auto hdg_parser = new HDGSentenceParser(&nmea0183_io->parser_);
  auto vhw_parser = new VHWSentenceParser(&nmea0183_io->parser_);
  auto dpt_parser = new DPTSentenceParser(&nmea0183_io->parser_);
  auto mwv_parser = new WIMWVSentenceParser(&nmea0183_io->parser_);

  // --- N2K Senders ---

  auto gnss_sender = new halser::N2kGNSSSender(nmea2000);
  auto cogsog_sender = new halser::N2kCOGSOGSender(nmea2000);
  auto heading_sender = new halser::N2kHeadingSender(nmea2000);
  auto speed_sender = new halser::N2kBoatSpeedSender(nmea2000);
  auto depth_sender = new halser::N2kDepthSender(nmea2000);
  auto wind_sender = new halser::N2kWindSender(nmea2000);

  // --- Wiring: Parser outputs → N2K senders ---

  // GGA → GNSS position
  gga_parser->position_.connect_to(
      new LambdaConsumer<Position>([gnss_sender](Position pos) {
        gnss_sender->latitude_.update(pos.latitude);
        gnss_sender->longitude_.update(pos.longitude);
        gnss_sender->altitude_.update(pos.altitude);
      }));
  gga_parser->num_satellites_.connect_to(
      new LambdaConsumer<int>([gnss_sender](int n) {
        gnss_sender->num_satellites_.update(n);
      }));
  gga_parser->horizontal_dilution_.connect_to(
      new LambdaConsumer<float>([gnss_sender](float hdop) {
        gnss_sender->hdop_.update(hdop);
      }));
  gga_parser->geoidal_separation_.connect_to(
      new LambdaConsumer<float>([gnss_sender](float sep) {
        gnss_sender->geoidal_separation_.update(sep);
      }));
  gga_parser->quality_.connect_to(
      new LambdaConsumer<int>([gnss_sender](int q) {
        gnss_sender->gnss_quality_.update(q);
      }));

  // RMC → GNSS time + COG/SOG
  rmc_parser->datetime_.connect_to(
      new LambdaConsumer<time_t>([gnss_sender](time_t t) {
        gnss_sender->datetime_.update(t);
      }));
  rmc_parser->true_course_.connect_to(
      new LambdaConsumer<float>([cogsog_sender](float cog) {
        cogsog_sender->cog_.update(cog);
      }));
  rmc_parser->speed_.connect_to(
      new LambdaConsumer<float>([cogsog_sender](float sog) {
        cogsog_sender->sog_.update(sog);
      }));

  // VTG → COG/SOG (alternative source)
  vtg_parser->true_course_.connect_to(
      new LambdaConsumer<float>([cogsog_sender](float cog) {
        cogsog_sender->cog_.update(cog);
      }));
  vtg_parser->speed_.connect_to(
      new LambdaConsumer<float>([cogsog_sender](float sog) {
        cogsog_sender->sog_.update(sog);
      }));

  // HDG → Heading
  hdg_parser->magnetic_heading_.connect_to(
      new LambdaConsumer<float>([heading_sender](float h) {
        heading_sender->heading_.update(h);
      }));
  hdg_parser->deviation_.connect_to(
      new LambdaConsumer<float>([heading_sender](float d) {
        heading_sender->deviation_.update(d);
      }));
  hdg_parser->variation_.connect_to(
      new LambdaConsumer<float>([heading_sender](float v) {
        heading_sender->variation_.update(v);
      }));

  // VHW → Boat speed
  vhw_parser->water_speed_.connect_to(
      new LambdaConsumer<float>([speed_sender](float s) {
        speed_sender->water_speed_.update(s);
      }));

  // DPT → Water depth
  dpt_parser->depth_.connect_to(
      new LambdaConsumer<float>([depth_sender](float d) {
        depth_sender->depth_.update(d);
      }));
  dpt_parser->offset_.connect_to(
      new LambdaConsumer<float>([depth_sender](float o) {
        depth_sender->offset_.update(o);
      }));

  // MWV → Wind
  mwv_parser->apparent_wind_speed_.connect_to(
      new LambdaConsumer<float>([wind_sender](float s) {
        wind_sender->wind_speed_.update(s);
      }));
  mwv_parser->apparent_wind_angle_.connect_to(
      new LambdaConsumer<float>([wind_sender](float a) {
        wind_sender->wind_angle_.update(a);
      }));

  // --- Periodic N2K transmission ---

  event_loop()->onRepeat(100, [heading_sender]() { heading_sender->send(); });
  event_loop()->onRepeat(250, [cogsog_sender]() { cogsog_sender->send(); });
  event_loop()->onRepeat(1000, [gnss_sender]() { gnss_sender->send(); });
  event_loop()->onRepeat(1000, [speed_sender]() { speed_sender->send(); });
  event_loop()->onRepeat(1000, [depth_sender]() { depth_sender->send(); });
  event_loop()->onRepeat(1000, [wind_sender]() { wind_sender->send(); });

  // --- LED: blink off briefly on each received NMEA 0183 sentence ---

  nmea0183_io->sentence_filter_queue_->connect_to(
      new LambdaConsumer<String>([](const String&) {
        led_off_until = millis() + 50;
      }));

  // --- LED animation + main loop ---

  event_loop()->onRepeat(10, []() {
    if (millis() < led_off_until) {
      led->setPixelColor(0, 0);
    } else {
      uint16_t hue = (uint16_t)((millis() % 1000) * 65536UL / 1000);
      led->setPixelColor(0, led->ColorHSV(hue));
    }
    led->show();
  });

  while (true) {
    event_loop()->tick();
  }
}
