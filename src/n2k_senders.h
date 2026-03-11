#ifndef HALSER_SRC_N2K_SENDERS_H_
#define HALSER_SRC_N2K_SENDERS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

namespace halser {

/// Tracks a value with automatic expiry for N2K message building.
/// When the value hasn't been updated within max_age_ms, to_n2k() returns
/// N2kDoubleNA instead of a stale value.
///
/// Thread safety: all update() and is_valid()/to_n2k() calls must occur on
/// the same thread (the main SensESP event loop). The NMEA0183IOTask marshals
/// parsed values back to the main loop via TaskQueueProducer, so this is safe
/// in the current architecture.
template <typename T>
class ExpiringValue {
 public:
  ExpiringValue(unsigned long max_age_ms) : max_age_(max_age_ms) {}

  void update(const T& value) {
    value_ = value;
    last_update_ = millis();
  }

  bool is_valid() const {
    return last_update_ > 0 && (millis() - last_update_) < max_age_;
  }

  const T& value() const { return value_; }

  double to_n2k() const {
    return is_valid() ? static_cast<double>(value_) : N2kDoubleNA;
  }

 private:
  T value_{};
  unsigned long last_update_ = 0;
  unsigned long max_age_;
};

/// Map GGA quality indicator (0-8) to N2K GNSS method enum.
/// Values 0-5 map directly; 6-8 (estimated/manual/simulator) map to GNSSfix.
inline tN2kGNSSmethod GGAQualityToN2kMethod(int quality) {
  if (quality >= 0 && quality <= 5) {
    return static_cast<tN2kGNSSmethod>(quality);
  }
  if (quality >= 6 && quality <= 8) {
    return N2kGNSSm_GNSSfix;
  }
  return N2kGNSSm_Unavailable;
}

/// PGN 129029 — GNSS Position Data (1000ms)
class N2kGNSSSender {
 public:
  N2kGNSSSender(tNMEA2000* nmea2000, unsigned long expiry = 5000)
      : nmea2000_(nmea2000),
        latitude_(expiry),
        longitude_(expiry),
        altitude_(expiry),
        num_satellites_(expiry),
        hdop_(expiry),
        geoidal_separation_(expiry),
        datetime_(expiry),
        gnss_quality_(expiry) {}

  bool has_data() const {
    return latitude_.is_valid() || longitude_.is_valid();
  }

  void send() {
    if (!has_data()) return;

    uint16_t days_since_1970 = N2kUInt16NA;
    double seconds_since_midnight = N2kDoubleNA;

    if (datetime_.is_valid()) {
      time_t t = static_cast<time_t>(datetime_.value());
      days_since_1970 = t / 86400;
      seconds_since_midnight = t % 86400;
    }

    int n_sats = num_satellites_.is_valid() ? num_satellites_.value() : 0xff;
    tN2kGNSSmethod method = gnss_quality_.is_valid()
                                ? GGAQualityToN2kMethod(gnss_quality_.value())
                                : N2kGNSSm_Unavailable;

    tN2kMsg msg;
    SetN2kGNSS(msg, 0xff, days_since_1970, seconds_since_midnight,
               latitude_.to_n2k(), longitude_.to_n2k(), altitude_.to_n2k(),
               N2kGNSSt_GPS, method, n_sats, hdop_.to_n2k(),
               N2kDoubleNA, geoidal_separation_.to_n2k());
    nmea2000_->SendMsg(msg);
  }

  ExpiringValue<double> latitude_;
  ExpiringValue<double> longitude_;
  ExpiringValue<double> altitude_;
  ExpiringValue<int> num_satellites_;
  ExpiringValue<float> hdop_;
  ExpiringValue<float> geoidal_separation_;
  ExpiringValue<time_t> datetime_;
  ExpiringValue<int> gnss_quality_;

 private:
  tNMEA2000* nmea2000_;
};

/// PGN 129026 — COG & SOG, Rapid Update (250ms)
class N2kCOGSOGSender {
 public:
  N2kCOGSOGSender(tNMEA2000* nmea2000, unsigned long expiry = 5000)
      : nmea2000_(nmea2000), cog_(expiry), sog_(expiry) {}

  bool has_data() const { return cog_.is_valid() || sog_.is_valid(); }

  void send() {
    if (!has_data()) return;
    tN2kMsg msg;
    SetN2kCOGSOGRapid(msg, 0xff, N2khr_true, cog_.to_n2k(), sog_.to_n2k());
    nmea2000_->SendMsg(msg);
  }

  ExpiringValue<float> cog_;
  ExpiringValue<float> sog_;

 private:
  tNMEA2000* nmea2000_;
};

/// PGN 127250 — Vessel Heading (100ms)
class N2kHeadingSender {
 public:
  N2kHeadingSender(tNMEA2000* nmea2000, unsigned long expiry = 5000)
      : nmea2000_(nmea2000),
        heading_(expiry),
        deviation_(expiry),
        variation_(expiry) {}

  bool has_data() const { return heading_.is_valid(); }

  void send() {
    if (!has_data()) return;
    tN2kMsg msg;
    SetN2kPGN127250(msg, 0xff, heading_.to_n2k(), deviation_.to_n2k(),
                    variation_.to_n2k(), N2khr_magnetic);
    nmea2000_->SendMsg(msg);
  }

  ExpiringValue<float> heading_;
  ExpiringValue<float> deviation_;
  ExpiringValue<float> variation_;

 private:
  tNMEA2000* nmea2000_;
};

/// PGN 128259 — Speed, Water Referenced (1000ms)
class N2kBoatSpeedSender {
 public:
  N2kBoatSpeedSender(tNMEA2000* nmea2000, unsigned long expiry = 5000)
      : nmea2000_(nmea2000), water_speed_(expiry) {}

  bool has_data() const { return water_speed_.is_valid(); }

  void send() {
    if (!has_data()) return;
    tN2kMsg msg;
    SetN2kBoatSpeed(msg, 0xff, water_speed_.to_n2k());
    nmea2000_->SendMsg(msg);
  }

  ExpiringValue<float> water_speed_;

 private:
  tNMEA2000* nmea2000_;
};

/// PGN 128267 — Water Depth (1000ms)
class N2kDepthSender {
 public:
  N2kDepthSender(tNMEA2000* nmea2000, unsigned long expiry = 5000)
      : nmea2000_(nmea2000), depth_(expiry), offset_(expiry) {}

  bool has_data() const { return depth_.is_valid(); }

  void send() {
    if (!has_data()) return;
    tN2kMsg msg;
    SetN2kWaterDepth(msg, 0xff, depth_.to_n2k(), offset_.to_n2k());
    nmea2000_->SendMsg(msg);
  }

  ExpiringValue<float> depth_;
  ExpiringValue<float> offset_;

 private:
  tNMEA2000* nmea2000_;
};

/// PGN 130306 — Wind Data (1000ms)
class N2kWindSender {
 public:
  N2kWindSender(tNMEA2000* nmea2000, unsigned long expiry = 5000)
      : nmea2000_(nmea2000), wind_speed_(expiry), wind_angle_(expiry) {}

  bool has_data() const {
    return wind_speed_.is_valid() || wind_angle_.is_valid();
  }

  void send() {
    if (!has_data()) return;
    tN2kMsg msg;
    SetN2kWindSpeed(msg, 0xff, wind_speed_.to_n2k(), wind_angle_.to_n2k(),
                    N2kWind_Apparent);
    nmea2000_->SendMsg(msg);
  }

  ExpiringValue<float> wind_speed_;
  ExpiringValue<float> wind_angle_;

 private:
  tNMEA2000* nmea2000_;
};

}  // namespace halser

#endif  // HALSER_SRC_N2K_SENDERS_H_
