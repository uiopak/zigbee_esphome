#pragma once

#include "esphome/core/component.h"
#include "esphome/components/time/real_time_clock.h"
#include "../zigbee.h"

namespace esphome {
namespace zigbee {

class ZigBeeComponent;

class ZigbeeTime;
extern ZigbeeTime *global_time;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

class ZigbeeTime : public time::RealTimeClock {
 public:
  ZigbeeTime(ZigBeeComponent *parent, uint8_t ep) : parent_(parent), time_ep_(ep) {}
  void setup() override;
  void update() override;
  void set_epoch_time(uint32_t utc);
  // Time attribute (seconds since 2000-01-01 UTC) read back from the
  // coordinator. Called on the main loop.
  void handle_coordinator_time(uint32_t utc_2000);

 protected:
  void register_zb_time();
  void request_coordinator_time_();
  bool awaiting_response_{false};
  static void set_utc_time(uint32_t utc);
  static uint32_t get_utc_time();
  static void cb(ezb_err_t status);
  bool has_time_{false};
  uint8_t time_ep_;
  ZigBeeComponent *parent_;
};

}  // namespace zigbee
}  // namespace esphome
