#pragma once

#include <atomic>

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/time/real_time_clock.h"
#include "../zigbee.h"

namespace esphome {
namespace zigbee {

class ZigBeeComponent;

class ZigbeeTime;
extern ZigbeeTime *global_time;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

// What happened to the clock, for on_clock_event listeners.
enum ZigbeeClockEvent : uint8_t {
  // A coordinator read moved the clock by more than 2 s. diff = new - old.
  CLOCK_EVENT_COORDINATOR_CORRECTED = 1,
  // The clock moved by itself, outside a coordinator sync, and was put back.
  // diff = how far it had moved (seen - expected).
  CLOCK_EVENT_JUMP_REVERTED = 2,
  // The Zigbee SDK tried to set the clock through the Time server interface
  // and was ignored. diff = what it wanted minus the current clock.
  CLOCK_EVENT_SDK_SET_IGNORED = 3,
  // The scheduled (update_interval) sync had not run for update_interval +
  // 60 s, so loop() asked the coordinator itself. diff = seconds since the
  // previous request.
  CLOCK_EVENT_SYNC_OVERDUE = 4,
  // esp_timer (the microsecond timer behind ESPHome's scheduler AND the wall
  // clock) fell behind the FreeRTOS tick count and was advanced back into
  // line. diff = seconds it had lost.
  CLOCK_EVENT_TIMER_REPAIRED = 5,
  // esp_timer jumped AHEAD of the tick count. Not undone (that would stall
  // every timer scheduled since); the wall clock guard handles the clock.
  // diff = seconds it gained.
  CLOCK_EVENT_TIMER_FORWARD = 6,
};

class ZigbeeTime : public time::RealTimeClock {
 public:
  ZigbeeTime(ZigBeeComponent *parent, uint8_t ep) : parent_(parent), time_ep_(ep) {}
  void setup() override;
  void loop() override;
  void update() override;
  void set_epoch_time(uint32_t utc);
  // Time attribute (seconds since 2000-01-01 UTC) read back from the
  // coordinator. Called on the main loop.
  void handle_coordinator_time(uint32_t utc_2000);

  // Always called on the main loop.
  template<typename F> void add_on_clock_event_callback(F &&callback) {
    this->clock_event_callback_.add(std::forward<F>(callback));
  }
  uint32_t get_reverted_jumps() const { return this->reverted_jumps_; }
  uint32_t get_ignored_sdk_sets() const { return this->ignored_sdk_sets_; }
  uint32_t get_timer_repairs() const { return this->timer_repairs_; }

 protected:
  void check_timer_();
  void register_zb_time();
  void request_coordinator_time_();
  void apply_coordinator_epoch_(time_t epoch);
  void rebase_guard_();
  bool awaiting_response_{false};
  static void set_utc_time(uint32_t utc);
  static uint32_t get_utc_time();
  static void cb(ezb_err_t status);
  bool has_time_{false};
  uint8_t time_ep_;
  ZigBeeComponent *parent_;

  // Jump guard: wall clock expected = guard_epoch_ + (millis - guard_ms_).
  time_t guard_epoch_{0};
  uint32_t guard_ms_{0};
  uint32_t guard_checked_ms_{0};
  // When the last coordinator request went out, so loop() can re-request if
  // the PollingComponent schedule ever stops firing.
  uint32_t last_request_ms_{0};
  uint32_t reverted_jumps_{0};
  uint32_t ignored_sdk_sets_{0};
  // Timer guard: esp_timer_get_time() and millis() (FreeRTOS ticks, a separate
  // SYSTIMER counter) sampled together on the previous loop().
  int64_t tg_us_{0};
  uint32_t tg_ms_{0};
  bool tg_valid_{false};
  uint32_t timer_repairs_{0};
  CallbackManager<void(uint8_t, int32_t)> clock_event_callback_{};
};

}  // namespace zigbee
}  // namespace esphome
