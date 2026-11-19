#include "zigbee_time.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <climits>

#ifdef USE_ESP32
#include <esp_timer.h>
#include <esp_private/esp_timer_private.h>
#endif

namespace esphome {
namespace zigbee {

// This time standard is the number of
// seconds since 0 hrs 0 mins 0 sec on 1st January 2000 UTC (Universal Coordinated Time).
constexpr time_t EPOCH_2000 = 946684800;

ZigbeeTime *global_time = nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

void ZigbeeTime::setup() {
  global_time = this;
  this->parent_->add_on_join_callback([this](bool x) { this->register_zb_time(); });
}

void ZigbeeTime::register_zb_time() {
  ezb_zcl_time_interface_t time_interface = {
      .get_utc_time = this->get_utc_time,
      .set_utc_time = this->set_utc_time,
  };
  ezb_err_t ret;
  ret = ezb_zcl_time_server_interface_register(this->time_ep_, time_interface);
  if (ret != EZB_ERR_NONE) {
    ESP_LOGW(TAG, "Setup failed: %u", ret);
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "Using Zigbee network as time source");
  this->update();
}

void ZigbeeTime::cb(ezb_err_t status) {
  if (status == EZB_ERR_NONE) {
    ESP_LOGD(TAG, "Time synchronization successful");
  } else if (status == EZB_ERR_TIMEOUT) {
    ESP_LOGW(TAG, "Time synchronization timed out");
  } else {
    ESP_LOGW(TAG, "Time synchronization failed with error: %d", status);
  }
}

// Sync by reading the Time attribute straight from the coordinator, rather
// than ezb_zcl_time_server_synchronize_time().
//
// The SDK routine broadcasts to every Time server on the network and adopts
// the "best" answer. In practice that left a device running exactly
// floor(2^32 us / 1e6) = 4294 s slow, and every later sync kept it there:
// the clock never recovered until it was written by hand. Whatever the SDK is
// doing internally, the coordinator is the network's authoritative clock
// (Zigbee2MQTT answers genTime reads with Master|Superseding status), so ask
// it directly and apply exactly what it says.
void ZigbeeTime::update() { this->request_coordinator_time_(); }

// Timer guard. ESP32-C6 rev v0.2 (seen on Seeed XIAO ESP32-C6 boards): SYSTIMER
// UNIT0, the counter behind esp_timer, spontaneously clears a group of its
// bits while the radio is busy and keeps counting from the lower value - a
// hardware fault, no write path involved. UNIT1 (FreeRTOS tick) is not
// affected. See espressif/esp-idf#19036. Here: 2026-11-15 -3 x 2^32 us at 3 h
// 50 min uptime, 2026-11-19 -7 x 2^32 us at 8 h 54 min (bits 36+ of the
// 16 MHz count). The damage is precise and repairable:
//   - ESPHome's scheduler runs on millis_64() = esp_timer, so every
//     interval and PollingComponent was suddenly due ~k x 71.6 min in the
//     future and stopped, while millis()/set_timeout users kept working;
//   - the wall clock is boot_time + esp_timer, so it jumped back as well.
// millis() comes from the FreeRTOS tick count, which runs on a different
// SYSTIMER counter and was unaffected both times. Compare the two every loop
// and, if esp_timer fell behind, move it forward by exactly what it lost:
// scheduler and wall clock are then both correct again, nothing reboots.
void ZigbeeTime::check_timer_() {
#ifdef USE_ESP32
  const uint32_t now_ms = millis();
  const int64_t now_us = esp_timer_get_time();
  if (this->tg_valid_) {
    const int64_t d_tick_ms = (int64_t) (uint32_t) (now_ms - this->tg_ms_);
    const int64_t d_timer_ms = (now_us - this->tg_us_) / 1000;
    const int64_t lag_ms = d_tick_ms - d_timer_ms;  // > 0: esp_timer fell behind
    // A cleared bit can never take more than the counter held, i.e. the
    // uptime. Anything bigger is not this fault - do not touch the timer.
    const bool plausible = lag_ms <= (int64_t) now_ms + 60000;
    if (lag_ms > 2000 && plausible) {
      this->timer_repairs_++;
      // Same bracketing IDF uses for its own counter adjustment after light
      // sleep: the esp_timer list lock, so no alarm is processed half-way.
      esp_timer_private_lock();
      esp_timer_private_advance(lag_ms * 1000);
      esp_timer_private_unlock();
      const int64_t fixed_us = esp_timer_get_time();
      ESP_LOGW(TAG,
               "esp_timer fell %" PRId64 " ms behind the tick count (%.3f x 2^32 us; was %" PRId64 " us, previous loop "
               "%" PRId64 " us) - advanced it back to %" PRId64 " us (repair #%" PRIu32 ", esp-idf#19036)",
               lag_ms, (double) lag_ms * 1000.0 / 4294967296.0, now_us, this->tg_us_, fixed_us,
               this->timer_repairs_);
      this->clock_event_callback_.call(
          CLOCK_EVENT_TIMER_REPAIRED,
          (int32_t) std::min<int64_t>(INT32_MAX, lag_ms / 1000));
      // Re-sample both together, after the logging above.
      this->tg_ms_ = millis();
      this->tg_us_ = esp_timer_get_time();
      return;
    }
    if (lag_ms > 2000) {
      ESP_LOGE(TAG, "esp_timer %" PRId64 " ms behind the tick count, more than the uptime - NOT repairing", lag_ms);
    }
    if (lag_ms < -2000) {
      ESP_LOGW(TAG, "esp_timer jumped %" PRId64 " ms AHEAD of the tick count (was %" PRId64 " us, previous loop %" PRId64
               " us) - left as is",
               -lag_ms, now_us, this->tg_us_);
      this->clock_event_callback_.call(
          CLOCK_EVENT_TIMER_FORWARD,
          (int32_t) std::min<int64_t>(INT32_MAX, -lag_ms / 1000));
    }
  }
  this->tg_us_ = now_us;
  this->tg_ms_ = now_ms;
  this->tg_valid_ = true;
#endif
}

void ZigbeeTime::loop() {
  // First, before anything below reads the time.
  this->check_timer_();
  const uint32_t now_ms = millis();

  // Backstop for the scheduled sync. update() runs from the PollingComponent
  // schedule; if that schedule ever stops firing (seen 2026-11-15: every
  // interval on the device went quiet for hours while one-shot timers kept
  // working), the clock must not silently stop being corrected.
  const uint32_t overdue_ms = this->get_update_interval() + 60000;
  if (this->last_request_ms_ != 0 && now_ms - this->last_request_ms_ > overdue_ms) {
    const int32_t since_s = (int32_t) ((now_ms - this->last_request_ms_) / 1000);
    ESP_LOGW(TAG, "Scheduled time sync has not run for %" PRId32 " s, requesting from loop", since_s);
    this->clock_event_callback_.call(CLOCK_EVENT_SYNC_OVERDUE, since_s);
    this->request_coordinator_time_();
  }

  // Jump guard. Between coordinator syncs the wall clock must advance exactly
  // as fast as millis(). Anything else is something other than us writing the
  // clock - on 2026-11-15 it went back by exactly 3 x 2^32 us (12885 s) with
  // no sync anywhere near it. check_timer_() above now catches the known cause
  // (esp_timer losing its upper bits) first; this stays as the second line of
  // defence for anything else that moves the wall clock. Put it back and say so.
  if (!this->has_time_ || now_ms - this->guard_checked_ms_ < 1000)
    return;
  this->guard_checked_ms_ = now_ms;
  const time_t expected = this->guard_epoch_ + (time_t) ((now_ms - this->guard_ms_) / 1000);
  const time_t seen = this->timestamp_now();
  const long long diff = (long long) seen - (long long) expected;
  if (diff > 5 || diff < -5) {
    this->reverted_jumps_++;
    ESP_LOGW(TAG, "Clock jumped by %lld s outside a coordinator sync - reverting (jump #%" PRIu32 ")", diff,
             this->reverted_jumps_);
    this->synchronize_epoch_((uint32_t) expected);
    this->rebase_guard_();
    this->clock_event_callback_.call(CLOCK_EVENT_JUMP_REVERTED,
                                     (int32_t) std::max<long long>(INT32_MIN, std::min<long long>(INT32_MAX, diff)));
    // And confirm against the coordinator straight away.
    this->request_coordinator_time_();
  }
}

void ZigbeeTime::rebase_guard_() {
  this->guard_epoch_ = this->timestamp_now();
  this->guard_ms_ = millis();
  this->guard_checked_ms_ = this->guard_ms_;
}

void ZigbeeTime::request_coordinator_time_() {
  this->last_request_ms_ = millis();
  if (this->last_request_ms_ == 0)
    this->last_request_ms_ = 1;  // 0 means "never"
  if (!this->parent_->is_connected()) {
    ESP_LOGD(TAG, "Not connected to Zigbee network, cannot synchronize time");
    return;
  }
  if (!esp_zigbee_lock_acquire(20 / portTICK_PERIOD_MS)) {
    ESP_LOGW(TAG, "Could not acquire Zigbee lock to synchronize time, will retry...");
    this->defer([this]() { this->request_coordinator_time_(); });
    return;
  }
  static uint16_t attrs[] = {0x0000};  // Time, UTCTime
  ezb_zcl_read_attr_cmd_t cmd{};
  cmd.cmd_ctrl.dst_addr.addr_mode = EZB_ADDR_MODE_SHORT;
  cmd.cmd_ctrl.dst_addr.u.short_addr = 0x0000;  // coordinator
  cmd.cmd_ctrl.dst_ep = 1;
  cmd.cmd_ctrl.src_ep = this->time_ep_;
  cmd.cmd_ctrl.cluster_id = 0x000A;  // Time
  cmd.payload.attr_number = 1;
  cmd.payload.attr_field = attrs;
  ezb_err_t ret = ezb_zcl_read_attr_cmd_req(&cmd);
  esp_zigbee_lock_release();
  if (ret != EZB_ERR_NONE) {
    ESP_LOGW(TAG, "Time read request failed: %d, retrying in 60 s", ret);
    this->set_timeout("zb_time_retry", 60000, [this]() { this->request_coordinator_time_(); });
    return;
  }
  ESP_LOGD(TAG, "Requested time from coordinator");
  this->awaiting_response_ = true;
  // No answer -> try again in a minute rather than waiting a full
  // update_interval with a clock that may be wrong.
  this->set_timeout("zb_time_rsp", 10000, [this]() {
    if (!this->awaiting_response_)
      return;
    this->awaiting_response_ = false;
    ESP_LOGW(TAG, "No time response from coordinator, retrying in 60 s");
    this->set_timeout("zb_time_retry", 60000, [this]() { this->request_coordinator_time_(); });
  });
}

void ZigbeeTime::handle_coordinator_time(uint32_t utc_2000) {
  this->awaiting_response_ = false;
  // 0xFFFFFFFF is ZCL "invalid"; anything before 2024 is a coordinator that
  // has no idea either. Neither is worth jumping the clock to.
  if (utc_2000 == 0xFFFFFFFF || utc_2000 < 757382400) {
    ESP_LOGW(TAG, "Coordinator returned an invalid time (%u), ignoring", (unsigned) utc_2000);
    return;
  }
  this->apply_coordinator_epoch_((time_t) utc_2000 + EPOCH_2000);
}

void ZigbeeTime::apply_coordinator_epoch_(time_t epoch) {
  const time_t before = this->timestamp_now();
  long long diff = 0;
  if (before > EPOCH_2000) {
    diff = (long long) epoch - (long long) before;
    if (diff > 2 || diff < -2) {
      ESP_LOGW(TAG, "Clock was off by %lld s, correcting from coordinator", diff);
    }
  }
  this->synchronize_epoch_((uint32_t) epoch);
  this->has_time_ = true;
  this->rebase_guard_();
  if (diff > 2 || diff < -2) {
    this->clock_event_callback_.call(CLOCK_EVENT_COORDINATOR_CORRECTED,
                                     (int32_t) std::max<long long>(INT32_MIN, std::min<long long>(INT32_MAX, diff)));
  }
}

// Answer other devices' genTime reads. Before the first coordinator sync the
// clock is at 1970, and 1970 - 2000 wraps to year ~2106; ZCL's "invalid" is
// 0xFFFFFFFF.
uint32_t ZigbeeTime::get_utc_time() {
  if (global_time == nullptr || !global_time->has_time_)
    return 0xFFFFFFFF;
  const time_t now = global_time->timestamp_now();
  if (now < EPOCH_2000)
    return 0xFFFFFFFF;
  return (uint32_t) (now - EPOCH_2000);
}

// The SDK calls this through the Time server interface. It is NOT trusted:
// the only clock source this component accepts is its own read of the
// coordinator. Whatever the SDK computes internally has been seen to be off by
// whole multiples of 2^32 us. Record the attempt and ignore it.
void ZigbeeTime::set_utc_time(uint32_t utc) {
  // Zigbee task - hand off to the main loop.
  global_time->set_epoch_time(utc + EPOCH_2000);
}

void ZigbeeTime::set_epoch_time(uint32_t utc) {
  this->defer([this, utc]() {
    this->ignored_sdk_sets_++;
    const long long diff = (long long) utc - (long long) this->timestamp_now();
    ESP_LOGW(TAG, "Zigbee stack tried to set the clock to %u (%+lld s) - ignored, coordinator reads only",
             (unsigned) utc, diff);
    this->clock_event_callback_.call(CLOCK_EVENT_SDK_SET_IGNORED,
                                     (int32_t) std::max<long long>(INT32_MIN, std::min<long long>(INT32_MAX, diff)));
  });
}

}  // namespace zigbee
}  // namespace esphome
