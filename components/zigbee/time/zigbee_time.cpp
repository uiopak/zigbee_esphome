#include "zigbee_time.h"
#include "esphome/core/log.h"

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

void ZigbeeTime::request_coordinator_time_() {
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
    ESP_LOGW(TAG, "Time read request failed: %d", ret);
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
  const time_t epoch = (time_t) utc_2000 + EPOCH_2000;
  const time_t before = this->timestamp_now();
  if (before > EPOCH_2000) {
    const long long diff = (long long) epoch - (long long) before;
    if (diff > 2 || diff < -2) {
      ESP_LOGW(TAG, "Clock was off by %lld s, correcting from coordinator", diff);
    }
  }
  this->synchronize_epoch_((uint32_t) epoch);
  this->has_time_ = true;
}

uint32_t ZigbeeTime::get_utc_time() { return (uint32_t) (global_time->timestamp_now() - EPOCH_2000); }

void ZigbeeTime::set_utc_time(uint32_t utc) {
  ESP_LOGD(TAG, "Received time synchronization request with UTC time: %u", utc + EPOCH_2000);
  global_time->set_epoch_time(utc + EPOCH_2000);
}

void ZigbeeTime::set_epoch_time(uint32_t utc) {
  // called from zigbee task, defer to main loop
  this->defer([this, utc]() {
    ESP_LOGD(TAG, "Setting device time to UTC: %u", utc);
    this->synchronize_epoch_(utc);
    this->has_time_ = true;
  });
}

}  // namespace zigbee
}  // namespace esphome