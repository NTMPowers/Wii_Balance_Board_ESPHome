#include "esphome/core/log.h"
#include "wii_balance_board.h"

#include "esphome/core/application.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <nvs.h>
#include "utils.h"

namespace esphome {
namespace wii_balance_board {

static const char *TAG = "wii_balance_board.component";
static constexpr char CALIBRATION_NVS_NAMESPACE[] = "wbb_cal";
static constexpr uint32_t CALIBRATION_MAGIC = 0x57424331;
static constexpr size_t SAMPLE_WINDOW_SIZE = 64;
static constexpr size_t TRIMMED_SAMPLE_COUNT = 6;

uint8_t interpret_battery_level(uint8_t batteryLevel) {
  if (batteryLevel >= 0x8d) {
    return 100;
  } else if (batteryLevel >= 0x7d) {
    return 75;
  } else if (batteryLevel >= 0x78) {
    return 50;
  } else if (batteryLevel >= 0x6A) {
    return 25;
  } else {
    return 0;
  }
}

WiiBalanceBoard::WiiBalanceBoard() : wii(&bluetooth), std_dev_(0.4) {}

void WiiBalanceBoard::board_connected(uint16_t handle, uint64_t bdaddr) {
  ESP_LOGI(TAG, "Connected board %012llX, scheduling 15 second backup disconnect",
           static_cast<unsigned long long>(bdaddr));

  if (sampleMap.count(handle) > 0) {
    ESP_LOGE(TAG, "Same handle connected twice, ignoring connection.");
    return;
  }

  // Drop any stale disconnect task for a previous connection with this handle
  // (handles get reused on quick reconnects), then queue sampling state.
  queue.cancel(handle);

  // Queue sampling timeout
  Sample sample;
  active_handle_ = handle;
  active_bdaddr_ = bdaddr;
  active_board_ = true;
  CalibrationProfile profile{};
  if (load_calibration_(bdaddr, &profile)) {
    active_calibration_ = profile;
    sample.calibrated = true;
    sample.zero_offset_kg = profile.zero_offset_kg;
    sample.scale_factor = profile.scale_factor;
    ESP_LOGI(TAG, "Loaded board calibration offset=%.3f kg scale=%.6f", sample.zero_offset_kg,
             sample.scale_factor);
    set_calibration_status_("Board Calibrated; 1 Kilogram Minimum Active");
  } else {
    active_calibration_ = CalibrationProfile{CALIBRATION_MAGIC, 0.0f, 1.0f};
    ESP_LOGI(TAG, "No saved calibration for this board; 10 kg minimum active");
    set_calibration_status_("Uncalibrated Board; Capture Empty To Calibrate");
  }
  sampleMap.emplace(handle, sample);

  // Schedule timeout disconnect; guarded by session generation so a task from
  // a dead session cannot fire on a new connection reusing the same handle.
  uint32_t generation = ++sessionGeneration;
  active_generation_ = generation;
  schedule_disconnect_(handle, generation, 15000);
}

void WiiBalanceBoard::schedule_disconnect_(uint16_t handle, uint32_t generation, uint32_t delay_ms) {
  queue.add(handle, millis() + delay_ms, [this, generation](int scheduled_handle) {
    uint16_t handle = static_cast<uint16_t>(scheduled_handle);
    if (generation != sessionGeneration) {
      ESP_LOGD(TAG, "Ignoring stale scheduled disconnect");
      return;
    }
    if (calibration_stage_ != CalibrationStage::IDLE && calibration_handle_ == handle) {
      ESP_LOGI(TAG, "Deferring backup disconnect while calibration is active");
      schedule_disconnect_(handle, generation, 15000);
      return;
    }
    ESP_LOGI(TAG, "Scheduled disconnect.");
    wii.disconnect(handle, 0x0013);
  });
}

void WiiBalanceBoard::board_disconnected(uint16_t handle) {
  ESP_LOGI(TAG, "Board disconnected handle=%u", handle);
  // Cancel any pending scheduled disconnect for this handle so it cannot
  // fire on a future connection that reuses the same handle number.
  queue.cancel(handle);
  if (active_board_ && active_handle_ == handle) {
    active_board_ = false;
    if (calibration_stage_ != CalibrationStage::IDLE && calibration_handle_ == handle) {
      calibration_stage_ = CalibrationStage::IDLE;
      reset_calibration_samples_();
      set_calibration_status_("Calibration Interrupted: Board Disconnected");
    }
  }
  if (sampleMap.count(handle) > 0) {
    auto &sample = sampleMap[handle];
    if (!sample.measurement_published) {
      if (sample.referenceTemperature > 0) {
        if (reference_temperature_sensor_ != nullptr)
          reference_temperature_sensor_->publish_state(sample.referenceTemperature);
        if (temperature_sensor_ != nullptr)
          temperature_sensor_->publish_state(sample.temperature);
        if (battery_level_ != nullptr)
          battery_level_->publish_state(sample.battery);
      }
      if (!isnan(sample.measurement) && weight_ != nullptr) {
        weight_->publish_state(sample.measurement);
      }
    }
    sampleMap.erase(handle);
  }


}

bool WiiBalanceBoard::load_calibration_(uint64_t bdaddr, CalibrationProfile *profile) {
  char key[16];
  snprintf(key, sizeof(key), "c%012llX", static_cast<unsigned long long>(bdaddr & 0xFFFFFFFFFFFFull));
  nvs_handle_t nvs_handle;
  if (nvs_open(CALIBRATION_NVS_NAMESPACE, NVS_READONLY, &nvs_handle) != ESP_OK) {
    return false;
  }
  size_t size = sizeof(*profile);
  esp_err_t result = nvs_get_blob(nvs_handle, key, profile, &size);
  nvs_close(nvs_handle);
  return result == ESP_OK && size == sizeof(*profile) && profile->magic == CALIBRATION_MAGIC &&
         std::isfinite(profile->zero_offset_kg) && profile->zero_offset_kg >= 0.0f &&
         profile->zero_offset_kg <= 250.0f && std::isfinite(profile->scale_factor) &&
         profile->scale_factor >= 0.1f && profile->scale_factor <= 10.0f;
}

bool WiiBalanceBoard::save_calibration_(uint64_t bdaddr, const CalibrationProfile &profile) {
  char key[16];
  snprintf(key, sizeof(key), "c%012llX", static_cast<unsigned long long>(bdaddr & 0xFFFFFFFFFFFFull));
  nvs_handle_t nvs_handle;
  esp_err_t result = nvs_open(CALIBRATION_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Unable to open calibration storage: %s", esp_err_to_name(result));
    return false;
  }
  result = nvs_set_blob(nvs_handle, key, &profile, sizeof(profile));
  if (result == ESP_OK) {
    result = nvs_commit(nvs_handle);
  }
  nvs_close(nvs_handle);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Unable to save calibration: %s", esp_err_to_name(result));
    return false;
  }
  return true;
}

void WiiBalanceBoard::set_calibration_status_(const char *status) {
  if (calibration_status_sensor_ != nullptr) {
    calibration_status_sensor_->publish_state(status);
  }
}

void WiiBalanceBoard::reset_calibration_samples_() {
  calibration_sample_count_ = 0;
  calibration_samples_filled_ = 0;
}

void WiiBalanceBoard::capture_calibration_empty() {
  if (!active_board_) {
    set_calibration_status_("Connect The Board Before Calibration");
    ESP_LOGW(TAG, "Cannot capture empty board: no board is connected");
    return;
  }
  if (calibration_stage_ == CalibrationStage::CAPTURE_LOAD) {
    set_calibration_status_("Loaded Capture Is Active; Cancel Before Restarting");
    return;
  }
  calibration_handle_ = active_handle_;
  calibration_empty_weight_ = NAN;
  calibration_stage_ = CalibrationStage::CAPTURE_EMPTY;
  reset_calibration_samples_();
  if (!queue.reschedule(active_handle_, millis() + 15000)) {
    schedule_disconnect_(active_handle_, active_generation_, 15000);
  }
  set_calibration_status_("Capturing Empty Board; Do Not Touch It");
  ESP_LOGI(TAG, "Calibration: capturing empty-board samples");
}

void WiiBalanceBoard::set_calibration_reference_weight(float weight_kg) {
  if (!std::isfinite(weight_kg) || weight_kg < 1.0f || weight_kg > 200.0f) {
    calibration_reference_weight_ = NAN;
    set_calibration_status_("Reference Weight Must Be Between 1 And 200 Kilograms");
    return;
  }
  calibration_reference_weight_ = weight_kg;
  ESP_LOGI(TAG, "Calibration reference weight set to %.2f kg", weight_kg);
}

void WiiBalanceBoard::capture_calibration_load() {
  if (!active_board_ || calibration_stage_ != CalibrationStage::WAIT_FOR_LOAD ||
      calibration_handle_ != active_handle_) {
    set_calibration_status_("Capture An Empty Board Before Starting Loaded Capture");
    return;
  }
  if (!std::isfinite(calibration_reference_weight_) || calibration_reference_weight_ < 1.0f ||
      calibration_reference_weight_ > 200.0f) {
    set_calibration_status_("Enter A Reference Weight Between 1 And 200 Kilograms First");
    return;
  }
  calibration_stage_ = CalibrationStage::CAPTURE_LOAD;
  reset_calibration_samples_();
  if (!queue.reschedule(active_handle_, millis() + 15000)) {
    schedule_disconnect_(active_handle_, active_generation_, 15000);
  }
  set_calibration_status_("Stand On The Board And Hold Still; Capture Is Automatic");
  ESP_LOGI(TAG, "Calibration: capturing loaded samples for %.2f kg", calibration_reference_weight_);
}

void WiiBalanceBoard::cancel_calibration() {
  if (calibration_stage_ == CalibrationStage::IDLE) {
    set_calibration_status_(active_board_ ? "Calibration Cancelled" : "Connect The Board Before Calibration");
    return;
  }
  calibration_stage_ = CalibrationStage::IDLE;
  reset_calibration_samples_();
  set_calibration_status_("Calibration Cancelled; Previous Calibration Retained");
  ESP_LOGW(TAG, "Calibration cancelled");
  if (active_board_) {
    if (!queue.reschedule(active_handle_, millis() + 15000)) {
      schedule_disconnect_(active_handle_, active_generation_, 15000);
    }
  }
}

void WiiBalanceBoard::process_calibration_sample_(float adjusted_weight) {
  calibration_samples_[calibration_sample_count_] = adjusted_weight;
  calibration_sample_count_ = (calibration_sample_count_ + 1) % SAMPLE_WINDOW_SIZE;
  calibration_samples_filled_ = std::min(calibration_samples_filled_ + 1, SAMPLE_WINDOW_SIZE);
  if (calibration_samples_filled_ < SAMPLE_WINDOW_SIZE || calibration_sample_count_ % 16 != 0) {
    return;
  }

  auto sorted = calibration_samples_;
  std::sort(sorted.begin(), sorted.end());
  constexpr size_t retained = SAMPLE_WINDOW_SIZE - 2 * TRIMMED_SAMPLE_COUNT;
  float mean = 0.0f;
  for (size_t i = TRIMMED_SAMPLE_COUNT; i < SAMPLE_WINDOW_SIZE - TRIMMED_SAMPLE_COUNT; ++i) {
    mean += sorted[i];
  }
  mean /= retained;
  float variance = 0.0f;
  for (size_t i = TRIMMED_SAMPLE_COUNT; i < SAMPLE_WINDOW_SIZE - TRIMMED_SAMPLE_COUNT; ++i) {
    const float difference = sorted[i] - mean;
    variance += difference * difference / (retained - 1);
  }
  const float deviation = std::sqrt(variance);
  const float max_deviation = calibration_stage_ == CalibrationStage::CAPTURE_LOAD ? std::max(std_dev_, 1.0f)
                                                                                    : std_dev_;
  if (!std::isfinite(mean) || deviation >= max_deviation) {
    ESP_LOGD(TAG, "Calibration window not stable yet: mean=%.3f kg deviation=%.3f kg", mean, deviation);
    return;
  }

  if (calibration_stage_ == CalibrationStage::CAPTURE_EMPTY) {
    if (mean < 0.0f || mean > 250.0f) {
      set_calibration_status_("Empty Capture Outside Valid Range; Retry Capture");
      reset_calibration_samples_();
      return;
    }
    calibration_empty_weight_ = mean;
    calibration_stage_ = CalibrationStage::WAIT_FOR_LOAD;
    reset_calibration_samples_();
    set_calibration_status_("Empty Captured; Enter Known Weight And Start Loaded Capture");
    ESP_LOGI(TAG, "Calibration empty capture=%.3f kg", calibration_empty_weight_);
    return;
  }

  if (calibration_stage_ != CalibrationStage::CAPTURE_LOAD) {
    return;
  }
  const float span = mean - calibration_empty_weight_;
  const float minimum_load_delta = std::max(0.5f, std::min(1.0f, calibration_reference_weight_ * 0.25f));
  if (span < minimum_load_delta) {
    ESP_LOGD(TAG, "Waiting for loaded capture: delta=%.3f kg required=%.3f kg", span, minimum_load_delta);
    return;
  }

  CalibrationProfile profile{CALIBRATION_MAGIC, calibration_empty_weight_, calibration_reference_weight_ / span};
  if (!std::isfinite(profile.scale_factor) || profile.scale_factor < 0.1f || profile.scale_factor > 10.0f ||
      !save_calibration_(active_bdaddr_, profile)) {
    calibration_stage_ = CalibrationStage::WAIT_FOR_LOAD;
    reset_calibration_samples_();
    set_calibration_status_("Calibration Could Not Be Saved; Step Off And Retry");
    return;
  }

  active_calibration_ = profile;
  auto sampleIt = sampleMap.find(calibration_handle_);
  if (sampleIt != sampleMap.end()) {
    Sample &sample = sampleIt->second;
    sample.calibrated = true;
    sample.zero_offset_kg = profile.zero_offset_kg;
    sample.scale_factor = profile.scale_factor;
    sample.sample_count = 0;
    sample.samples_filled = 0;
    sample.measurement = NAN;
    sample.measurement_published = false;
    std::fill(std::begin(sample.samples), std::end(sample.samples), 0.0f);
  }
  calibration_stage_ = CalibrationStage::IDLE;
  reset_calibration_samples_();
  set_calibration_status_("Calibration Saved For This Board");
  ESP_LOGI(TAG, "Calibration saved: empty=%.3f kg known=%.2f kg scale=%.6f", profile.zero_offset_kg,
           calibration_reference_weight_, profile.scale_factor);
  if (!queue.reschedule(calibration_handle_, millis() + 15000)) {
    schedule_disconnect_(calibration_handle_, active_generation_, 15000);
  }
}

void WiiBalanceBoard::board_sample(uint16_t handle, uint8_t battery, uint8_t reference_temp, uint8_t temperature,
                                   float topRightLoad, float bottomRightLoad, float topLeftLoad, float bottomLeftLoad) {
  auto sampleIt = sampleMap.find(handle);
  if (sampleIt == sampleMap.end()) {
    ESP_LOGD(TAG, "Ignoring measurement for inactive handle=%u", handle);
    return;
  }
  Sample &sample = sampleIt->second;

  // Ignore zero data
  if (reference_temp == 0 || !isnan(sample.measurement)) {
    return;
  }

  sample.referenceTemperature = reference_temp;
  sample.battery = battery;
  sample.temperature = temperature;

  float totalWeight = (topRightLoad + bottomRightLoad + topLeftLoad + bottomLeftLoad) / 1000;
  float factoryWeight = (.999 * totalWeight * (1.0 - .0007 * (sample.temperature - sample.referenceTemperature)));

  if (calibration_stage_ != CalibrationStage::IDLE && handle == calibration_handle_) {
    process_calibration_sample_(factoryWeight);
    return;
  }

  float adjusted = factoryWeight;
  if (sample.calibrated) {
    adjusted = std::max(0.0f, (factoryWeight - sample.zero_offset_kg) * sample.scale_factor);
  }

  // Ignore small samples (noise), in std dev calculation.
  const float minimum_weight = sample.calibrated ? 1.0f : 10.0f;
  if (adjusted < minimum_weight) {
    return;
  }

  int size = 64;
  sample.samples[sample.sample_count] = adjusted;
  sample.sample_count = (sample.sample_count + 1) % size;
  sample.samples_filled = std::min(sample.samples_filled + 1, static_cast<size_t>(size));

  // Not enough samples yet
  if (sample.samples_filled < static_cast<size_t>(size)) {
    return;
  }

  // For every 16th data point, sample standard deviation.
  if (sample.sample_count % 16 == 0) {
    std::array<float, SAMPLE_WINDOW_SIZE> sorted_samples;
    std::copy(std::begin(sample.samples), std::end(sample.samples), sorted_samples.begin());
    std::sort(sorted_samples.begin(), sorted_samples.end());
    constexpr size_t retained = SAMPLE_WINDOW_SIZE - 2 * TRIMMED_SAMPLE_COUNT;
    float mean = 0;
    for (size_t i = TRIMMED_SAMPLE_COUNT; i < SAMPLE_WINDOW_SIZE - TRIMMED_SAMPLE_COUNT; ++i) {
      mean += sorted_samples[i];
    }
    mean /= retained;

    float variance = 0.0f;
    for (size_t i = TRIMMED_SAMPLE_COUNT; i < SAMPLE_WINDOW_SIZE - TRIMMED_SAMPLE_COUNT; ++i) {
      const float difference = sorted_samples[i] - mean;
      variance += difference * difference / (retained - 1);
    }

    float deviation = std::sqrt(variance);

    if (mean >= minimum_weight && deviation < std_dev_) {
      sample.measurement = mean;
      if (reference_temperature_sensor_ != nullptr)
        reference_temperature_sensor_->publish_state(sample.referenceTemperature);
      if (temperature_sensor_ != nullptr)
        temperature_sensor_->publish_state(sample.temperature);
      if (battery_level_ != nullptr)
        battery_level_->publish_state(sample.battery);
      if (weight_ != nullptr)
        weight_->publish_state(sample.measurement);
      sample.measurement_published = true;

      // We have a valid sample, schedule board disconnect.
      ESP_LOGI(TAG, "Stable weight %.2f kg for handle=%u", mean, handle);
      queue.reschedule(handle, millis() + 100);
    }
  }
}

void WiiBalanceBoard::setup() {
  set_calibration_status_("Connect The Board With A To Check Calibration");
  if (led_pin_ >= 0) {
    pinMode(led_pin_, OUTPUT);
    digitalWrite(led_pin_, HIGH);
  }
  bluetooth.onReady([](auto) { ESP_LOGI(TAG, "Bluetooth initialized"); });

  wii.onEvent([this](const detail::WiiEvent &event) {
    std::visit(overloaded{
                   [this](const detail::ScanStarted &) {
                     syncing_->publish_state(true);
                     if (led_pin_ >= 0) {
                       digitalWrite(led_pin_, LOW);
                     }
                   },
                   [this](const detail::ScanStopped &) {
                     syncing_->publish_state(false);
                     if (led_pin_ >= 0) {
                       digitalWrite(led_pin_, HIGH);
                     }
                   },
                    [this](const detail::BalanceBoardConnected &board) {
                      syncing_->publish_state(false);
                      if (led_pin_ >= 0) {
                        digitalWrite(led_pin_, HIGH);
                      }
                      sync(false);
                      this->board_connected(board.handle, board.bdaddr);
                    },
                   [this](const detail::BalanceBoardDisconnected &board) { this->board_disconnected(board.handle); },
                   [this](const detail::BalanceBoardData &data) {
                     this->board_sample(data.handle, interpret_battery_level(data.batteryLevel),
                                        data.referenceTemperature, data.temperature, data.tr, data.br, data.tl,
                                        data.bl);
                   },
               },
               event);
  });
}

void WiiBalanceBoard::loop() {
  wii.step();
  queue.process(millis());
}

void WiiBalanceBoard::sync(bool enable) {
  ESP_LOGI(TAG, enable ? "Starting scan" : "Stopping scan");
  wii.sync(enable);
}

void WiiBalanceBoard::dump_config() { ESP_LOGCONFIG(TAG, "Wii Balance Board"); }

void WiiBalanceBoard::set_temperature_sensor(sensor::Sensor *temperature_sensor) {
  temperature_sensor_ = temperature_sensor;
}
void WiiBalanceBoard::set_reference_temperature_sensor(sensor::Sensor *reference_temperature_sensor) {
  reference_temperature_sensor_ = reference_temperature_sensor;
}
void WiiBalanceBoard::set_battery_level(sensor::Sensor *battery_level) { battery_level_ = battery_level; }
void WiiBalanceBoard::set_weight(sensor::Sensor *weight) { weight_ = weight; }
void WiiBalanceBoard::set_stddev(float stddev) { this->std_dev_ = stddev; }
void WiiBalanceBoard::set_led_pin(int led_pin) { this->led_pin_ = led_pin; }
void WiiBalanceBoard::set_syncing(binary_sensor::BinarySensor *syncing) { this->syncing_ = syncing; }
void WiiBalanceBoard::set_calibration_status_sensor(text_sensor::TextSensor *sensor) {
  calibration_status_sensor_ = sensor;
}

}  // namespace wii_balance_board
}  // namespace esphome
