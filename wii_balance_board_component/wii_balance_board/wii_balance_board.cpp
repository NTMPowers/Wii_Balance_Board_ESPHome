#include "esphome/core/log.h"
#include "wii_balance_board.h"

#include "esphome/core/application.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include "utils.h"

namespace esphome {
namespace wii_balance_board {

static const char *TAG = "wii_balance_board.component";
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

void WiiBalanceBoard::set_ready_(bool ready) {
  if (ready_to_step_on_ != nullptr) {
    ready_to_step_on_->publish_state(ready);
  }
}

void WiiBalanceBoard::board_connected(uint16_t handle, uint64_t bdaddr) {
  ESP_LOGI(TAG, "Connected board: %012llX", static_cast<unsigned long long>(bdaddr));

  if (sampleMap.count(handle) > 0) {
    ESP_LOGE(TAG, "Same handle connected twice, ignoring connection.");
    return;
  }

  // Drop a stale disconnect task for a previous connection on this handle.
  queue.cancel(handle);

  Sample sample;
  active_handle_ = handle;
  active_bdaddr_ = bdaddr;
  active_board_ = true;
  board_zeroed_ = false;
  set_ready_(false);
  sampleMap.emplace(handle, sample);
  ESP_LOGI(TAG, "Zeroing board");

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
    auto sample = sampleMap.find(handle);
    if (sample == sampleMap.end() || !sample->second.measurement_published) {
      ESP_LOGI(TAG, "Timeout reached. Disconnecting board.");
    }
    wii.disconnect(handle);
  });
}

void WiiBalanceBoard::board_tared(uint16_t handle, bool ok) {
  if (!ok) {
    ESP_LOGE(TAG, "The board could not be zeroed, so no weight will be reported this session.");
    return;
  }
  if (!active_board_ || handle != active_handle_) {
    return;
  }
  board_zeroed_ = true;
  set_ready_(true);
  ESP_LOGI(TAG, "Ready to weigh");
}

void WiiBalanceBoard::board_disconnected(uint16_t handle) {
  ESP_LOGD(TAG, "Board connection ended");
  // Cancel any pending scheduled disconnect for this handle so it cannot
  // fire on a future connection that reuses the same handle number.
  queue.cancel(handle);
  if (active_board_ && active_handle_ == handle) {
    active_board_ = false;
    board_zeroed_ = false;
  }
  if (active_bdaddr_ == 0) {
    set_ready_(false);
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
  set_ready_(false);
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

  // No measurement until the board has been zeroed.
  if (!board_zeroed_) {
    return;
  }

  sample.referenceTemperature = reference_temp;
  sample.battery = battery;
  sample.temperature = temperature;

  // Sum the four cells, each already interpolated against the board's own 0/17/34 kg
  // points, then apply its temperature correction against the reference temperature
  // captured when the board was zeroed.
  float totalWeight = (topRightLoad + bottomRightLoad + topLeftLoad + bottomLeftLoad) / 1000;
  float weight = (.999 * totalWeight * (1.0 - .0007 * (sample.temperature - sample.referenceTemperature)));

  // Ignore small samples (noise), in std dev calculation.
  constexpr float minimum_weight = 3.0f;
  if (weight < minimum_weight) {
    const uint32_t now = millis();
    if (static_cast<uint32_t>(now - last_low_weight_log_ms_) >= 1000) {
      last_low_weight_log_ms_ = now;
      ESP_LOGD(TAG, "Low load raw=%.3f kg minimum=%.1f kg", static_cast<double>(weight),
               static_cast<double>(minimum_weight));
    }
    return;
  }

  int size = 64;
  sample.samples[sample.sample_count] = weight;
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
      ESP_LOGI(TAG, "Weight measured: %.2f kg", mean);
      if (!queue.reschedule(handle, millis() + 100)) {
        ESP_LOGW(TAG, "Disconnect timer missing after measurement; scheduling fallback");
        schedule_disconnect_(handle, active_generation_, 100);
      }
    }
  }
}

void WiiBalanceBoard::setup() {
  if (led_pin_ >= 0) {
    pinMode(led_pin_, OUTPUT);
    digitalWrite(led_pin_, HIGH);
  }
  set_ready_(false);
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
                   [this](const detail::BalanceBoardTared &tared) { this->board_tared(tared.handle, tared.ok); },
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
  if (enable) {
    ESP_LOGI(TAG, "Sync scan started");
  } else {
    ESP_LOGD(TAG, "Stopping sync scan");
  }
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
void WiiBalanceBoard::set_ready_to_step_on(binary_sensor::BinarySensor *ready) { this->ready_to_step_on_ = ready; }

}  // namespace wii_balance_board
}  // namespace esphome
