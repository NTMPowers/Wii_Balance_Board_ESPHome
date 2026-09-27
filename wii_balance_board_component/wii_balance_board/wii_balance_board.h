#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "wii.h"

#include <array>
#include <string>
#include <unordered_map>
#include "task_queue.h"

namespace esphome {
namespace wii_balance_board {

struct Sample {
  float samples[64]{};
  size_t sample_count{0};
  size_t samples_filled{0};
  uint8_t temperature{0};
  uint8_t referenceTemperature{0};
  float measurement{NAN};
  bool measurement_published{false};
  bool telemetry_published{false};
};

class WiiBalanceBoard : public Component {
 public:
  WiiBalanceBoard();

  void setup() override;
  void loop() override;
  void dump_config() override;
  void sync(bool enable);

  // Forget stored pairing. remove_link_key() acts on the connected board and does
  // nothing when none is connected.
  void remove_link_key();
  void remove_all_link_keys();

  // Set the calibration offset, in kg, for the currently connected board.
  void set_offset(const std::string &value);

  void set_temperature_sensor(sensor::Sensor *temperature_sensor);
  void set_reference_temperature_sensor(sensor::Sensor *reference_temperature_sensor);
  void set_battery_level(sensor::Sensor *battery_level);
  void set_weight(sensor::Sensor *weight);
  void set_syncing(binary_sensor::BinarySensor *syncing);
  void set_ready_to_step_on(binary_sensor::BinarySensor *ready);
  void set_stddev(float stddev);
  void set_led_pin(int led_pin);
  void set_led_inverted(bool inverted);

  // Fires when a weighing is accepted, with the measured weight in kg.
  Trigger<float> *get_measurement_trigger() { return &this->measurement_trigger_; }

 protected:
  void board_connected(uint16_t handle, uint64_t bdaddr);
  void board_disconnected(uint16_t handle);
  void board_tared(uint16_t handle, bool ok);
  void board_sample(uint16_t handle, uint8_t battery, uint8_t reference_temp, uint8_t temperature, float topRightLoad,
                    float bottomRightLoad, float topLeftLoad, float bottomLeftLoad);
  void disconnect_active_board_();
  void schedule_disconnect_(uint16_t handle, uint32_t generation, uint32_t delay_ms);
  void set_ready_(bool ready);

  // Per-board offset storage (NVS), keyed by bdaddr.
  static bool load_offset_(uint64_t bdaddr, float *offset);
  static void save_offset_(uint64_t bdaddr, float offset);
  static bool remove_offset_(uint64_t bdaddr);
  static int remove_all_offsets_();

  detail::Bluetooth bluetooth;
  detail::Wii wii;
  std::unordered_map<uint16_t, Sample> sampleMap;
  detail::TaskQueue queue;
  uint32_t last_low_weight_log_ms_{0};
  // Disconnect tasks capture the generation at scheduling time and are discarded when
  // stale, so a timer from a dead session cannot kill a fresh one (the controller
  // reuses connection handles).
  uint32_t sessionGeneration{0};

  uint16_t active_handle_{0};
  uint64_t active_bdaddr_{0};
  uint32_t active_generation_{0};
  bool active_board_{false};
  // Set once the board's 0 kg points and reference temperature have been rewritten.
  bool board_zeroed_{false};
  // Calibration offset for the currently connected board, loaded from NVS on connect.
  float active_offset_{0.0f};

  float std_dev_;
  int led_pin_ = -1;
  bool led_inverted_ = false;

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *reference_temperature_sensor_{nullptr};
  sensor::Sensor *battery_level_{nullptr};
  sensor::Sensor *weight_{nullptr};
  binary_sensor::BinarySensor *syncing_{nullptr};
  binary_sensor::BinarySensor *ready_to_step_on_{nullptr};
  Trigger<float> measurement_trigger_;
};

}  // namespace wii_balance_board
}  // namespace esphome
