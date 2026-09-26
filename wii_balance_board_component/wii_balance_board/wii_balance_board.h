#pragma once

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "wii.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <array>
#include "task_queue.h"
#include <unordered_map>

namespace esphome {
namespace wii_balance_board {

struct Sample {
  float samples[64]{};
  size_t sample_count{0};
  size_t samples_filled{0};
  uint8_t battery{0};
  uint8_t temperature{0};
  uint8_t referenceTemperature{0};
  float measurement{NAN};
  bool measurement_published{false};
  bool calibrated{false};
  float zero_offset_kg{0.0f};
  float scale_factor{1.0f};
};

class WiiBalanceBoard : public Component {
 public:
  WiiBalanceBoard();

  void setup() override;
  void loop() override;
  void dump_config() override;
  void sync(bool enable);
  void capture_calibration_empty();
  void set_calibration_reference_weight(float weight_kg);
  void capture_calibration_load();
  void cancel_calibration();

  void set_temperature_sensor(sensor::Sensor *temperature_sensor);
  void set_reference_temperature_sensor(sensor::Sensor *reference_temperature_sensor);
  void set_battery_level(sensor::Sensor *battery_level);
  void set_weight(sensor::Sensor *weight);
  void set_syncing(binary_sensor::BinarySensor *syncing);
  void set_calibration_status_sensor(text_sensor::TextSensor *sensor);
  void set_stddev(float stddev);
  void set_led_pin(int led_pin);

 protected:
  enum class CalibrationStage : uint8_t { IDLE, CAPTURE_EMPTY, WAIT_FOR_LOAD, CAPTURE_LOAD };

  struct CalibrationProfile {
    uint32_t magic;
    float zero_offset_kg;
    float scale_factor;
  };

  void board_connected(uint16_t handle, uint64_t bdaddr);
  void board_disconnected(uint16_t handle);
  void board_sample(uint16_t handle, uint8_t battery, uint8_t reference_temp, uint8_t temperature, float topRightLoad,
                    float bottomRightLoad, float topLeftLoad, float bottomLeftLoad);
  void schedule_disconnect_(uint16_t handle, uint32_t generation, uint32_t delay_ms);
  void reset_calibration_samples_();
  void process_calibration_sample_(float adjusted_weight);
  void set_calibration_status_(const char *status);
  bool load_calibration_(uint64_t bdaddr, CalibrationProfile *profile);
  bool save_calibration_(uint64_t bdaddr, const CalibrationProfile &profile);

  detail::Bluetooth bluetooth;
  detail::Wii wii;
  std::unordered_map<uint16_t, Sample> sampleMap;
  detail::TaskQueue queue;
  // Session generation: incremented on every board_connected. Disconnect
  // tasks capture the generation at scheduling time and are discarded when it
  // is stale, so a timer from a dead session can never kill a fresh one
  // (connection handles are reused by the controller).
  uint32_t sessionGeneration{0};

  CalibrationStage calibration_stage_{CalibrationStage::IDLE};
  std::array<float, 64> calibration_samples_{};
  size_t calibration_sample_count_{0};
  size_t calibration_samples_filled_{0};
  float calibration_empty_weight_{NAN};
  float calibration_reference_weight_{NAN};
  uint16_t calibration_handle_{0};
  uint16_t active_handle_{0};
  uint64_t active_bdaddr_{0};
  uint32_t active_generation_{0};
  bool active_board_{false};
  CalibrationProfile active_calibration_{0, 0.0f, 1.0f};

  float std_dev_;
  int led_pin_;

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *reference_temperature_sensor_{nullptr};
  sensor::Sensor *battery_level_{nullptr};
  sensor::Sensor *weight_{nullptr};
  binary_sensor::BinarySensor *syncing_{nullptr};
  text_sensor::TextSensor *calibration_status_sensor_{nullptr};
};

}  // namespace wii_balance_board
}  // namespace esphome
