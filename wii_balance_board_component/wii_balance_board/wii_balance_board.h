#pragma once

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "esphome/components/sensor/sensor.h"
#include "wii.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

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
};

class WiiBalanceBoard : public Component {
 public:
  WiiBalanceBoard();

  void setup() override;
  void loop() override;
  void dump_config() override;
  void sync(bool enable);

  void set_temperature_sensor(sensor::Sensor *temperature_sensor);
  void set_reference_temperature_sensor(sensor::Sensor *reference_temperature_sensor);
  void set_battery_level(sensor::Sensor *battery_level);
  void set_weight(sensor::Sensor *weight);
  void set_syncing(binary_sensor::BinarySensor *syncing);
  void set_stddev(float stddev);
  void set_led_pin(int led_pin);

 protected:
  void board_connected(uint16_t handle, uint64_t bdaddr);
  void board_disconnected(uint16_t handle);
  void board_sample(uint16_t handle, uint8_t battery, uint8_t reference_temp, uint8_t temperature, float topRightLoad,
                    float bottomRightLoad, float topLeftLoad, float bottomLeftLoad);

  detail::Bluetooth bluetooth;
  detail::Wii wii;
  std::unordered_map<uint16_t, Sample> sampleMap;
  detail::TaskQueue queue;
  // Session generation: incremented on every board_connected. Disconnect
  // tasks capture the generation at scheduling time and are discarded when it
  // is stale, so a timer from a dead session can never kill a fresh one
  // (connection handles are reused by the controller).
  uint32_t sessionGeneration{0};

  float std_dev_;
  int led_pin_;

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *reference_temperature_sensor_{nullptr};
  sensor::Sensor *battery_level_{nullptr};
  sensor::Sensor *weight_{nullptr};
  binary_sensor::BinarySensor *syncing_{nullptr};
};

}  // namespace wii_balance_board
}  // namespace esphome
