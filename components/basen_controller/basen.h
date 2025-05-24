#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace basen {

class BasenBMS;  // Forward declaration

class BasenController : public uart::UARTDevice, public Component {
 public:
  void set_throttle(uint32_t throttle) { this->throttle_ = throttle; }

  void register_bms(BasenBMS *bms) {
    this->devices_.push_back(bms);
  }

  void dump_config() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  bool empty_rx (void);
  uint8_t checksum (const uint8_t *data, uint8_t len);
  void send_command(BasenBMS *BMS, const uint8_t command);
  void handle_data (const uint8_t *header, const uint8_t *data, uint8_t length);
  void handle_info(const uint8_t *data, uint8_t length);
  uint8_t handle_cell_voltages(const uint8_t *data, uint8_t length);
  void queue_device(BasenBMS *bms);
  void update_device();
  void uart_loop();

  enum {
    STATE_BUS_CHECK = 0,
    STATE_IDLE,
    STATE_COMMAND_TX,
    STATE_RX_HEADER,
    STATE_RX_DATA
  };

  uint8_t state_{STATE_BUS_CHECK};
  uint8_t header_[4]{0};
  uint8_t data_required_{0};
  uint8_t address_{1};

  bool enable_{false};
  bool publishing_{true};
  uint32_t last_bus_check_{0};
  uint32_t last_publish_{0};
  uint32_t throttle_{0};
  uint32_t timeout_{2000};  // Timeout for command response in ms

  std::vector<BasenBMS *> devices_;
  BasenBMS *current_{nullptr};  // Pointer to the currently processing BMS
};

class BasenBMS : public PollingComponent {
 public:
  void update();

  void set_parent(BasenController *parent) { parent_ = parent; parent->register_bms(this); }
  void set_address(uint8_t address) { address_ = address; }

  // Sensors
  void set_bms_version_text_sensor(text_sensor::TextSensor *bms_version_text_sensor) {
    bms_version_text_sensor_ = bms_version_text_sensor;
  }
  void set_barcode_text_sensor(text_sensor::TextSensor *barcode_text_sensor) { barcode_text_sensor_ = barcode_text_sensor; }
  void set_status_bitmask_text_sensor(text_sensor::TextSensor *sensor) {
    status_bitmask_sensor_ = sensor;
  }

  void set_connected_binary_sensor(binary_sensor::BinarySensor *binary_sensor) {
    connected_binary_sensor_ = binary_sensor;
  }

  void set_voltage_sensor(sensor::Sensor *sensor) {
    voltage_sensor_ = sensor;
  }
  void set_current_sensor(sensor::Sensor *sensor) {
    current_sensor_ = sensor;
  }
  void set_power_sensor(sensor::Sensor *sensor) {
    power_sensor_ = sensor;
  }
  void set_capacity_sensor(sensor::Sensor *sensor) {
    capacity_sensor_ = sensor;
  }
  void set_state_of_charge_sensor(sensor::Sensor *sensor) {
    soc_sensor_ = sensor;
  }
  void set_state_of_health_sensor(sensor::Sensor *sensor) {
    soh_sensor_ = sensor;
  }
  void set_cycles_sensor(sensor::Sensor *sensor) {
    cycles_sensor_ = sensor;
  }
  void set_temperature1_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[0] = sensor;
  }
  void set_temperature2_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[1] = sensor;
  }
  void set_temperature3_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[2] = sensor;
  }
  void set_temperature4_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[3] = sensor;
  }
  void set_temperature_mos_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[4] = sensor;
  }
  void set_temperature_ambient_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[5] = sensor;
  }
  void set_avg_cell_voltage_sensor(sensor::Sensor *sensor) {
    avg_cell_voltage_sensor_ = sensor;
  }
  void set_min_cell_voltage_sensor(sensor::Sensor *sensor) {
    min_cell_voltage_sensor_ = sensor;
  }
  void set_max_cell_voltage_sensor(sensor::Sensor *sensor) {
    max_cell_voltage_sensor_ = sensor;
  }
  void set_min_cell_index_sensor(sensor::Sensor *sensor) {
    min_cell_index_sensor_ = sensor;
  }
  void set_max_cell_index_sensor(sensor::Sensor *sensor) {
    max_cell_index_sensor_ = sensor;
  }
  void set_delta_cell_voltage_sensor(sensor::Sensor *sensor) {
    delta_cell_voltage_sensor_ = sensor;
  }
  
 protected:
  friend BasenController;
  uint8_t address_{1};
  uint8_t update_{0};
  uint32_t last_transmission_{0};

  // Sensors
  text_sensor::TextSensor *bms_version_text_sensor_{nullptr};
  text_sensor::TextSensor *barcode_text_sensor_{nullptr};
  text_sensor::TextSensor *status_bitmask_sensor_{nullptr};
  binary_sensor::BinarySensor *connected_binary_sensor_{nullptr};
  
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *capacity_sensor_{nullptr};
  sensor::Sensor *soc_sensor_{nullptr};
  sensor::Sensor *soh_sensor_{nullptr};
  sensor::Sensor *cycles_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_[6]{nullptr};
  sensor::Sensor *avg_cell_voltage_sensor_{nullptr};
  sensor::Sensor *min_cell_voltage_sensor_{nullptr};
  sensor::Sensor *max_cell_voltage_sensor_{nullptr};
  sensor::Sensor *min_cell_index_sensor_{nullptr};
  sensor::Sensor *max_cell_index_sensor_{nullptr};
  sensor::Sensor *delta_cell_voltage_sensor_{nullptr};

  BasenController *parent_;
};

}  // namespace basen
}  // namespace esphome
