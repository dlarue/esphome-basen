#include "basen.h"
#include "esphome/core/log.h"
#include <algorithm>
#include "esphome/core/helpers.h"

namespace esphome {
namespace basen {

static const char *const TAG = "basen";

static const uint8_t SOI = 0x7E;
static const uint8_t EOI = 0x0D;
#define COMMAND_INFO 0x01
#define COMMAND_BMS_VERSION 0x33
#define COMMAND_BARCODE 0x42

void BasenController::dump_config() {
  ESP_LOGCONFIG(TAG, "Basen:");

  check_uart_settings(9600);
}

bool BasenController::empty_rx (void) {
  uint8_t data;
  uint8_t count = 255;
  bool result = false;
  while (available() && --count) {
    read_byte(&data);
    //ESP_LOGD(TAG, "Emptying RX buffer: %02X", data);
    result = true;
  }

  return result;
}

uint8_t BasenController::checksum (const uint8_t *data, uint8_t len) {
  uint8_t num1 = 0;
  uint16_t num2 = 0;

  for (uint8_t i = 0; i < len; i++) {
      num1 ^= data[i];
      num2 += data[i];
  }

  return (uint8_t)((num1 ^ num2) & 0xFF);
}

void BasenController::send_command(BasenBMS *BMS, const uint8_t command) {
  uint8_t data[6] = {SOI, BMS->address_, command, 0x00, 0xFE, EOI};

  // Calculate checksum
  // Examples: (Checksum byte is before EOI 0x0d)
  // 7e 01 01 00 fe 0d
  // 7e 01 42 00 fc 0d
  // 7e 01 dc 03 06 00 00 c2 0d
  // 7e 01 33 00 fe 0d
  data[4] = this->checksum(data, 4);

  write_array(data, sizeof(data));
  this->state_ = STATE_COMMAND_TX;
  memcpy(this->header_, data, sizeof(header_));
  BMS->last_transmission_ = millis();
  ESP_LOGD(TAG, "Sending command: %02X %02X %02X %02X %02X %02X", data[0], data[1], data[2], data[3], data[4], data[5]);
}

void BasenController::update_device ()
{
  if (current_ == NULL)
    return;

  if (!current_->bms_version_text_sensor_->has_state()) {
    // Get BMS version
    this->send_command(current_, COMMAND_BMS_VERSION);
  } else if (!current_->barcode_text_sensor_->has_state()) {
    // Get barcode
    this->send_command(current_, COMMAND_BARCODE);
  } else if ((millis()-current_->last_transmission_) > 1000) {
    // Get info
    this->send_command(current_, COMMAND_INFO);
  }
}

void BasenController::uart_loop() {
  // Check state
  switch (this->state_) {
    case STATE_BUS_CHECK:
      // Check if there are no transmissions on the bus
      if (millis() - last_bus_check_ >= 3000) {
        // Check and empty RX buffer
        bool ret = empty_rx();
        this->last_bus_check_ = millis();

        if (ret) {
          ESP_LOGW(TAG, "Bus is not idle...");
          // Check again in 3 seconds
          return;
        }
        ESP_LOGD(TAG, "Bus is idle");
        this->state_ = STATE_IDLE;
      }
      return;
    case STATE_IDLE:
      // Start sending commands
      this->update_device();
      return;
    case STATE_COMMAND_TX:
      // Command was sent, wait for response
      this->state_ = STATE_RX_HEADER;
      // Minimum 6 bytes response
      this->data_required_ = 6;
      return;
    case STATE_RX_HEADER:
    case STATE_RX_DATA:
      if (current_ == NULL) {
        ESP_LOGW(TAG, "No device updating, cannot receive data");
        this->state_ = STATE_BUS_CHECK;
        return;
      }
      break;
  }

  // Check if we have received the data
  if (available() < this->data_required_) {
    if (millis() - current_->last_transmission_ >= timeout_) {
      // Timeout while waiting for data.
      ESP_LOGW(TAG, "Timeout while waiting for data for address %02X", current_->address_);
      current_->connected_binary_sensor_->publish_state(false);
      current_->state_ = BasenBMS::BMS_STATE_DONE;  // Mark as finished
      current_ = NULL;  // Reset current device
      this->state_ = STATE_BUS_CHECK;
    }
    return;
  }

  if (state_ == STATE_RX_HEADER) {
    // Get first four bytes
    uint8_t header[4] = {0};
    read_array(header, sizeof(header));
    ESP_LOGD(TAG, "Received header: %02X %02X %02X %02X", header[0], header[1], header[2], header[3]);

    if (header[0] != SOI || header[1] != this->header_[1] || header[2] != this->header_[2]) {
        ESP_LOGW(TAG, "Header does not match expected values");
        this->state_ = STATE_BUS_CHECK;
        return;
    }

    // Check length
    uint8_t length = header[3];

    if (length == 0x00) {
      // Command echo, verify EOI
      uint8_t tail[2] = {0};
      read_array(tail, sizeof(tail));

      if (tail[1] != EOI) {
        ESP_LOGW(TAG, "Invalid tail");
        this->state_ = STATE_BUS_CHECK;
      }

      return;
    }

    if (length > 0xF0) {
      // Invalid length
      ESP_LOGW(TAG, "Invalid length");
      this->state_ = STATE_BUS_CHECK;
      return;
    }

    // Check if we have enough data
    this->data_required_ = length + 2;  // Length + checksum + EOI

    memcpy(this->header_, header, sizeof(header_));
    state_ = STATE_RX_DATA;

    // Return here to keep loop time short
    return;
  }

  if (available () < this->data_required_) {
    // Not enough data, wait
    return;
  }

  if (this->data_required_ == 3) {
    // Only one data byte, most likely an error code:
    // 00H Normal
    // 01H VER Error
    // 02H CHKSUM Error
    // 03H LCHKSUM Error
    // 04H CID2 Invalid
    // 05H Command Format Error
    // 06H Invalid data
    uint8_t error[3] = {0};
    read_array(error, sizeof(error));
    ESP_LOGW(TAG, "Received error: %02X %02X %02X", error[0], error[1], error[2]);
    this->state_ = STATE_BUS_CHECK;
    return;
  }

  // Get data
  uint16_t framesize = sizeof (header_) + data_required_;
  uint8_t frame[framesize] = {0};
  memcpy (frame, header_, sizeof(header_));
  uint8_t *data = frame + sizeof(header_);
  read_array(data, this->data_required_);

  // ESP_LOGD(TAG, "Received data: ");
  // for (size_t i = 0; i < this->data_required_; i++) {
  //   ESP_LOGD(TAG, "%02X ", data[i]);
  // }
  // ESP_LOGD(TAG, "");

  // Check EOI
  if (frame[framesize - 1] != EOI) {
    ESP_LOGW(TAG, "Invalid EOI byte: %02X", frame[framesize - 1]);
    this->state_ = STATE_BUS_CHECK;
    return;
  }

  // Verify checksum
  uint8_t checksum = this->checksum(frame, sizeof(header_) + header_[3]);

  if (checksum != data[header_[3]]) {
    ESP_LOGW(TAG, "Checksum mismatch: %02X != %02X", checksum, data[header_[3]]);
    this->state_ = STATE_BUS_CHECK;
    return;
  }
  
  // Handle data
  handle_data (header_, data, header_[3]);

  this->state_ = STATE_IDLE;
}

void BasenController::queue_device(BasenBMS *device) {
  // Check if we already have a device updating
  if (current_ != NULL)
    return;

  // No, set current device
  device->state_ = BasenBMS::BMS_STATE_UPDATING;
  current_ = device;
  ESP_LOGD(TAG, "Active device with address %02X", device->address_);
}

void BasenController::loop() {
  if (millis() >= 1000*10)
    enable_ = true;  // Enable after 10 seconds

  // Check if enabled
  if (!enable_) {
    if (current_ != NULL) {
      // Reset current device
      current_->state_= BasenBMS::BMS_STATE_IDLE;
      current_ = NULL;
    }
    state_ = STATE_BUS_CHECK;
    return;
  }

  uint8_t found = 0;

  // Check which devices need an update
  for (auto &device : this->devices_) {
    switch (device->state_) {
      default:
        // Device is idle, nothing to do
        break;
      case BasenBMS::BMS_STATE_WANT_UPDATE:
        // Update this device
        queue_device(device);
        found++;
        break;
      case BasenBMS::BMS_STATE_PUBLISH:
        device->publish();
        // Keep the loop time low
        return;
    }
  }

  if (!found) {
    // Remove finished devices
    for (auto &device : this->devices_) {
      if (device->state_ == BasenBMS::BMS_STATE_DONE)
        device->state_ = BasenBMS::BMS_STATE_IDLE;
    }
  }
  
  // Call the UART loop
  uart_loop();
}

uint8_t BasenController::handle_cell_voltages (const uint8_t *data, uint8_t length) {
  if (length < 1) {
    ESP_LOGW(TAG, "Invalid length for cell voltages: %d", length);
    return 0;
  }
  length--;
  uint8_t num_cells = data[0];  // First byte is the number of cells

  if (num_cells == 0) {
    ESP_LOGW(TAG, "No cells found in data");
    return 0;
  }

  if ((num_cells*2) > length) {
    ESP_LOGW(TAG, "Number of cells (%d) exceeds data length (%d)", num_cells, length);
    return 0;
  }

  float cell_avg_voltage = 0.0f;
  float cell_min_voltage = 1000.0f;
  float cell_max_voltage = 0.0f;
  uint8_t min_cell_index = 0;
  uint8_t max_cell_index = 0;
  for (uint8_t i = 0; i < num_cells; i++) {
    // Each cell voltage is represented by two bytes
    uint16_t cell_voltage = (data[i*2 + 1] << 8) | data[i*2 + 2];
    // Convert to V
    float voltage_V = cell_voltage / 1000.0f;
    ESP_LOGD(TAG, "Cell %d voltage: %.3f V", i + 1, voltage_V);

    // Update average, min and max voltages
    cell_avg_voltage += voltage_V;
    if (voltage_V < cell_min_voltage) {
      cell_min_voltage = voltage_V;
      min_cell_index = i + 1;  // Cell index starts at 1
    }
    if (voltage_V > cell_max_voltage) {
      cell_max_voltage = voltage_V;
      max_cell_index = i + 1;  // Cell index starts at 1
    }

    // Publish cell voltage
  }

  cell_avg_voltage /= num_cells;  // Calculate average voltage

  // Publish calculated values
  if (current_ != NULL) {
    current_->cell_avg_voltage_ = cell_avg_voltage;
    current_->cell_min_voltage_ = cell_min_voltage;
    current_->cell_max_voltage_ = cell_max_voltage;
    current_->cell_min_index_ = min_cell_index;
    current_->cell_max_index_ = max_cell_index;
  }

  // Return the number of bytes processed
  return 1 + num_cells * 2;
}

void BasenController::handle_info (const uint8_t *data, uint8_t length) {
  // Handle info data
  if (length != 0x7C) {
    ESP_LOGW(TAG, "Invalid length for COMMAND_INFO: %d", length);
    return;
  }
  if (data[0] != 0x01) {
    ESP_LOGW(TAG, "Invalid version for COMMAND_INFO: %02X", data[0]);
    return;
  }

  uint8_t *position = const_cast<uint8_t *>(data) + 1;  // Skip version byte
  length--;

  uint8_t processed = 0;
  // Process cell voltages
  processed = handle_cell_voltages(position, length);

  if (processed == 0) {
    return;
  }

  position += processed;
  length -= processed;

  while (length > 0) {
    if (length < 2) {
      ESP_LOGW(TAG, "Invalid length for next position: %d", length);
      return;
    }

    uint8_t type = position[0];  // Data type
    uint8_t count = position[1]; // Number of elements
    // Size in bytes of one element
    uint8_t size = 2;
    if (type > 0x0A)
        size = 4;

    // Get first data element
    uint16_t data16 = 0;
    if (count && length >= 4) {
      data16 = (position[2] << 8) | position[3];
    }
  
    switch (position[0]) {
      case 0x02:  // Current
        current_->current_ = (data16 / -100.0f) + 300;  // Convert to A
        break;
      case 0x03:  // SoC
        current_->soc_ = data16 / 100.0f;  // Publish SoC in percentage
        break;
      case 0x04:  // Capacity
        current_->capacity_ = data16 / 100;  // Publish capacity in Ah
        break;
      case 0x05:  // Temperature
        if (count > 6) {
          ESP_LOGW(TAG, "Invalid temperature count: %d", count);
          break;
        }
        for (uint8_t i = 0; i < count; i++) {
          uint8_t type = position[2 + 2*i];  // Type of temperature sensor
          int8_t value = position[2 + 2*i + 1] - 50;  // Value of temperature sensor
          if (type == 0x40) {
            // MOS temperature
            current_->temperature_mos_ = value;
          } else if (type == 0x20) {
            // Ambient temperature
            current_->temperature_ambient_ = value;
          } else {
            // Normal temperature sensor
            if (i < 4)
              current_->temperature_[i] = value;
          }
        }
        break;
      case 0x06:  // Status bitmask
        // Create a string containing all bytes
        if (!count || (count > 5))
          break;
        memcpy (current_->status_bitmask_, position + 2, count * size);
        break;
      case 0x07:  // Cycles
        current_->cycles_ = data16;
        break;
      case 0x08:  // Total voltage
        current_->voltage_ = data16 / 100.0f;
        break;
      case 0x09:  // SoH
        current_->soh_ = data16 / 100.0f;
        break;
      default:        
        ESP_LOGD(TAG, "Data type %02X with count %d", type, count);
    }

    // Advance to next data type
    uint16_t data_size = count * size + 2;  // 2 bytes for type and count
    if (length < data_size) {
      ESP_LOGW(TAG, "Not enough data for type %02X with count %d", type, count);
      break;
    }

    length -= data_size;
    position += data_size;
  }
}

void BasenController::handle_data (const uint8_t *header, const uint8_t *data, uint8_t length) {
  if (current_ == NULL)
    return;

  // Handle data based on the command
  switch (header[2]) {
    case COMMAND_BMS_VERSION:
      // BMS Version
      current_->bms_version_text_sensor_->publish_state(std::string(reinterpret_cast<const char *>(data), length));
      ESP_LOGD(TAG, "Address: %d BMS Version: %s", current_->address_, current_->bms_version_text_sensor_->get_state().c_str());
      break;
    case COMMAND_BARCODE:
      // Barcode
      current_->barcode_text_sensor_->publish_state(std::string(reinterpret_cast<const char *>(data), length));
      ESP_LOGD(TAG, "Address: %d Barcode: %s", current_->address_, current_->barcode_text_sensor_->get_state().c_str());
      break;
    case COMMAND_INFO:
      handle_info(data, length);
      // All data has been updated, mark the device as updated and connected
      current_->state_ = BasenBMS::BMS_STATE_PUBLISH;
      if (!current_->connected_binary_sensor_->state)
        current_->connected_binary_sensor_->publish_state(true);
      current_ = NULL;
      break;
    default:
      ESP_LOGW(TAG, "Unknown command: %02X", header[2]);
      break;
  }
}

void BasenBMS::update()
{
  if (this->state_ == BMS_STATE_IDLE) {
    this->state_ = BMS_STATE_WANT_UPDATE;  // Set update flag
  }
}

void BasenBMS::publish_status()
{
  // Create a string containing all bytes
  char status_bitmask[40] = {0};

  for (uint8_t i = 0; i < sizeof (this->status_bitmask_); i++) {
    sprintf (status_bitmask + i * 3, "%02X:", this->status_bitmask_[i]);
  }

  status_bitmask[sizeof (this->status_bitmask_)*3 - 1] = '\0';
  this->status_bitmask_sensor_->publish_state(status_bitmask);
}

void BasenBMS::publish()
{
  if (this->state_ != BMS_STATE_PUBLISH)
    return;

  // Publish sensors in blocks to reduce loop time
  switch (this->publish_count_) {
    case 0:
      voltage_sensor_->publish_state(this->voltage_);
      current_sensor_->publish_state(this->current_);
      power_sensor_->publish_state(this->voltage_ * this->current_);
      capacity_sensor_->publish_state(this->capacity_);
      soc_sensor_->publish_state(this->soc_);
      soh_sensor_->publish_state(this->soh_);
      cycles_sensor_->publish_state(this->cycles_);
      this->publish_count_++;
      break;
    case 1:
      avg_cell_voltage_sensor_->publish_state(this->cell_avg_voltage_);
      min_cell_voltage_sensor_->publish_state(this->cell_min_voltage_);
      max_cell_voltage_sensor_->publish_state(this->cell_max_voltage_);
      min_cell_index_sensor_->publish_state(this->cell_min_index_);
      max_cell_index_sensor_->publish_state(this->cell_max_index_);
      delta_cell_voltage_sensor_->publish_state(this->cell_max_voltage_ - this->cell_min_voltage_);
      this->publish_count_++;
      break;
    case 2:
      temperature_sensor_[0]->publish_state(this->temperature_[0]);
      temperature_sensor_[1]->publish_state(this->temperature_[1]);
      temperature_sensor_[2]->publish_state(this->temperature_[2]);
      temperature_sensor_[3]->publish_state(this->temperature_[3]);
      temperature_sensor_[4]->publish_state(this->temperature_mos_);
      temperature_sensor_[5]->publish_state(this->temperature_ambient_);
      publish_status();
      this->publish_count_++;
      break;
    default:
      this->publish_count_ = 0;
      this->state_ = BMS_STATE_DONE;
      break;
  }
}

}  // namespace basen
}  // namespace esphome
