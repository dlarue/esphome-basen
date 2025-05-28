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

static struct {
  uint8_t     offset;
  uint8_t     bit;
  const char  *message;
} status_messages[] = {
  {0, 0x08, "Cell voltage low fault"},
  {0, 0x10, "Voltage line break"},
  {0, 0x20, "Charge MOS fault"},
  {0, 0x40, "Dischare MOS fault"},
  {0, 0x80, "Voltage sensor fault"},
  {1, 0x01, "NTC disconnection"},
  {1, 0x02, "ADC MOD fault"},
  {1, 0x02, "ADC MOD fault"},
  {1, 0x04, "Reverse battery"},
  {1, 0x08, "Hot failure"},
  {1, 0x10, "Battery locked"},
  {1, 0x20, "Communication timeout"},
  {1, 0x40, "SN code failure"},
  {1, 0x80, "Secondary trip protection"},
  {2, 0x01, "DISC_OV_TEMP_Protection"},
  {2, 0x02, "DISC_UN_TEMP_Protection"},
  {2, 0x40, "536_COM_Timeout"},
  {3, 0x01, "Charging"},
  {3, 0x02, "Discharging"},
  {3, 0x04, "Short Circuit Protection"},
  {3, 0x08, "Overcurrent Protection"},
  {3, 0x40, "CHG_OV_TEMP_Protection"},
  {3, 0x80, "CHG_UN_TEMP_Protection"},
  {4, 0x01, "Ambient_Low_TEMP_Protection"},
  {4, 0x02, "ENV_High_TEMP_Protection"},
  {4, 0x10, "SOC Low protect"},
  {5, 0x01, "Manual_CHG_MOS_Open"},
  {5, 0x02, "Manual_CHG_MOS_Off"},
  {5, 0x04, "Manual_DISC_MOS_Open"},
  {5, 0x08, "Manual_DISC_MOS_Off"},
  {5, 0x10, "Heating pad"},
  {5, 0x20, "MOSFET_OV_TEMP_Protection"},
  {5, 0x40, "MOSFET_LO_TEMP_Protection"},
  {5, 0x80, "CHG_Open_TEMP_Too_Low"},
  {7, 0x01, "SOC_Low_Alarm2"},
  {7, 0x02, "Vibration Alarm"},
  {7, 0x04, "Waiting to recharge during a break"},
  {7, 0x08, "Aerosol fault"},
  {7, 0x10, "SOH Low Alarm"},
  {7, 0x20, "NTC short circuit"},
  {7, 0x40, "Temp_Diff Alarm"},
  {7, 0x80, "System Lock"},
  {8, 0x01, "AMB_Over_TEMP_Alarm"},
  {8, 0x02, "AMB_Low_TEMP_Alarm"},
  {8, 0x04, "MOS_Over_TEMP_Alarm"},
  {8, 0x08, "SOC_Low_Alarm"},
  {8, 0x10, "Vol_DIF_Alarm"},
  {8, 0x20, "BAT_DISC_Over_TEMP_Alarm"},
  {8, 0x40, "BAT_DISC_Low_TEMP_Alarm"},
  {9, 0x01, "Cell_Over_VOL_Alarm"},
  {9, 0x02, "Cell_Low_VOL_Alarm"},
  {9, 0x04, "Pack_Over_VOL_Alarm"},
  {9, 0x08, "Pack_Low_VOL_Alarm"},
  {9, 0x10, "CHG_Over_CUR_Alarm"},
  {9, 0x20, "DISC_Over_CURR_Alarm"},
  {9, 0x40, "BAT_CHG_Over_TEMP_Alarm"},
  {9, 0x80, "BAT_CHG_Low_TEMP_Alarm"},
};

void BasenController::dump_config() {
  ESP_LOGI(TAG, "Basen:");

  for (auto &device : this->devices_) {
    ESP_LOGI(TAG, " Device %02X:", device->address_);
    ESP_LOGI(TAG, "  Timeout count: %d", device->timeout_count_);
  }

  check_uart_settings(9600);
}

void BasenController::set_state(uint8_t state)
{
  if (state > STATE_RX_DATA) {
    ESP_LOGW(TAG, "Invalid state %d", state);
    return;
  }

  if (this->state_ == state)
    return;

  // Change state
  this->state_ = state;
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
  
  memcpy(this->header_, data, sizeof(header_));

  ESP_LOGV(TAG, "Sending command: %02X %02X %02X %02X %02X %02X", data[0], data[1], data[2], data[3], data[4], data[5]);

  uint32_t now = millis();
  this->last_command_ = now;
  BMS->last_transmission_ = now;

  set_state (STATE_COMMAND_TX);
}

void BasenController::check_timeout ()
{
  // Check for timeout while waiting for data
  uint32_t now = millis();
  uint32_t elapsed = now - current_->last_transmission_;
  if (elapsed > timeout_) {
    // Timeout while waiting for data.
    ESP_LOGW(TAG, "Timeout (%d ms) while waiting for data (%d < %d) for address %02X", elapsed, this->frame_size_, this->rx_required_, current_->address_);
    current_->timeout_count_++;
    current_->connected_binary_sensor_->publish_state(false);
    current_->state_ = BasenBMS::BMS_STATE_DONE;  // Mark as finished
    current_ = NULL;  // Reset current device
    set_state (STATE_BUS_CHECK);
  }
}

void BasenController::update_device ()
{
  if (current_ == NULL)
    return;

  // Keep minimum delay between commands
  uint32_t now = millis();
  if ((now - this->last_command_) < this->command_delay_) {
    // Wait for the next command
    return;
  }

  if (!current_->bms_version_text_sensor_->has_state()) {
    // Get BMS version
    this->send_command(current_, COMMAND_BMS_VERSION);
  } else if (!current_->barcode_text_sensor_->has_state()) {
    // Get barcode
    this->send_command(current_, COMMAND_BARCODE);
  } else if ((now-current_->last_transmission_) > 1000) {
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
        set_state (STATE_IDLE);
      }
      return;
    case STATE_IDLE:
      // Start sending commands
      this->update_device();
      return;
    case STATE_COMMAND_TX:
      // Command was sent, wait for response
      set_state (STATE_RX_HEADER);
      // Minimum 6 bytes response
      this->rx_required_ = 6;
      this->frame_size_ = 0;
      return;
    case STATE_RX_HEADER:
    case STATE_RX_DATA:
      if (current_ == NULL) {
        ESP_LOGW(TAG, "No device updating, cannot receive data");
        set_state (STATE_BUS_CHECK);
        return;
      }
      break;
  }

  // Check if we have received the data
  int available_bytes = available();
  if (available_bytes) {
    // Check if we have enough space in the frame buffer
    if (this->frame_size_ + available_bytes > sizeof(this->frame_)) {
      ESP_LOGW(TAG, "Frame buffer overflow, clearing RX buffer");
      set_state (STATE_BUS_CHECK);
      return;
    }

    // Copy available data to the frame buffer
    read_array(this->frame_ + this->frame_size_, available_bytes);
    this->frame_size_ += available_bytes;
  }

  // Check if we have enough data
  if (this->frame_size_ < this->rx_required_) {
    // No, check if we have a timeout
    check_timeout();
    return;
  }

  if (state_ == STATE_RX_HEADER) {
    ESP_LOGV(TAG, "Received header: %02X %02X %02X %02X", frame_[0], frame_[1], frame_[2], frame_[3]);

    if (frame_[0] != SOI || frame_[1] != this->header_[1] || frame_[2] != this->header_[2]) {
        ESP_LOGW(TAG, "Header does not match expected values: %02X %02X %02X != %02X %02X %02X", 
                 frame_[0], frame_[1], frame_[2], this->header_[0], this->header_[1], this->header_[2]);
        set_state (STATE_BUS_CHECK);
        return;
    }

    // Check length
    uint8_t length = frame_[3];

    if (length == 0x00) {
      // Command echo, verify EOI
      if (frame_[5] != EOI) {
        ESP_LOGW(TAG, "Invalid EOI");
        set_state (STATE_BUS_CHECK);
      }

      return;
    }

    if (length > 0xF0) {
      // Invalid length
      ESP_LOGW(TAG, "Invalid length");
      set_state (STATE_BUS_CHECK);
      return;
    }

    // Check if we have enough data
    this->rx_required_ = 4 + length + 2;  // Header + data + checksum + EOI

    set_state (STATE_RX_DATA);

    // Return here to keep loop time short
    return;
  }

  if (this->rx_required_ == 7) {
    // Only one data byte, most likely an error code:
    // 00H Normal
    // 01H VER Error
    // 02H CHKSUM Error
    // 03H LCHKSUM Error
    // 04H CID2 Invalid
    // 05H Command Format Error
    // 06H Invalid data
    ESP_LOGW(TAG, "Received error: %02X %02X %02X %02%", frame_[1], frame_[2], frame_[3], frame_[4]);
    set_state (STATE_BUS_CHECK);
    return;
  }

  // ESP_LOGD(TAG, "Received data: ");
  // for (size_t i = 0; i < this->data_required_; i++) {
  //   ESP_LOGD(TAG, "%02X ", data[i]);
  // }
  // ESP_LOGD(TAG, "");

  // Check EOI
  if (frame_[frame_size_ - 1] != EOI) {
    ESP_LOGW(TAG, "Invalid EOI byte: %02X", frame_[frame_size_ - 1]);
    set_state (STATE_BUS_CHECK);
    return;
  }

  uint8_t *data = frame_ + sizeof(header_);
  uint8_t data_size = frame_[3];

  // Verify checksum
  uint8_t checksum = this->checksum(frame_, sizeof(header_) + data_size);
  
  if (checksum != data[data_size]) {
    ESP_LOGW(TAG, "Checksum mismatch: %02X != %02X", checksum, data[data_size]);
    set_state (STATE_BUS_CHECK);
    return;
  }
  
  // Handle data
  handle_data (frame_, data, data_size);

  set_state (STATE_IDLE);
}

void BasenController::queue_device(BasenBMS *device) {
  // Check if we already have a device updating
  if (current_ != NULL)
    return;

  // No, set current device
  device->state_ = BasenBMS::BMS_STATE_UPDATING;
  current_ = device;
  ESP_LOGV(TAG, "Active device with address %02X", device->address_);
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
    set_state (STATE_BUS_CHECK);
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
    ESP_LOGV(TAG, "Cell %d voltage: %.3f V", i + 1, voltage_V);

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
        ESP_LOGV(TAG, "Data type %02X with count %d", type, count);
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

  // Create status message
  std::string status_message;

  for (uint8_t i = 0; i < sizeof(status_messages) / sizeof(status_messages[0]); i++) {
    if (this->status_bitmask_[status_messages[i].offset] & status_messages[i].bit) {
      if (!status_message.empty())
        status_message += ", ";
      status_message += status_messages[i].message;
    }
  }

  if (status_message.empty()) {
    status_message = "Idle";
  }

  this->status_sensor_->publish_state(status_message);
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
