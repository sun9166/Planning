#ifndef __MODULES_COMMON_CANET_TOOL_H__
#define __MODULES_COMMON_CANET_TOOL_H__

#include <iostream>
#include <string>
#include <unordered_map>

#include "can_protocol.pb.h"
#include "common/util/file.h"
#include "common/util/util.h"
// #include "google/protobuf/io/zero_copy_stream_impl.h"
// #include "google/protobuf/text_format.h"

namespace acu {
namespace common {
namespace util {

struct SignalParams {
  uint32_t start_bit;
  uint32_t length;
  float factor;
  float offset;
  int value;
};

enum class CanFormat { Motorola = -1, Intel = 1 };

using CanProtocolMap = std::unordered_map<std::string, SignalParams>;

class CANetTool {
public:
  CANetTool(CanFormat format) : format_(static_cast<int>(format)){};
  ~CANetTool(){};

public:
  void Init(const std::string &can_config_filename) {
    // Parse config file
    std::string path = " ";
    if (!acu::common::util::getPackagePath("proto", path)) {
      std::cout << "Can't locate proto package" << std::endl;
      exit(0);
    }

    CanProtocol can_protocol;
    std::string temp = path + "/../../../" + can_config_filename;
    if (!acu::common::util::GetProtoFromFile(temp, &can_protocol)) {
      std::cout << "Unable to parse CAN config file" << std::endl;
    }
    GenerateCANProtocolMap(can_protocol); 
  }

  bool GenerateCANProtocolMap(const CanProtocol &can_protocol) {
    SignalParams signal_struct;
    std::string name;
    for (const auto &can_frame : can_protocol.can_frame()) {
      for (const auto &signal : can_frame.signal_params()) {
        name = signal.name();
        signal_struct.start_bit = signal.start_bit();
        signal_struct.length = signal.length();
        signal_struct.factor = signal.factor();
        signal_struct.offset = signal.offset();
        signal_struct.value = signal.value();
        can_protocol_map_[name] = signal_struct;
      }
    }
  }

  void SetCanetHeader(const uint32_t id, uint8_t *&canet_send_buf) {
    canet_send_buf[0] = 0x08; // stand frame & 8 bytes for data frame
    canet_send_buf[1] = 0x00;
    canet_send_buf[2] = 0x00;
    canet_send_buf[3] = (uint8_t)(id >> 8);
    canet_send_buf[4] = (uint8_t)(0xFF & id);
  }

  void SetByte(const std::string name, uint8_t *&data) {
    int value = can_protocol_map_[name].value;
    SetByte(name, value, data);
  }

  template <typename T>
  void SetByte(const std::string name, T value, uint8_t *&data) {
    uint32_t start_byte = can_protocol_map_[name].start_bit / 8;
    uint32_t start_pos = can_protocol_map_[name].start_bit % 8;
    uint32_t length = can_protocol_map_[name].length;
    float factor = can_protocol_map_[name].factor;
    float offset = can_protocol_map_[name].offset;

    int value_int = (int)((value - offset) / factor);
    if ((start_pos + length) <= 8) {
      *(data + start_byte) |= (uint8_t)(value_int << start_pos);
    } else if ((start_pos + length) > 8 && (start_pos + length) <= 16) {
      *(data + start_byte) |= (uint8_t)(value_int << start_pos);
      *(data + start_byte + 1 * format_) |= (uint8_t)(value_int >> (8 - start_pos));
    } else if ((start_pos + length) > 16 && (start_pos + length) <= 24) {
      *(data + start_byte) |= (uint8_t)(value_int << start_pos);
      *(data + start_byte + 1 * format_) |= (uint8_t)(value_int >> (8 - start_pos));
      *(data + start_byte + 2 * format_) |= (uint8_t)(value_int >> (16 - start_pos));
    } else if ((start_pos + length) > 24) {
      *(data + start_byte) |= (uint8_t)(value_int << start_pos);
      *(data + start_byte + 1 * format_) |= (uint8_t)(value_int >> (8 - start_pos));
      *(data + start_byte + 2 * format_) |= (uint8_t)(value_int >> (16 - start_pos));
      *(data + start_byte + 3 * format_) |= (uint8_t)(value_int >> (24 - start_pos));
    }
  }

  template <typename T>
  void GetByte(const std::string name, T &var, uint8_t *&data) {
    uint32_t start_byte = can_protocol_map_[name].start_bit / 8;
    uint32_t start_pos = can_protocol_map_[name].start_bit % 8;
    uint32_t length = can_protocol_map_[name].length;
    float factor = can_protocol_map_[name].factor;
    float offset = can_protocol_map_[name].offset;

    if (length == 0) {
      std::cout << "Error length in CAN protocol!" << std::endl;
      exit(0);
    }
    int t0 = 0, t1 = 0, t2 = 0, t3 = 0;
    if ((start_pos + length) <= 8) {
      t0 = (int)((data[start_byte] >> start_pos) & RANG_MASK_1_L[length - 1]);
    } else if ((start_pos + length) > 8 && (start_pos + length) <= 16) {
      t0 = (int)((data[start_byte] >> start_pos) & RANG_MASK_1_L[7 - start_pos]);
      t1 = (int)(data[start_byte + 1 * format_] & RANG_MASK_1_L[start_pos + length - 9]);
      t1 <<= (8 - start_pos);
    } else if ((start_pos + length) > 16 && (start_pos + length) <= 24) {
      t0 = (int)((data[start_byte] >> start_pos) & RANG_MASK_1_L[7 - start_pos]);
      t1 = (int)data[start_byte + 1 * format_];
      t1 <<= (8 - start_pos);
      t2 = (int)(data[start_byte + 2 * format_] & RANG_MASK_1_L[length + start_pos - 17]);
      t2 <<= (16 - start_pos);
    } else if ((start_pos + length) > 24) {
      t0 = (int)((data[start_byte] >> start_pos) & RANG_MASK_1_L[7 - start_pos]);
      t1 = (int)data[start_byte + 1 * format_];
      t1 <<= (8 - start_pos);
      t2 = (int)data[start_byte + 2 * format_];
      t2 <<= (16 - start_pos);
      t3 = (int)(data[start_byte + 3 * format_] & RANG_MASK_1_L[length + start_pos - 25]);
      t3 <<= (24 - start_pos);
    }
    var = (T)((t0 + t1 + t2 + t3) * factor + offset);
  }

  CanProtocolMap &GetCanProtocolMap() { return can_protocol_map_; }

private:
  const uint8_t RANG_MASK_1_L[8] = {0x01, 0x03, 0x07, 0x0F, 
                                    0x1F, 0x3F, 0x7F, 0xFF};
  const uint8_t RANG_MASK_0_L[8] = {0xFF, 0xFE, 0XFC, 0xF8,
                                    0xF0, 0xE0, 0xC0, 0x80};

  int format_;  // Motorola[default] or Intel
  CanProtocolMap can_protocol_map_;

}; // class CANetTool

}  // namespace util
}  // namespace common
}  // namespace acu

#endif // __MODULES_COMMON_CANET_TOOL_H__