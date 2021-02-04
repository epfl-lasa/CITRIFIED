
#pragma once

namespace netft_rdt_driver {

struct RDTMessage {
  uint32_t rdtSequence_;
  uint32_t ftSequence_;
  uint32_t status_;
  int32_t fx_;
  int32_t fy_;
  int32_t fz_;
  int32_t tx_;
  int32_t ty_;
  int32_t tz_;

  enum {
    RDT_MESSAGE_SIZE = 36
  };
  void unpack(const uint8_t* buffer);
  static uint32_t unpack32(const uint8_t* buffer);
};

uint32_t RDTMessage::unpack32(const uint8_t* buffer) {
  return
      (uint32_t(buffer[0]) << 24) |
          (uint32_t(buffer[1]) << 16) |
          (uint32_t(buffer[2]) << 8) |
          (uint32_t(buffer[3]) << 0);
}

void RDTMessage::unpack(const uint8_t* buffer) {
  rdtSequence_ = unpack32(buffer + 0);
  ftSequence_ = unpack32(buffer + 4);
  status_ = unpack32(buffer + 8);
  fx_ = unpack32(buffer + 12);
  fy_ = unpack32(buffer + 16);
  fz_ = unpack32(buffer + 20);
  tx_ = unpack32(buffer + 24);
  ty_ = unpack32(buffer + 28);
  tz_ = unpack32(buffer + 32);
}

struct RDTCommand {
  uint16_t commandHeader_;
  uint16_t command_;
  uint32_t sampleCount_;

  RDTCommand() : commandHeader_(HEADER) {
    // empty
  }

  enum {
    HEADER = 0x1234
  };

  // Possible values for command_
  enum {
    CMD_STOP_STREAMING = 0,
    CMD_START_HIGH_SPEED_STREAMING = 2,
    // More command values are available but are not used by this driver
  };

  // Special values for sample count
  enum {
    INFINITE_SAMPLES = 0
  };

  enum {
    RDT_COMMAND_SIZE = 8
  };

  // Pack structure into buffer for network transport
  // Buffer should be RDT_COMMAND_SIZE
  void pack(uint8_t* buffer) const;
};

void RDTCommand::pack(uint8_t* buffer) const {
  // Data is big-endian
  buffer[0] = (commandHeader_ >> 8) & 0xFF;
  buffer[1] = (commandHeader_ >> 0) & 0xFF;
  buffer[2] = (command_ >> 8) & 0xFF;
  buffer[3] = (command_ >> 0) & 0xFF;
  buffer[4] = (sampleCount_ >> 8) & 0xFF;
  buffer[5] = (sampleCount_ >> 0) & 0xFF;
  buffer[6] = (sampleCount_ >> 8) & 0xFF;
  buffer[7] = (sampleCount_ >> 0) & 0xFF;
}
}