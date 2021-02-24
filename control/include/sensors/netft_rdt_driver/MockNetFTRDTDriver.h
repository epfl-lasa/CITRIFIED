
#pragma once

#include "sensors/netft_rdt_driver/INetFTRDTDriver.h"

namespace netft_rdt_driver {

class MockNetFTRDTDriver : public INetFTRDTDriver {
public:
  explicit MockNetFTRDTDriver();

  void getData(RawWrenchMessage& data) override;

  bool waitForNewData() override;
};
}