
#include "sensors/netft_rdt_driver/MockNetFTRDTDriver.h"

#include <iostream>

namespace netft_rdt_driver {

MockNetFTRDTDriver::MockNetFTRDTDriver() {
  std::cout << "This is a mock FT sensor that returns zero measurements." << std::endl;
}

bool MockNetFTRDTDriver::waitForNewData() {
  return true;
}

void MockNetFTRDTDriver::getData(RawWrenchMessage& data) {
  RawWrenchMessage newData;
  data = newData;
}
}
