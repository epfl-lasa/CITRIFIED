
#pragma once

#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

#include <chrono>
#include <array>

//#include "diagnostic_updater/DiagnosticStatusWrapper.h"

namespace netft_rdt_driver {

struct Vec3D {
  Vec3D() : x(0), y(0), z(0) {}
  explicit Vec3D(std::array<double, 3> vec) : x(vec[0]), y(vec[1]), z(vec[2]) {}
  double x;
  double y;
  double z;
};

struct RawWrenchMessage {
  std::chrono::system_clock::time_point time = std::chrono::system_clock::now();
  Vec3D force = Vec3D(std::array<double, 3>{0, 0, 0});
  Vec3D torque = Vec3D(std::array<double, 3>{0, 0, 0});
};

class NetFTRDTDriver {
public:
  // Start receiving data from NetFT device
  explicit NetFTRDTDriver(const std::string& address, std::size_t timeout);

  ~NetFTRDTDriver();

  // Get newest RDT data from netFT device
  void getData(RawWrenchMessage& data);

  // Add device diagnostics status wrapper
//  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& d);

  // Wait for new NetFT data to arrive.
  // Returns true if new data has arrived, false it function times out
  bool waitForNewData(void);

protected:
  void recvThreadFunc(void);

  // Asks NetFT to start streaming data.
  void startStreaming(void);

  enum {
    RDT_PORT = 49152
  };
  std::string address_;
  std::size_t timeout_;

  boost::asio::io_service ioService_;
  boost::asio::ip::udp::socket socket_;
  boost::mutex mutex_;
  boost::thread recvThread_;
  boost::condition condition_;
  volatile bool stopRecvThread_;
  // True if recv loop is still running
  bool recvThreadRunning_;
  // Set if recv thread exited because of error
  std::string recvThreadErrorMsg_;

  // Newest data received from netft device
  RawWrenchMessage newData_;
  // Count number of received <good> packets
  unsigned packetCount_;
  // Count of lost RDT packets using RDT sequence number
  unsigned lostPackets_;
  // Counts number of out-of-order (or duplicate) received packets
  unsigned outOfOrderCount_;
  // Incremental counter for wrench header
  unsigned seqCounter_;

  // Scaling factor for converting raw force values from device into Newtons
  double forceScale_;
  // Scaling factor for converting raw torque values into Newton*meters
  double torqueScale_;

//  // Packet count last time diagnostics thread published output
//  unsigned diag_packet_count_;
//  // Last time diagnostics was published
//  std::chrono::system_clock::time_point last_diag_pub_time_;

  // to keep track of out-of-order or duplicate packet
  uint32_t lastRDTSequence_;
  // to keep track of any error codes reported by netft
  uint32_t systemStatus_;
};
}