
#include "sensors/NetFTRDTDriver.h"

#include <cstdint>
#include <iostream>
#include <exception>

#include "sensors/netft_rdt_communication.h"

using boost::asio::ip::udp;

namespace netft_rdt_driver {

NetFTRDTDriver::NetFTRDTDriver(const std::string& address) :
    address_(address),
    socket_(ioService_),
    stopRecvThread_(false),
    recvThreadRunning_(false),
    packetCount_(0),
    lostPackets_(0),
    outOfOrderCount_(0),
    seqCounter_(0),
//    diag_packet_count_(0),
//    last_diag_pub_time_(std::chrono::system_clock::now()),
    lastRDTSequence_(0),
    systemStatus_(0) {
  // Construct UDP socket
  // TODO RDT_PORT ?!
  udp::endpoint netftEndpoint(boost::asio::ip::address_v4::from_string(address), RDT_PORT);
  socket_.open(udp::v4());
  socket_.connect(netftEndpoint);

  // TODO : Get/Set Force/Torque scale for device
  // Force/Sclae is based on counts per force/torque value from device
  // these value are manually read from device webserver, but in future they
  // may be collected using http get requests
  static const double countsPerForce = 1000000;
  static const double countsPerTorque = 1000000;
  forceScale_ = 1.0 / countsPerForce;
  torqueScale_ = 1.0 / countsPerTorque;

  // Start receive thread
  recvThread_ = boost::thread(&NetFTRDTDriver::recvThreadFunc, this);

  // Since start steaming command is sent with UDP packet,
  // the packet could be lost, retry startup 10 times before giving up
  for (int i = 0; i < 10; ++i) {
    startStreaming();
    if (waitForNewData()) {
      break;
    }
  }
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    if (packetCount_ == 0) {
      throw std::runtime_error("No data received from NetFT device");
    }
  }

}

NetFTRDTDriver::~NetFTRDTDriver() {
  // TODO stop transmission,
  // stop thread
  stopRecvThread_ = true;
  if (!recvThread_.timed_join(boost::posix_time::time_duration(0, 0, 1, 0))) {
    std::cerr << "Interrupting recv thread" << std::endl;
    recvThread_.interrupt();
    if (!recvThread_.timed_join(boost::posix_time::time_duration(0, 0, 1, 0))) {
      std::cerr << "Failed second join to recv thread" << std::endl;
    }
  }
  socket_.close();
}

bool NetFTRDTDriver::waitForNewData() {
  // Wait upto 100ms for new data
  bool gotNewData = false;
  {
    boost::mutex::scoped_lock lock(mutex_);
    unsigned currentPacketCount = packetCount_;
    condition_.timed_wait(lock, boost::posix_time::milliseconds(100));
    gotNewData = packetCount_ != currentPacketCount;
  }

  return gotNewData;
}

void NetFTRDTDriver::startStreaming() {
  // Command NetFT to start data transmission
  RDTCommand startTransmission;
  startTransmission.command_ = RDTCommand::CMD_START_HIGH_SPEED_STREAMING;
  startTransmission.sampleCount_ = RDTCommand::INFINITE_SAMPLES;
  // TODO change buffer into boost::array
  uint8_t buffer[RDTCommand::RDT_COMMAND_SIZE];
  startTransmission.pack(buffer);
  socket_.send(boost::asio::buffer(buffer, RDTCommand::RDT_COMMAND_SIZE));
}

void NetFTRDTDriver::recvThreadFunc() {
  try {
    recvThreadRunning_ = true;
    RDTMessage rdtMessage;
    RawWrenchMessage tmpData;
    uint8_t buffer[RDTMessage::RDT_MESSAGE_SIZE + 1];
    while (!stopRecvThread_) {
      size_t len = socket_.receive(boost::asio::buffer(buffer,
                                                       RDTMessage::RDT_MESSAGE_SIZE + 1));
      if (len != RDTMessage::RDT_MESSAGE_SIZE) {
        printf("Receive size of %d bytes does not match expected size of %d",
               int(len), int(RDTMessage::RDT_MESSAGE_SIZE));
      } else {
        rdtMessage.unpack(buffer);
        if (rdtMessage.status_ != 0) {
          // Latch any system status error code
          boost::unique_lock<boost::mutex> lock(mutex_);
          systemStatus_ = rdtMessage.status_;
        }
        auto seqdiff = int32_t(rdtMessage.rdtSequence_ - lastRDTSequence_);
        lastRDTSequence_ = rdtMessage.rdtSequence_;
        if (seqdiff < 1) {
          boost::unique_lock<boost::mutex> lock(mutex_);
          // Don't use data that is old
          ++outOfOrderCount_;
        } else {
          tmpData.time = std::chrono::system_clock::now();
          tmpData.force.x = double(rdtMessage.fx_) * forceScale_;
          tmpData.force.y = double(rdtMessage.fy_) * forceScale_;
          tmpData.force.z = double(rdtMessage.fz_) * forceScale_;
          tmpData.torque.x = double(rdtMessage.tx_) * torqueScale_;
          tmpData.torque.y = double(rdtMessage.ty_) * torqueScale_;
          tmpData.torque.z = double(rdtMessage.tz_) * torqueScale_;
          {
            boost::unique_lock<boost::mutex> lock(mutex_);
            newData_ = tmpData;
            lostPackets_ += (seqdiff - 1);
            ++packetCount_;
            condition_.notify_all();
          }
        }
      }
    } // end while
  } catch (std::exception& e) {
    recvThreadRunning_ = false;
    {
      boost::unique_lock<boost::mutex> lock(mutex_);
      recvThreadErrorMsg_ = e.what();
    }
  }
}

void NetFTRDTDriver::getData(RawWrenchMessage& data) {
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    data = newData_;
  }
}

//void NetFTRDTDriver::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& d) {
//  // Publish diagnostics
//  d.name = "NetFT RDT Driver : " + address_;
//
//  d.summary(d.OK, "OK");
//  d.hardware_id = "0";
//
//  if (diag_packet_count_ == packet_count_) {
//    d.mergeSummary(d.ERROR, "No new data in last second");
//  }
//
//  if (!recv_thread_running_) {
//    d.mergeSummaryf(d.ERROR, "Receive thread has stopped : %s",
//                    recv_thread_error_msg_.c_str());
//  }
//
//  if (system_status_ != 0) {
//    d.mergeSummaryf(d.ERROR, "NetFT reports error 0x%08x", system_status_);
//  }
//
//  ros::Time current_time(ros::Time::now());
//  double recv_rate = double(int32_t(packet_count_ - diag_packet_count_)) /
//      (current_time - last_diag_pub_time_).toSec();
//
//  d.clear();
//  d.addf("IP Address", "%s", address_.c_str());
//  d.addf("System status", "0x%08x", system_status_);
//  d.addf("Good packets", "%u", packet_count_);
//  d.addf("Lost packets", "%u", lost_packets_);
//  d.addf("Out-of-order packets", "%u", out_of_order_count_);
//  d.addf("Recv rate (pkt/sec)", "%.2f", recv_rate);
//  d.addf("Force scale (N/bit)", "%f", force_scale_);
//  d.addf("Torque scale (Nm/bit)", "%f", torque_scale_);
//
//  geometry_msgs::WrenchStamped data;
//  getData(data);
//  d.addf("Force X (N)", "%f", data.wrench.force.x);
//  d.addf("Force Y (N)", "%f", data.wrench.force.y);
//  d.addf("Force Z (N)", "%f", data.wrench.force.z);
//  d.addf("Torque X (Nm)", "%f", data.wrench.torque.x);
//  d.addf("Torque Y (Nm)", "%f", data.wrench.torque.y);
//  d.addf("Torque Z (Nm)", "%f", data.wrench.torque.z);
//
//  last_diag_pub_time_ = current_time;
//  diag_packet_count_ = packet_count_;
//}
}
