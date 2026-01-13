#include "param.h"

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Dense>

// Modbus TCP client for Inspire Hand
class ModbusTCPClient
{
public:
  using SharedPtr = std::shared_ptr<ModbusTCPClient>;

  ModbusTCPClient(const std::string& ip, int port = 6000)
    : ip_(ip), port_(port), sock_(-1), transaction_id_(0)
  {
    connect();
  }

  ~ModbusTCPClient()
  {
    if (sock_ >= 0)
      close(sock_);
  }

  bool connect()
  {
    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0)
    {
      std::cerr << "Failed to create socket for " << ip_ << std::endl;
      return false;
    }

    // Set socket timeout
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000; // 50ms timeout
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    inet_pton(AF_INET, ip_.c_str(), &server_addr.sin_addr);

    if (::connect(sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
      std::cerr << "Failed to connect to " << ip_ << ":" << port_ << std::endl;
      close(sock_);
      sock_ = -1;
      return false;
    }

    std::cout << "Connected to Inspire Hand at " << ip_ << ":" << port_ << std::endl;
    return true;
  }

  bool isConnected() const { return sock_ >= 0; }

  // Write multiple registers (Function Code 0x10)
  // Register 1486-1497: Set value of the angle for each DOF (6 x int16)
  bool writeAngle(const Eigen::Matrix<double, 6, 1>& angles)
  {
    if (!isConnected()) return false;

    std::lock_guard<std::mutex> lock(mutex_);

    // Convert angles (0-1 range) to hand units (0-1000 for position or angle value)
    // According to manual: ANGLE_SET registers are int16, range -32768 to 32767
    // Representing angle in 0.01 degree units
    // But for compatibility with existing code, we'll use 0-1000 mapping

    uint16_t angle_values[6];
    for (int i = 0; i < 6; i++)
    {
      // Clamp to 0-1 range, then scale to 0-1000
      double val = std::max(0.0, std::min(1.0, angles(i)));
      angle_values[i] = static_cast<uint16_t>(val * 1000);
    }

    // Build Modbus TCP frame for Write Multiple Registers (0x10)
    // Register address: 1486 (ANGLE_SET(0))
    uint8_t request[25];
    uint16_t tid = transaction_id_++;

    // MBAP Header
    request[0] = (tid >> 8) & 0xFF;      // Transaction ID high
    request[1] = tid & 0xFF;              // Transaction ID low
    request[2] = 0x00;                    // Protocol ID high (Modbus)
    request[3] = 0x00;                    // Protocol ID low
    request[4] = 0x00;                    // Length high
    request[5] = 0x13;                    // Length low (19 bytes follow)

    // PDU
    request[6] = 0x01;                    // Unit ID (hand ID, typically 1)
    request[7] = 0x10;                    // Function code: Write Multiple Registers
    request[8] = (1486 >> 8) & 0xFF;     // Start address high
    request[9] = 1486 & 0xFF;            // Start address low
    request[10] = 0x00;                   // Quantity high
    request[11] = 0x06;                   // Quantity low (6 registers)
    request[12] = 0x0C;                   // Byte count (12 bytes = 6 x 16-bit)

    // Register values (big-endian for Modbus)
    for (int i = 0; i < 6; i++)
    {
      request[13 + i*2] = (angle_values[i] >> 8) & 0xFF;
      request[14 + i*2] = angle_values[i] & 0xFF;
    }

    ssize_t sent = send(sock_, request, 25, 0);
    if (sent != 25) return false;

    // Read response (12 bytes for write multiple response)
    uint8_t response[12];
    ssize_t received = recv(sock_, response, 12, 0);

    return received == 12 && response[7] == 0x10;
  }

  // Read holding registers (Function Code 0x03)
  // Register 1546-1557: Actual angle for each DOF (6 x int16)
  bool readAngle(Eigen::Matrix<double, 6, 1>& angles)
  {
    if (!isConnected()) return false;

    std::lock_guard<std::mutex> lock(mutex_);

    // Build Modbus TCP frame for Read Holding Registers (0x03)
    uint8_t request[12];
    uint16_t tid = transaction_id_++;

    // MBAP Header
    request[0] = (tid >> 8) & 0xFF;
    request[1] = tid & 0xFF;
    request[2] = 0x00;
    request[3] = 0x00;
    request[4] = 0x00;
    request[5] = 0x06;                    // Length (6 bytes follow)

    // PDU
    request[6] = 0x01;                    // Unit ID
    request[7] = 0x03;                    // Function code: Read Holding Registers
    request[8] = (1546 >> 8) & 0xFF;     // Start address high (ANGLE(0))
    request[9] = 1546 & 0xFF;            // Start address low
    request[10] = 0x00;                   // Quantity high
    request[11] = 0x06;                   // Quantity low (6 registers)

    ssize_t sent = send(sock_, request, 12, 0);
    if (sent != 12) return false;

    // Read response (9 header + 12 data bytes)
    uint8_t response[21];
    ssize_t received = recv(sock_, response, 21, 0);

    if (received != 21 || response[7] != 0x03)
      return false;

    // Parse register values
    for (int i = 0; i < 6; i++)
    {
      int16_t raw = (response[9 + i*2] << 8) | response[10 + i*2];
      // Convert from hand units to 0-1 range
      angles(i) = std::max(0.0, std::min(1.0, raw / 1000.0));
    }

    return true;
  }

private:
  std::string ip_;
  int port_;
  int sock_;
  uint16_t transaction_id_;
  std::mutex mutex_;
};


class InspireRunner
{
public:
  InspireRunner(const std::string& left_ip, const std::string& right_ip)
  {
    // Connect to hands via TCP Modbus
    lefthand = std::make_shared<ModbusTCPClient>(left_ip, 6000);
    righthand = std::make_shared<ModbusTCPClient>(right_ip, 6000);

    if (!lefthand->isConnected())
      std::cerr << "Warning: Left hand not connected!" << std::endl;
    if (!righthand->isConnected())
      std::cerr << "Warning: Right hand not connected!" << std::endl;

    // DDS
    handcmd = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(
        "rt/" + param::ns + "/cmd");
    handcmd->msg_.cmds().resize(12);
    handstate = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(
        "rt/" + param::ns + "/state");
    handstate->msg_.states().resize(12);

    // Initialize state
    qcmd.setZero();
    qstate.setZero();

    // Start running
    thread = std::make_shared<unitree::common::RecurrentThread>(
      10000, std::bind(&InspireRunner::run, this)  // 100Hz
    );
  }

  void run()
  {
    // Set command
    if(!handcmd->isTimeout())
    {
      for(int i = 0; i < 12; i++)
      {
        qcmd(i) = handcmd->msg_.cmds()[i].q();
      }

      // Right hand: indices 0-5, Left hand: indices 6-11
      righthand->writeAngle(qcmd.block<6, 1>(0, 0));
      lefthand->writeAngle(qcmd.block<6, 1>(6, 0));
    }

    // Read state
    Eigen::Matrix<double, 6, 1> qtemp;

    if(righthand->readAngle(qtemp))
    {
      qstate.block<6, 1>(0, 0) = qtemp;
    }
    else
    {
      for(int i = 0; i < 6; i++)
        handstate->msg_.states()[i].lost()++;
    }

    if(lefthand->readAngle(qtemp))
    {
      qstate.block<6, 1>(6, 0) = qtemp;
    }
    else
    {
      for(int i = 0; i < 6; i++)
        handstate->msg_.states()[i+6].lost()++;
    }

    // Publish state
    if(handstate->trylock())
    {
      for(int i = 0; i < 12; i++)
      {
        handstate->msg_.states()[i].q() = qstate(i);
      }
      handstate->unlockAndPublish();
    }
  }

  unitree::common::ThreadPtr thread;

  // TCP Modbus clients
  ModbusTCPClient::SharedPtr lefthand;
  ModbusTCPClient::SharedPtr righthand;
  Eigen::Matrix<double, 12, 1> qcmd, qstate;

  // DDS
  std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> handstate;
  std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> handcmd;
};


int main(int argc, char** argv)
{
  // Default IPs for Inspire hands
  std::string left_ip = "192.168.123.210";
  std::string right_ip = "192.168.123.211";

  // Parse command line arguments
  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];
    if (arg == "--left" && i + 1 < argc)
      left_ip = argv[++i];
    else if (arg == "--right" && i + 1 < argc)
      right_ip = argv[++i];
    else if (arg == "-h" || arg == "--help")
    {
      std::cout << "Usage: " << argv[0] << " [options]\n"
                << "Options:\n"
                << "  --left IP    Left hand IP (default: 192.168.123.210)\n"
                << "  --right IP   Right hand IP (default: 192.168.123.211)\n";
      return 0;
    }
  }

  auto vm = param::helper(argc, argv);
  unitree::robot::ChannelFactory::Instance()->Init(0, param::network);

  std::cout << " --- Unitree Robotics --- " << std::endl;
  std::cout << "  Inspire Hand Controller (TCP Modbus)  " << std::endl;
  std::cout << "  Left Hand:  " << left_ip << ":6000" << std::endl;
  std::cout << "  Right Hand: " << right_ip << ":6000" << std::endl;

  InspireRunner runner(left_ip, right_ip);

  while (true)
  {
    sleep(1);
  }
  return 0;
}

