#ifndef MOBICTL_SENSORS_H_
#define MOBICTL_SENSORS_H_

#include <functional>
#include <memory>

#include "libserial/SerialPort.h"
#include "min.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "vector"

using LibSerial::SerialPort;

class MobiCtl : public rclcpp::Node {
 public:
  static MobiCtl* mobictl() { return node; }

  struct min_context min_ctx;
  std::vector<uint8_t> tx_queue;
  
  MobiCtl();
  void handle_min_frame(uint8_t min_id, uint8_t const *min_payload, uint8_t len_payload);

  SerialPort get_serial_port() { return this->serial_port; }

 private:
  static MobiCtl *node;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  SerialPort serial_port;
};

#endif