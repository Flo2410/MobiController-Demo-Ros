#include "MobiCtl.hpp"

#include <functional>
#include <memory>
#include <string>

#include "chrono"
#include "libserial/SerialPort.h"
#include "min.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "vector"
#include "PayloadBuilder.hpp"

using LibSerial::BaudRate;
using LibSerial::DataBuffer;

MobiCtl::MobiCtl() : Node("mobictl") {
    
  MobiCtl::node = this;

#ifdef MOBICTL_DEBUG_PRINTING
  (void)rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
#endif

  RCLCPP_INFO(this->get_logger(), "Starting MobiCtl");
  RCLCPP_DEBUG(this->get_logger(), "DEBUG ENABLED!\n");
  
  this->publisher_ = this->create_publisher<std_msgs::msg::Float32>("topic", 10);

  this->serial_port.Open("/dev/ttyACM0");
  this->serial_port.SetBaudRate(BaudRate::BAUD_115200);

  min_init_context(&min_ctx, 0);  // Init min proto
  

  PayloadBuilder *pb = new PayloadBuilder();
  pb->append_uint16(50);
  min_queue_frame(&min_ctx, 35, pb->get_payload(), pb->size());
  delete pb;


  while (rclcpp::ok()) {
    auto read_count = this->serial_port.GetNumberOfBytesAvailable();
    DataBuffer data_buf;

    if (read_count == 0) {
      min_poll(&min_ctx, {}, 0);
      continue;
    }


    // RCLCPP_INFO(this->get_logger(), "Read Count: %d\n", read_count);
    this->serial_port.Read(data_buf, read_count, 100);
    min_poll(&min_ctx, data_buf.data(), data_buf.size());
  }
}

void MobiCtl::handle_min_frame(uint8_t min_id, uint8_t const *min_payload, uint8_t len_payload) {
  PayloadBuilder *pb = new PayloadBuilder(min_payload, len_payload);
  auto data = pb->read_float();
  RCLCPP_INFO(this->get_logger(), "Data: %f", data);
  delete pb;
  std_msgs::msg::Float32 msg = std_msgs::msg::Float32();
  msg.data = data;
  this->publisher_->publish(msg);
}



