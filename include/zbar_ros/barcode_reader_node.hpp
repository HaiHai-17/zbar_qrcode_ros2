#ifndef ZBAR_ROS__BARCODE_READER_NODE_HPP_
#define ZBAR_ROS__BARCODE_READER_NODE_HPP_

#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "./zbar.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace zbar_ros
{

class BarcodeReaderNode : public rclcpp::Node
{
public:
  BarcodeReaderNode();

private:
  void imageCb(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void cleanCb();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr barcode_pub_;
  zbar::ImageScanner scanner_;

  rclcpp::TimerBase::SharedPtr clean_timer_;
  std::mutex memory_mutex_;
  std::unordered_map<std::string, rclcpp::Time> barcode_memory_;
  double throttle_;
};

}  // namespace zbar_ros

#endif  // ZBAR_ROS__BARCODE_READER_NODE_HPP_
