#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanChecker : public rclcpp::Node {
public:
  ScanChecker() : Node("scan_checker") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ScanChecker::scan_callback, this, std::placeholders::_1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    size_t scan_size = msg->ranges.size();
    if (scan_size > 330) {
      float front_value = msg->ranges[330];
      RCLCPP_INFO(this->get_logger(),
                  "Scan size: %zu | Front value [330]: %.2f", scan_size,
                  front_value);
    } else {
      RCLCPP_WARN(this->get_logger(), "Scan size is too small: %zu", scan_size);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanChecker>());
  rclcpp::shutdown();
  return 0;
}
