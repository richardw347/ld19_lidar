#ifndef INCLUDE_LD19_LIDAR_LD19_NODE
#define INCLUDE_LD19_LIDAR_LD19_NODE

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "lipkg.h"
#include "async_serial.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <boost/system/system_error.hpp>

/*Fixed parameters of the sensor*/
static const float ANGLE_MIN = -M_PI;
static const float ANGLE_MAX = M_PI;
static const float ANGLE_INCREMENT_DEFAULT = 0.0139;
static const float READING_COUNT = std::ceil((ANGLE_MAX - ANGLE_MIN) / ANGLE_INCREMENT_DEFAULT);
static const float RANGE_MIN = 0.03;
static const float RANGE_MAX = 12.0;
static const uint32_t BAUDRATE = 230400;

class LD19Node : public rclcpp::Node
{
public:
  LD19Node();
  auto populate_message(const std::vector<PointData>& laser_data) -> void;
  auto init_device() -> bool;
  auto timer_callback() -> void;
  auto init_parameters() -> void;

private:
  static auto map_range(float x, float in_min, float in_max, float out_min, float out_max) -> float
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  static auto wrap_angle(float angle) -> float
  {
    if (angle > M_PI)
    {
      angle -= (M_PI * 2);
    }
    else if (angle < -M_PI)
    {
      angle += (M_PI * 2);
    }
    return angle;
  }
  std::shared_ptr<LiPkg> lidar_;
  std::shared_ptr<CallbackAsyncSerial> serial_port_;
  std::string port_;
  std::string frame_id_;
  std::string topic_name_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  sensor_msgs::msg::LaserScan output_;
};

#endif /* INCLUDE_LD19_LIDAR_LD19_NODE */
