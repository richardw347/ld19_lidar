#include "ld19_node.hpp"

using namespace std::chrono_literals;

LD19Node::LD19Node(const std::string& node_name)
  : rclcpp_lifecycle::LifecycleNode(node_name)
  , port_("/dev/ttyUSB0")
  , frame_id_("laser")
  , topic_name_("scan")
  , output_()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LD19Node::on_configure(const rclcpp_lifecycle::State&)
{
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<std::string>("frame_id", "laser");
  this->declare_parameter<std::string>("topic_name", "scan");
  this->get_parameter("port", port_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("topic_name", topic_name_);

  output_.header.stamp = this->now();
  output_.header.frame_id = frame_id_;
  output_.angle_min = ANGLE_MIN;
  output_.angle_max = ANGLE_MAX;
  output_.range_min = RANGE_MIN;
  output_.range_max = RANGE_MAX;
  output_.angle_increment = output_.time_increment = 0.0;
  output_.scan_time = 0.0;
  output_.ranges.assign(READING_COUNT, 0.0);
  output_.intensities.assign(READING_COUNT, 0.0);

  lidar_ = std::make_shared<LiPkg>();

  try
  {
    serial_port_ = std::make_shared<CallbackAsyncSerial>(port_, BAUDRATE);
  }
  catch (const boost::system::system_error& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error opening device port: %s: %s", port_.c_str(), e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  serial_port_->setCallback([=](const char* byte, size_t len) {
    if (lidar_->Parse((uint8_t*)byte, len))
    {
      lidar_->AssemblePacket();
    }
  });

  lidar_->SetPopulateCallback(std::bind(&LD19Node::populate_message, this, std::placeholders::_1));

  if (!serial_port_->isOpen())
  {
    RCLCPP_ERROR(this->get_logger(), "Error opening device port: %s", port_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully opened device port: %s", port_.c_str());

  publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("topic_name_", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&LD19Node::timer_callback, this));
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LD19Node::on_activate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_activate(state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LD19Node::on_deactivate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_deactivate(state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LD19Node::on_cleanup(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_cleanup(state);
  lidar_.reset();
  serial_port_.reset();
  timer_.reset();
  publisher_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LD19Node::on_shutdown(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_shutdown(state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

auto LD19Node::populate_message(const std::vector<PointData>& laser_data) -> void
{
  /*Angle resolution, the smaller the resolution, the smaller the error after conversion*/
  float angle_increment = ANGLE_TO_RADIAN(lidar_->GetSpeed() / 4500);
  int max_index = std::ceil((ANGLE_MAX - ANGLE_MIN) / angle_increment);
  output_.header.stamp = this->now();
  output_.angle_increment = angle_increment;
  for (auto point : laser_data)
  {
    float range = point.distance / 1000.0;
    float angle = ANGLE_TO_RADIAN(point.angle);
    // reverse the index of the readings
    // for some reason the sensor is reading things
    // backwards
    int index = map_range(angle, 0, M_PI * 2.0, max_index, 0);

    // also adding a 90 degree offset here
    // as the 0 point of the sensor isn't the
    // front of the sensor
    int index_offset = (int)map_range(M_PI / 2, 0, M_PI * 2.0, 0, max_index);  // 113;
    index += index_offset;
    std::cout << "index_offset: " << index_offset << std::endl;

    if (index > max_index)
    {
      index -= max_index;
    }
    if (index < 0)
    {
      index += max_index;
    }

    // int index = (int)((angle - output_.angle_min) / output_.angle_increment);
    std::cout << "max index:" << max_index << std::endl;
    std::cout << "deg: " << point.angle << " rads: " << angle << " index: " << index << std::endl;
    if (index >= 0 && index < max_index)
    {
      output_.ranges[index] = range;
      /*If the current content is Nan, it is assigned directly*/
      if (std::isnan(output_.ranges[index]))
      {
        output_.ranges[index] = range;
      }
      else
      { /*Otherwise, only when the distance is less than the current value, it can be re assigned*/
        if (range < output_.ranges[index])
        {
          output_.ranges[index] = range;
        }
      }
      output_.intensities[index] = point.confidence;
    }
  }
}

auto LD19Node::timer_callback() -> void
{
  if (publisher_->is_activated())
  {
    if (lidar_->IsFrameReady())
    {
      publisher_->publish(output_);
      lidar_->ResetFrameReady();
    }
  }
}
