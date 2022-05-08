// MIT License
//
// Copyright (c) 2022 Avery Girven
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <ros2_sick/LMS1xx/LMS1xx.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>

#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "string"

namespace Sick
{
class Sick : public rclcpp::Node
{
  public:
  explicit Sick(rclcpp::NodeOptions options);
  ~Sick();

  /**
    * @brief connect to the Sick LMS1xx lidar
    */
  void connect_lidar();

  private:
  // laser data
  LMS1xx laser;
  scanCfg cfg;
  scanOutputRange outputRange;
  scanDataCfg dataCfg;

  // parameters
  std::string host{};
  std::string frame_id{};
  int port{2112};
  bool tf_correction{true};
  int reconnect_timeout{0};
  sensor_msgs::msg::LaserScan scan_msg;
  laser_geometry::LaserProjection projector;

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr ls_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;

  /**
    * @brief construct a scan message based on the
    * configuration of the sick lidar
    * @param scan sensor_msgs::msg::laserscan
    */
  void construct_scan();

  /**
    * @brief get measurements from the lidar after it has 
    * been setup properly
    */
  void get_measurements();

  /**
    * @brief publishes scan messages
    */
  void publish_scan();

  /**
    * @brief publishes cloud messages
    */
  void publish_cloud();
};

}  // namespace Sick