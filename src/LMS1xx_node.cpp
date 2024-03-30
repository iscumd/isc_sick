#include "ros2_sick/LMS1xx_node.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include "ros2_sick/LMS1xx/LMS1xx.h"

constexpr double DEG2RAD = M_PI / 180.0;

namespace Sick
{
Sick::Sick(rclcpp::NodeOptions options) : Node("sick_node", options)
{
  host = this->declare_parameter("host", "192.168.1.100");
  frame_id = this->declare_parameter("frame_id", "laser_link");
  port = this->declare_parameter("port", 2112);
  tf_correction = this->declare_parameter("tf_correction", true);

  ls_publisher_ =
      this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

  pc_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/points", 10);

  this->connect_lidar();
}

void Sick::connect_lidar()
{
  //Attempt to connect to the lidar 5 times before giving up
  while (!laser.isConnected())
  {
    RCLCPP_INFO(this->get_logger(), "Connecting to Lidar at %s",
                this->host.c_str());
    laser.connect(host, port);

    if (laser.isConnected())
    {
      break;
    }

    RCLCPP_ERROR(this->get_logger(), "Unable to connect, retrying.");

    if (reconnect_timeout > 5)
    {
      RCLCPP_FATAL(this->get_logger(),
                   "Unable to connect to Lidar "
                   "after 5 attempts!");
      return;
    }
    else
    {
      reconnect_timeout++;
    }

    rclcpp::sleep_for(std::chrono::seconds(5));
  }

  RCLCPP_INFO(this->get_logger(), "Logging in to laser.");
  laser.login();
  cfg = laser.getScanCfg();
  outputRange = laser.getScanOutputRange();

  RCLCPP_INFO(this->get_logger(), "Connected to laser.");
  construct_scan();
  get_measurements();
}

void Sick::construct_scan()
{
  scan_msg.header.frame_id = frame_id;
  scan_msg.range_min = 0.01;
  scan_msg.range_max = 20.0;
  scan_msg.scan_time = 100.0 / cfg.scaningFrequency;
  scan_msg.angle_increment =
      ((double)outputRange.angleResolution / 2) / 10000.0 * DEG2RAD;
  if (tf_correction)
  {
    scan_msg.angle_min =
        ((((double)cfg.startAngle) / 10000.0) - 90.0) * DEG2RAD;
    scan_msg.angle_max = ((((double)cfg.stopAngle) / 10000.0) - 90.0) * DEG2RAD;
  }
  else
  {
    scan_msg.angle_min = ((double)cfg.startAngle) / 10000.0 * DEG2RAD;
    scan_msg.angle_max = ((double)cfg.stopAngle) / 10000.0 * DEG2RAD;
  }

  int num_values;

  if (cfg.angleResolution == 2500)
  {
    num_values = 1081;
  }
  else if (cfg.angleResolution == 5000)
  {
    num_values = 541;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Unsupported resolution");
    return;
  }

  scan_msg.ranges.resize(num_values);
  scan_msg.intensities.resize(num_values);

  scan_msg.time_increment = (outputRange.angleResolution / 10000.0) / 360.0 /
                            (cfg.scaningFrequency / 100.0);
}

void Sick::get_measurements()
{
  dataCfg.outputChannel = 1;
  dataCfg.outputChannel = 1;
  dataCfg.remission = true;
  dataCfg.resolution = 1;
  dataCfg.encoder = 0;
  dataCfg.position = false;
  dataCfg.deviceName = false;
  dataCfg.outputInterval = 1;

  RCLCPP_INFO(this->get_logger(), "Setting scan data configuration.");
  laser.setScanDataCfg(dataCfg);

  RCLCPP_INFO(this->get_logger(), "Starting scan measurement.");
  laser.startMeas();

  RCLCPP_INFO(this->get_logger(), "Waiting for ready status.");
  rclcpp::Time ready_status_timeout =
      this->get_clock()->now() + rclcpp::Duration::from_seconds(5);

  status_t stat = laser.queryStatus();

  // Spin until lidar is fully booted
  while (stat != ready_for_measurement)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Laser not ready. Retrying "
                 "initialization after 10 seconds.");

    rclcpp::sleep_for(std::chrono::seconds{10});

    stat = laser.queryStatus();
  }

  RCLCPP_INFO(this->get_logger(), "Starting device.");
  laser.startDevice();  // Log out to properly re-enable system after config

  RCLCPP_INFO(this->get_logger(), "Commanding continuous measurements.");
  laser.scanContinous(1);

  while (rclcpp::ok())
  {
    scan_msg.header.stamp = this->get_clock()->now();
    scanData data;
    if (laser.getScanData(data))
    {
      for (int i = 0; i < data.dist_len1; i++)
      {
        scan_msg.ranges[i] = data.dist1[i] * 0.001;
      }

      for (int i = 0; i < data.rssi_len1; i++)
      {
        scan_msg.intensities[i] = data.rssi1[i];
      }
      publish_scan();
      publish_cloud();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Laser timed out on delivering scan, "
                   "attempting to reinitialize.");
      break;
    }
  }
  laser.scanContinous(0);
  laser.stopMeas();
  laser.disconnect();
}

void Sick::publish_scan() { ls_publisher_->publish(scan_msg); }

void Sick::publish_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  projector.projectLaser(
      scan_msg,
      cloud_msg);  // perform a projection using laser_geometry
  pc_publisher_->publish(cloud_msg);
}

}  // namespace Sick

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto sick_node = std::make_shared<Sick::Sick>(options);
  exec.add_node(sick_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
