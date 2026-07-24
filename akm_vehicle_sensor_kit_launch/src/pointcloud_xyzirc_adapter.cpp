// Copyright 2026 lglab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "akm_vehicle_sensor_kit_launch/pointcloud_xyzirc_conversion.hpp"

#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace akm_vehicle_sensor_kit_launch
{

class PointCloudXyzircAdapter : public rclcpp::Node
{
public:
  explicit PointCloudXyzircAdapter(const rclcpp::NodeOptions & options)
  : Node("pointcloud_xyzirc_adapter", options)
  {
    const auto return_type = declare_parameter<int64_t>("default_return_type", 1);
    if (return_type < 0 || return_type > 255) {
      throw std::invalid_argument("default_return_type must be in [0, 255]");
    }
    default_return_type_ = static_cast<std::uint8_t>(return_type);

    auto output_qos = rclcpp::SensorDataQoS();
    output_qos.keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", output_qos);
    cuda_publisher_ =
      std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
        *this, "cuda_output");
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr input) { convert(input); });
  }

private:
  void convert(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input)
  {
    try {
      auto output = convert_to_xyzirc(*input, default_return_type_);

      const auto cuda_subscriptions = cuda_publisher_->get_subscription_count() +
                                      cuda_publisher_->get_intra_process_subscription_count();
      if (cuda_subscriptions > 0U) {
        auto cuda_output = std::make_unique<cuda_blackboard::CudaPointCloud2>(output);
        cuda_publisher_->publish(std::move(cuda_output));
      }

      publisher_->publish(std::move(output));
      if (!has_published_) {
        RCLCPP_INFO(
          get_logger(), "Publishing adapted PointXYZIRC pointcloud: %ux%u, frame=%s", input->width,
          input->height, input->header.frame_id.c_str());
        has_published_ = true;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot convert input pointcloud: %s", error.what());
    }
  }

  std::uint8_t default_return_type_{};
  bool has_published_{false};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    cuda_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace akm_vehicle_sensor_kit_launch

RCLCPP_COMPONENTS_REGISTER_NODE(akm_vehicle_sensor_kit_launch::PointCloudXyzircAdapter)
