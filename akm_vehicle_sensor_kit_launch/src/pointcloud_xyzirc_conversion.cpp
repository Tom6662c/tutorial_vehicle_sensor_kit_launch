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

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace akm_vehicle_sensor_kit_launch
{

sensor_msgs::msg::PointCloud2 convert_to_xyzirc(
  const sensor_msgs::msg::PointCloud2 & input, const std::uint8_t default_return_type)
{
  sensor_msgs::msg::PointCloud2 output;
  output.header = input.header;
  output.height = input.height;
  output.width = input.width;
  output.is_bigendian = input.is_bigendian;
  output.is_dense = input.is_dense;

  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::UINT8, "return_type", 1, sensor_msgs::msg::PointField::UINT8,
    "channel", 1, sensor_msgs::msg::PointField::UINT16);
  modifier.resize(static_cast<std::size_t>(input.width) * input.height);

  try {
    sensor_msgs::PointCloud2ConstIterator<float> input_x(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> input_y(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> input_z(input, "z");
    sensor_msgs::PointCloud2ConstIterator<float> input_intensity(input, "intensity");
    sensor_msgs::PointCloud2ConstIterator<std::uint16_t> input_ring(input, "ring");
    sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> output_intensity(output, "intensity");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> output_return_type(output, "return_type");
    sensor_msgs::PointCloud2Iterator<std::uint16_t> output_channel(output, "channel");

    for (; input_x != input_x.end(); ++input_x, ++input_y, ++input_z, ++input_intensity,
                                     ++input_ring, ++output_x, ++output_y, ++output_z,
                                     ++output_intensity, ++output_return_type, ++output_channel) {
      *output_x = *input_x;
      *output_y = *input_y;
      *output_z = *input_z;
      const float intensity = std::isfinite(*input_intensity) ? *input_intensity : 0.0F;
      *output_intensity =
        static_cast<std::uint8_t>(std::lround(std::clamp(intensity, 0.0F, 255.0F)));
      *output_return_type = default_return_type;
      *output_channel = *input_ring;
    }
  } catch (const std::runtime_error & error) {
    throw std::runtime_error(std::string("invalid input pointcloud fields: ") + error.what());
  }

  return output;
}

}  // namespace akm_vehicle_sensor_kit_launch
