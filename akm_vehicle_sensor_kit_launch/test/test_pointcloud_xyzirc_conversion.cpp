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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

sensor_msgs::msg::PointField field(
  const std::string & name, const std::uint32_t offset, const std::uint8_t datatype)
{
  sensor_msgs::msg::PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1;
  return result;
}

template <typename T>
void write_value(std::vector<std::uint8_t> & data, const std::size_t offset, const T value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

template <typename T>
T read_value(const std::vector<std::uint8_t> & data, const std::size_t offset)
{
  T value{};
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

sensor_msgs::msg::PointCloud2 make_input_cloud()
{
  sensor_msgs::msg::PointCloud2 input;
  input.header.frame_id = "velodyne_top";
  input.height = 1;
  input.width = 2;
  input.fields = {
    field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
    field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
    field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
    field("intensity", 16, sensor_msgs::msg::PointField::FLOAT32),
    field("ring", 20, sensor_msgs::msg::PointField::UINT16),
    field("time", 24, sensor_msgs::msg::PointField::FLOAT32),
  };
  input.is_bigendian = false;
  input.point_step = 32;
  input.row_step = input.point_step * input.width;
  input.data.resize(input.row_step);
  input.is_dense = true;

  write_value(input.data, 0, 1.0F);
  write_value(input.data, 4, 2.0F);
  write_value(input.data, 8, 3.0F);
  write_value(input.data, 16, 12.6F);
  write_value<std::uint16_t>(input.data, 20, 7U);

  const auto second = static_cast<std::size_t>(input.point_step);
  write_value(input.data, second + 0, -1.0F);
  write_value(input.data, second + 4, -2.0F);
  write_value(input.data, second + 8, -3.0F);
  write_value(input.data, second + 16, 400.0F);
  write_value<std::uint16_t>(input.data, second + 20, 31U);
  return input;
}

TEST(PointCloudXyzircConversion, ProducesStrictLayoutAndValues)
{
  const auto output = akm_vehicle_sensor_kit_launch::convert_to_xyzirc(make_input_cloud(), 2U);

  ASSERT_EQ(output.fields.size(), 6U);
  EXPECT_EQ(output.point_step, 16U);
  EXPECT_EQ(output.row_step, 32U);
  EXPECT_EQ(output.header.frame_id, "velodyne_top");

  const std::vector<std::string> names{"x", "y", "z", "intensity", "return_type", "channel"};
  const std::vector<std::uint32_t> offsets{0U, 4U, 8U, 12U, 13U, 14U};
  for (std::size_t index = 0; index < names.size(); ++index) {
    EXPECT_EQ(output.fields.at(index).name, names.at(index));
    EXPECT_EQ(output.fields.at(index).offset, offsets.at(index));
  }

  EXPECT_FLOAT_EQ(read_value<float>(output.data, 0), 1.0F);
  EXPECT_EQ(read_value<std::uint8_t>(output.data, 12), 13U);
  EXPECT_EQ(read_value<std::uint8_t>(output.data, 13), 2U);
  EXPECT_EQ(read_value<std::uint16_t>(output.data, 14), 7U);

  EXPECT_FLOAT_EQ(read_value<float>(output.data, 16), -1.0F);
  EXPECT_EQ(read_value<std::uint8_t>(output.data, 28), 255U);
  EXPECT_EQ(read_value<std::uint16_t>(output.data, 30), 31U);
}

TEST(PointCloudXyzircConversion, RejectsMissingRingField)
{
  auto input = make_input_cloud();
  input.fields.erase(input.fields.begin() + 4);
  EXPECT_THROW(akm_vehicle_sensor_kit_launch::convert_to_xyzirc(input, 1U), std::runtime_error);
}

}  // namespace
