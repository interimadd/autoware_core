// Copyright 2021-2025 TIER IV
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

#include "crop_box_filter.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <geometry_msgs/msg/point32.hpp>

namespace autoware::crop_box_filter
{

CropBoxFilterCore::CropBoxFilterCore(CropBoxSize box_size) : _box_size(box_size)
{
}

bool CropBoxFilterCore::is_point_inside_box(const float x, const float y, const float z) const
{
  return (
    z > _box_size.min_z && z < _box_size.max_z &&
    y > _box_size.min_y && y < _box_size.max_y &&
    x > _box_size.min_x && x < _box_size.max_x);
}

PointCloud2 CropBoxFilterCore::extract_pointcloud_inside_box(const PointCloud2 input) const
{
  PointCloud2 output;
  output.header = input.header;
  output.is_bigendian = input.is_bigendian;
  output.is_dense = input.is_dense;
  output.point_step = input.point_step;
  output.height = 1;
  output.fields = input.fields;
  output.data.reserve(input.data.size());

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(input, "z");

  size_t point_index = 0;
  size_t output_size = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index) {
    if (is_point_inside_box(*iter_x, *iter_y, *iter_z))
    {
      const size_t src_offset = point_index * input.point_step;
      output.data.insert(
        output.data.end(), &input.data[src_offset], &input.data[src_offset + input.point_step]);
      output_size += input.point_step;
    }
  }

  output.data.shrink_to_fit();
  output.width = static_cast<uint32_t>(output.data.size() / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);

  return output;
}

PointCloud2 CropBoxFilterCore::extract_pointcloud_outside_box(const PointCloud2 input) const
{
  PointCloud2 output;
  output.header = input.header;
  output.is_bigendian = input.is_bigendian;
  output.is_dense = input.is_dense;
  output.point_step = input.point_step;
  output.height = 1;
  output.fields = input.fields;
  output.data.reserve(input.data.size());

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(input, "z");

  size_t point_index = 0;
  size_t output_size = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index) {
    if (!is_point_inside_box(*iter_x, *iter_y, *iter_z))
    {
      const size_t src_offset = point_index * input.point_step;
      output.data.insert(
        output.data.end(), &input.data[src_offset], &input.data[src_offset + input.point_step]);
      output_size += input.point_step;
    }
  }

  output.data.shrink_to_fit();
  output.width = static_cast<uint32_t>(output.data.size() / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);

  return output;
}

BoxPolygonCreator::BoxPolygonCreator(CropBoxSize box_size, std::string frame_id)
: _box_size(box_size), _frame_id(frame_id)
{
}

geometry_msgs::msg::PolygonStamped BoxPolygonCreator::create_crop_box_polygon_msg(
  const rclcpp::Time timestamp)
{
  geometry_msgs::msg::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = _frame_id;
  polygon_msg.header.stamp = timestamp;
  polygon_msg.polygon.points = generate_box_polygon();
  return polygon_msg;
}

std::vector<geometry_msgs::msg::Point32> BoxPolygonCreator::generate_box_polygon()
{
  const double x1 = _box_size.max_x;
  const double x2 = _box_size.min_x;
  const double x3 = _box_size.min_x;
  const double x4 = _box_size.max_x;

  const double y1 = _box_size.max_y;
  const double y2 = _box_size.max_y;
  const double y3 = _box_size.min_y;
  const double y4 = _box_size.min_y;

  const double z1 = _box_size.min_z;
  const double z2 = _box_size.max_z;

  std::vector<geometry_msgs::msg::Point32> polygon;
  polygon.push_back(generate_point(x1, y1, z1));
  polygon.push_back(generate_point(x2, y2, z1));
  polygon.push_back(generate_point(x3, y3, z1));
  polygon.push_back(generate_point(x4, y4, z1));
  polygon.push_back(generate_point(x1, y1, z1));

  polygon.push_back(generate_point(x1, y1, z2));

  polygon.push_back(generate_point(x2, y2, z2));
  polygon.push_back(generate_point(x2, y2, z1));
  polygon.push_back(generate_point(x2, y2, z2));

  polygon.push_back(generate_point(x3, y3, z2));
  polygon.push_back(generate_point(x3, y3, z1));
  polygon.push_back(generate_point(x3, y3, z2));

  polygon.push_back(generate_point(x4, y4, z2));
  polygon.push_back(generate_point(x4, y4, z1));
  polygon.push_back(generate_point(x4, y4, z2));

  polygon.push_back(generate_point(x1, y1, z2));
  return polygon;
}

geometry_msgs::msg::Point32 BoxPolygonCreator::generate_point(double x, double y, double z)
{
  geometry_msgs::msg::Point32 point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

}  // namespace autoware::crop_box_filter
