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

#ifndef CROP_BOX_FILTER_HPP_
#define CROP_BOX_FILTER_HPP_

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware::crop_box_filter
{

struct CropBoxSize
{
  double min_x;
  double min_y;
  double min_z;
  double max_x;
  double max_y;
  double max_z;
};

using PointCloud2 = sensor_msgs::msg::PointCloud2;

// class CropBoxFilter
// {
// public:
//   CropBoxFilter(CropBoxSize box_size);
//   PointCloud2 crop_pointcloud_inside_box(const PointCloud2 input) const;
//   PointCloud2 crop_pointcloud_outside_box(const PointCloud2 input) const;

// private:
//   CropBoxSize _box_size;
// };

// class BoxPolygonCreator
// {
// public:
//   BoxPolygonCreator(CropBoxSize box_size);
//   geometry_msgs::msg::PolygonStamped create_crop_box_polygon_msg(const rclcpp::Time timestamp);

// private:
//   CropBoxSize _box_size;
// };

}  // namespace autoware::crop_box_filter
#endif  // CROP_BOX_FILTER_HPP_
