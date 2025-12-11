// Copyright(c) 2025 AutoCore Technology (Nanjing) Co., Ltd. All rights reserved.
//
// Copyright 2025 TIER IV, Inc.
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

#include <sensor_msgs/point_cloud2_iterator.hpp>


sensor_msgs::msg::PointCloud2 create_pointcloud2(std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  sensor_msgs::PointCloud2Modifier modifier(pointcloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");

  for (const auto & point : points) {
    *iter_x = point[0];
    *iter_y = point[1];
    *iter_z = point[2];
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return pointcloud;
}

std::vector<std::array<float, 3>> extract_points_from_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<std::array<float, 3>> points;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    points.push_back({*iter_x, *iter_y, *iter_z});
  }
  return points;
}

bool is_same_points(
  const std::vector<std::array<float, 3>> & points1,
  const std::vector<std::array<float, 3>> & points2)
{
  if (points1.size() != points2.size()) {
    printf("Size mismatch: %zu != %zu\n", points1.size(), points2.size());
    return false;
  }
  // sort both vectors to ensure order does not affect comparison
  std::vector<std::array<float, 3>> sorted_points1 = points1;
  std::vector<std::array<float, 3>> sorted_points2 = points2;
  std::sort(sorted_points1.begin(), sorted_points1.end());
  std::sort(sorted_points2.begin(), sorted_points2.end());
  // compare each point
  for (size_t i = 0; i < sorted_points1.size(); ++i) {
    if (sorted_points1[i] != sorted_points2[i]) {
      printf("Point mismatch at index %zu: (%f, %f, %f) != (%f, %f, %f)\n", i,
        sorted_points1[i][0], sorted_points1[i][1], sorted_points1[i][2],
        sorted_points2[i][0], sorted_points2[i][1], sorted_points2[i][2]);
      return false;
    }
  }
  return true;
}
