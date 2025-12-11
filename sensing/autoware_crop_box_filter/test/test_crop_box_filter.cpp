// Copyright 2025 TIER IV
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

#include "../src/crop_box_filter.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

namespace autoware::crop_box_filter
{

TEST(CropBoxFilterTest, CreateCropBoxPolygonMsg)
{
  CropBoxSize box_size;
  box_size.min_x = 0.0;
  box_size.min_y = 0.0;
  box_size.min_z = 0.0;
  box_size.max_x = 1.0;
  box_size.max_y = 1.0;
  box_size.max_z = 1.0;
  std::string frame_id = "test_frame";

  BoxPolygonCreator creator(box_size, frame_id);

  // Call the function to create polygon message
  auto polygon_msg = creator.create_crop_box_polygon_msg(rclcpp::Time(0));

  // Verify header
  EXPECT_EQ(polygon_msg.header.frame_id, "test_frame");

  // Verify polygon points - should form a 3D box wireframe
  // The polygon should have 16 points that trace the edges of the box
  ASSERT_EQ(polygon_msg.polygon.points.size(), 16u);
}

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

TEST(CropBoxFilterTest, ExtractPointsInsideBoxWithZeroLengthPointCloud)
{
  CropBoxSize box_size;
  box_size.min_x = -5.0;
  box_size.max_x = 5.0;
  box_size.min_y = -5.0;
  box_size.max_y = 5.0;
  box_size.min_z = -5.0;
  box_size.max_z = 5.0;
  std::vector<std::array<float, 3>> points;  // empty point cloud
  PointCloud2 input = create_pointcloud2(points);

  CropBoxFilterCore crop_box_filter(box_size);
  PointCloud2 output_inside = crop_box_filter.extract_pointcloud_inside_box(input);

  ASSERT_EQ(output_inside.data.size(), 0u);
}

TEST(CropBoxFilterTest, ExtractPointsInsideBox)
{
  CropBoxSize box_size;
  box_size.min_x = -1.0;
  box_size.max_x = 1.0;
  box_size.min_y = -2.0;
  box_size.max_y = 2.0;
  box_size.min_z = -3.0;
  box_size.max_z = 3.0;

  // clang-format off

  // input points
  std::vector<std::array<float, 3>> input_points = {
    // points inside the box
    {0.0f, 0.0f, 0.0f},
    {0.5f, 1.5f, -2.5f},
    {-0.5f, -1.5f, 2.5f},
    // points outside the box
    {2.0f, 0.0f, 0.0f},
    {-2.0f, 0.0f, 0.0f},
    {0.0f, 3.0f, 0.0f},
    {0.0f, -3.0f, 0.0f},
    {0.0f, 0.0f, 4.0f},
    {0.0f, 0.0f, -4.0f}
  };
  // expected points after filtering
  std::vector<std::array<float, 3>> expected_points = {
    {0.0f, 0.0f, 0.0f},
    {0.5f, 1.5f, -2.5f},
    {-0.5f, -1.5f, 2.5f}
  };

  // clang-format on

  PointCloud2 input_cloud = create_pointcloud2(input_points);
  CropBoxFilterCore crop_box_filter(box_size);
  PointCloud2 output = crop_box_filter.extract_pointcloud_inside_box(input_cloud);

  printf("Output point count: %zu\n", output.data.size() / output.point_step);
  printf("Expected point count: %zu\n", expected_points.size());
  EXPECT_TRUE(is_same_points(expected_points, extract_points_from_cloud(output)));
}

}  // namespace autoware::crop_box_filter
