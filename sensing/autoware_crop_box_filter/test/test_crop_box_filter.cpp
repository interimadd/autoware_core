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

#include "pointcloud_points_vector_conversion.hpp"

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

  EXPECT_TRUE(is_same_points(expected_points, extract_points_from_cloud(output)));
}

}  // namespace autoware::crop_box_filter
