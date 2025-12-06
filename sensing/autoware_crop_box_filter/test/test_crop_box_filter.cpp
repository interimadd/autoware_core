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

}  // namespace autoware::crop_box_filter