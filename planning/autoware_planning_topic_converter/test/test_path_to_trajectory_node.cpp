// Copyright 2026 TIER IV, Inc.
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

#include <autoware/planning_topic_converter/path_to_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using autoware::planning_topic_converter::PathToTrajectory;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::PathPoint;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

namespace
{
constexpr char INPUT_TOPIC[] = "/converter/input/path";
constexpr char OUTPUT_TOPIC[] = "/converter/output/trajectory";
}  // namespace

class PathToTrajectoryIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);

    path_publisher_ = test_node_->create_publisher<Path>(INPUT_TOPIC, rclcpp::QoS(1));

    trajectory_subscription_ = test_node_->create_subscription<Trajectory>(
      OUTPUT_TOPIC, rclcpp::QoS(1),
      [this](const Trajectory::SharedPtr message) { received_message_ = message; });
  }

  void initialize_converter_node()
  {
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("input_topic", std::string(INPUT_TOPIC));
    node_options.append_parameter_override("output_topic", std::string(OUTPUT_TOPIC));

    node_ = std::make_shared<PathToTrajectory>(node_options);
    executor_->add_node(node_);
  }

  void TearDown() override
  {
    executor_.reset();
    trajectory_subscription_.reset();
    path_publisher_.reset();
    test_node_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void publish_path(const Path & path)
  {
    // Spin until the converter node's subscription is discovered so the message is not dropped.
    auto start = std::chrono::steady_clock::now();
    while (path_publisher_->get_subscription_count() == 0) {
      if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(3000)) {
        break;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    path_publisher_->publish(path);
  }

  Trajectory::SharedPtr receive_published_message(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    received_message_.reset();
    auto start = std::chrono::steady_clock::now();
    while (!received_message_) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return nullptr;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return received_message_;
  }

  std::shared_ptr<PathToTrajectory> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_subscription_;

  Trajectory::SharedPtr received_message_;
};

PathPoint create_path_point(
  double x, double y, float longitudinal_velocity, float lateral_velocity, float heading_rate)
{
  PathPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = longitudinal_velocity;
  point.lateral_velocity_mps = lateral_velocity;
  point.heading_rate_rps = heading_rate;
  return point;
}

Path create_path(const std::string & frame_id, const std::vector<PathPoint> & points)
{
  Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  path.points = points;
  return path;
}

// The node copies pose and the three velocity fields from each PathPoint into a TrajectoryPoint.
bool is_same_point(const TrajectoryPoint & trajectory_point, const PathPoint & path_point)
{
  return trajectory_point.pose == path_point.pose &&
         trajectory_point.longitudinal_velocity_mps == path_point.longitudinal_velocity_mps &&
         trajectory_point.lateral_velocity_mps == path_point.lateral_velocity_mps &&
         trajectory_point.heading_rate_rps == path_point.heading_rate_rps;
}

bool is_same(
  const std::vector<TrajectoryPoint> & trajectory_points,
  const std::vector<PathPoint> & path_points)
{
  if (trajectory_points.size() != path_points.size()) {
    return false;
  }
  for (size_t index = 0; index < trajectory_points.size(); ++index) {
    if (!is_same_point(trajectory_points[index], path_points[index])) {
      return false;
    }
  }
  return true;
}

// An empty path takes the short path: the conversion loop never runs, so the output trajectory
// holds no points while still carrying over the input header.
TEST_F(PathToTrajectoryIntegrationTest, EmptyPathProducesEmptyTrajectory)
{
  // Arrange
  const auto path = create_path("map", {});
  initialize_converter_node();

  // Act
  publish_path(path);
  const auto result = receive_published_message();

  // Assert
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->header, path.header);
  EXPECT_TRUE(is_same(result->points, path.points));
}

// The longest path: a multi-point path drives the conversion loop once per point, copying each
// pose and velocity field into the output trajectory while preserving the header.
TEST_F(PathToTrajectoryIntegrationTest, PathPointsAreConvertedToTrajectoryPoints)
{
  // Arrange
  const auto path = create_path(
    "map", {
             create_path_point(0.0, 0.0, 1.0f, 0.1f, 0.01f),
             create_path_point(1.0, 2.0, 2.0f, 0.2f, 0.02f),
             create_path_point(3.0, 4.0, 3.0f, 0.3f, 0.03f),
           });
  initialize_converter_node();

  // Act
  publish_path(path);
  const auto result = receive_published_message();

  // Assert
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->header, path.header);
  EXPECT_TRUE(is_same(result->points, path.points));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
