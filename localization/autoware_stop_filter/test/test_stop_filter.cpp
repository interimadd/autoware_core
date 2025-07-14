#include "autoware/stop_filter/stop_filter.hpp"

#include <gtest/gtest.h>

namespace autoware::stop_filter
{

TEST(StopFilterTest, FilterStopWhenStopped)
{
    StopFilter filter(0.1, 0.1);
    Vector3D linear_velocity = {0.05, 0.0, 0.0};
    Vector3D angular_velocity = {0.0, 0.0, 0.05};

    FilterResult result = filter.apply_stop_filter(linear_velocity, angular_velocity);
    
    EXPECT_TRUE(result.was_stopped);
    EXPECT_EQ(result.linear_velocity.x, 0.0);
    EXPECT_EQ(result.linear_velocity.y, 0.0);
    EXPECT_EQ(result.linear_velocity.z, 0.0);
    EXPECT_EQ(result.angular_velocity.x, 0.0);
    EXPECT_EQ(result.angular_velocity.y, 0.0);
    EXPECT_EQ(result.angular_velocity.z, 0.0);
}

TEST(StopFilterTest, FilterStopWhenNotStopped)
{
    StopFilter filter(0.1, 0.1);
    Vector3D linear_velocity = {0.2, 0.0, 0.0};
    Vector3D angular_velocity = {0.0, 0.0, 0.2};
    
    FilterResult result = filter.apply_stop_filter(linear_velocity, angular_velocity);
    
    EXPECT_FALSE(result.was_stopped);
    EXPECT_EQ(result.linear_velocity.x, 0.2);
    EXPECT_EQ(result.linear_velocity.y, 0.0);
    EXPECT_EQ(result.linear_velocity.z, 0.0);
    EXPECT_EQ(result.angular_velocity.x, 0.0);
    EXPECT_EQ(result.angular_velocity.y, 0.0);
    EXPECT_EQ(result.angular_velocity.z, 0.2);
}

TEST(StopFilterTest, FilterStopOnlyLinearVelocityBelowThreshold)
{
    StopFilter filter(0.1, 0.1);
    Vector3D linear_velocity = {0.05, 0.0, 0.0};
    Vector3D angular_velocity = {0.0, 0.0, 0.2};

    FilterResult result = filter.apply_stop_filter(linear_velocity, angular_velocity);

    EXPECT_FALSE(result.was_stopped);
    EXPECT_EQ(result.linear_velocity.x, 0.05);
    EXPECT_EQ(result.angular_velocity.z, 0.2);
}

TEST(StopFilterTest, FilterStopOnlyAngularVelocityBelowThreshold)
{
    StopFilter filter(0.1, 0.1);
    Vector3D linear_velocity = {0.2, 0.0, 0.0};
    Vector3D angular_velocity = {0.0, 0.0, 0.05};

    FilterResult result = filter.apply_stop_filter(linear_velocity, angular_velocity);

    EXPECT_FALSE(result.was_stopped);
    EXPECT_EQ(result.linear_velocity.x, 0.2);
    EXPECT_EQ(result.angular_velocity.z, 0.05);
}

}