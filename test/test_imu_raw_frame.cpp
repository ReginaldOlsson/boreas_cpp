#include <boreas/ground_truth.hpp>

#include <gtest/gtest.h>

TEST(ImuRawFrame, left_handed_raw_matches_the_right_handed_imu_frame)
{
  boreas::ImuSample raw;
  raw.accel_x = -1.5;
  raw.accel_y = 0.4;
  raw.accel_z = -9.81;
  raw.angvel_x = 0.2;
  raw.angvel_y = -0.3;
  raw.angvel_z = 0.5;

  const boreas::ImuSample right_handed = boreas::to_right_handed_imu_raw(raw);
  const sensor_msgs::msg::Imu msg = boreas::imu_to_ros_msg(right_handed);

  EXPECT_DOUBLE_EQ(msg.linear_acceleration.x, -1.5);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.y, -0.4);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.z, -9.81);
  EXPECT_DOUBLE_EQ(msg.angular_velocity.x, -0.2);
  EXPECT_DOUBLE_EQ(msg.angular_velocity.y, -0.3);
  EXPECT_DOUBLE_EQ(msg.angular_velocity.z, -0.5);
}

TEST(ImuRawFrame, only_imu_raw_is_converted)
{
  EXPECT_TRUE(boreas::imu_csv_is_left_handed_raw("imu_raw.csv"));
  EXPECT_FALSE(boreas::imu_csv_is_left_handed_raw("imu.csv"));
}
