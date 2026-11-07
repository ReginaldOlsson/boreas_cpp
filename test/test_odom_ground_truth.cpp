#include <boreas/ground_truth.hpp>

#include <gtest/gtest.h>

#include <cmath>

TEST(OdomGroundTruth, yaw_offset_puts_enu_velocity_on_base_link_x)
{
  const Eigen::Isometry3d T_base_lidar = boreas::yaw_only_isometry(-45.0);
  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  T_map_lidar.linear() = T_base_lidar.linear();
  T_map_lidar.translation() = Eigen::Vector3d(10.0, 20.0, 3.0);

  boreas::BoreasPose pose;
  pose.vel_east = 4.0;
  pose.vel_north = 0.0;
  pose.vel_up = 0.0;
  pose.angvel_x = 1.0;
  pose.angvel_y = 0.0;
  pose.angvel_z = 0.0;

  const nav_msgs::msg::Odometry odom =
    boreas::map_base_link_odometry(T_map_lidar, T_base_lidar, pose, "map", "base_link");

  EXPECT_EQ(odom.header.frame_id, "map");
  EXPECT_EQ(odom.child_frame_id, "base_link");
  EXPECT_NEAR(odom.pose.pose.position.x, 10.0, 1e-9);
  EXPECT_NEAR(odom.pose.pose.position.y, 20.0, 1e-9);
  EXPECT_NEAR(odom.pose.pose.position.z, 3.0, 1e-9);
  EXPECT_NEAR(odom.pose.pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(odom.pose.pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(odom.pose.pose.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(odom.pose.pose.orientation.w, 1.0, 1e-9);

  EXPECT_NEAR(odom.twist.twist.linear.x, 4.0, 1e-9);
  EXPECT_NEAR(odom.twist.twist.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(odom.twist.twist.linear.z, 0.0, 1e-9);

  const double half_sqrt2 = std::sqrt(2.0) / 2.0;
  EXPECT_NEAR(odom.twist.twist.angular.x, half_sqrt2, 1e-9);
  EXPECT_NEAR(odom.twist.twist.angular.y, -half_sqrt2, 1e-9);
  EXPECT_NEAR(odom.twist.twist.angular.z, 0.0, 1e-9);
}
