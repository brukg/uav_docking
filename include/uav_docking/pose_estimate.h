#ifndef PoseEstimate_H
#define PoseEstimate_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>

namespace uav_docking {
  class PoseEstimate
  {
    public:
      PoseEstimate(ros::NodeHandle node, ros::NodeHandle private_nh);
      ~PoseEstimate();

      /**
       * @brief pose  call back
       * 
       * @param msg 
       */
      void odomPoseCallBack(const nav_msgs::Odometry::ConstPtr& msg);
    private:
      // advertise robot pose
      ros::Publisher odom_pub; // publisher for pose msg
      ros::Subscriber odom_sub; // subscriber for laser msg

      // frames
      std::string _map_frame_id; // map frame
      std::string _odom_frame_id; // odom frame
      std::string _base_link_frame_id; // base link frame
      double _x_var, _y_var, _yaw_var;
  };
}
#endif // PoseEstimate_H