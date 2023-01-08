#include <uav_docking/pose_estimate.h>

namespace uav_docking {
  PoseEstimate::PoseEstimate(ros::NodeHandle node, ros::NodeHandle private_nh)
  {
    /* Loading parameters  */
    // frames
    private_nh.param<std::string>("map_frame", _map_frame_id, "map");
    private_nh.param<std::string>("base_link_frame", _base_link_frame_id, "base_link");
    private_nh.param<std::string>("odom_frame", _odom_frame_id, "odom");

    private_nh.param<double>("x_var", _x_var, 0.0025);
    private_nh.param<double>("y_var", _y_var, 0.0025);
    private_nh.param<double>("yaw_var", _yaw_var, 0.0025);

    // advertize robot pose(lidar) with covariance
    odom_pub = node.advertise<nav_msgs::Odometry>("odometry", 1);

    // subscribe to lidar pose msg 
    odom_sub = node.subscribe("pose", 
                                1, 
                                &PoseEstimate::odomPoseCallBack, 
                                this,
                                ros::TransportHints().tcpNoDelay());
 }

  // Destructor
  PoseEstimate::~PoseEstimate(){};

  //lidar pose call back
  void PoseEstimate::odomPoseCallBack(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // ROS_INFO("lidar pose callback");
    nav_msgs::Odometry pose_msg;
    pose_msg.header.frame_id = _map_frame_id;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.pose.pose = msg->pose.pose;
    pose_msg.pose.covariance[0] = _x_var; // x covariance
    pose_msg.pose.covariance[7] = _y_var; // y coovaiance
    pose_msg.pose.covariance[35] = _yaw_var; // yaw covariance
    odom_pub.publish(pose_msg);
  };
}; // namespace lidar_pose_with_covariance


  


int main(int argc, char **argv)
{

  // Initialize ROS
  ros::init(argc, argv, "pose_estimate");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  uav_docking::PoseEstimate PoseEstimate(node, private_nh);

  ros::Rate loop_rate(10); //loop frequency
  ros::spin(); //invokes callback

  return 0;
};