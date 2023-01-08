
#ifndef WAYPOINT_GENERATOR_H
#define WAYPOINT_GENERATOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "uav_docking/utils.h"
#include <vector>
// Custom type
// struct Pose3D
// {
//     double x, y, z, psi;
// };
namespace waypoint_generator {
    class WaypointGenerator
    {
    public:
        WaypointGenerator();
        ~WaypointGenerator();

        /**
         * @brief 
         * 
         * @param msg 
         */
        void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg);

        /**
         * @brief 
         * 
         * @param msg 
         */
        void getGoalPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

        /**
         * @brief 
         * 
         * @param msg 
         */

        void generateWaypoints(Pose3D& goal, char wp_type, std::vector<Pose3D>& waypoints);

        /**
         * @brief 
         * 
         * @param msg 
         */
        void publishWaypoints();


    private:
        ros::Publisher waypoint_pub; // publisher for pose msg
        ros::Subscriber odom_sub; // subscriber for odometry_with_covariance odom msg
        ros::Subscriber goal_pose_sub; // subscriber for odometry_with_covariance odom msg

        std::string _map_frame_id; // map frame
        std::string _odom_frame_id; // odom frame
        std::string _base_link_frame_id; // base link frame

        nav_msgs::Odometry odom_msg;
        geometry_msgs::PoseStamped _current_pose;
        bool _odom_ready;

    };
}

#endif 
