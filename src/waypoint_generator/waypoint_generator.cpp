#include "uav_docking/waypoint_generator/waypoint_generator.h"



namespace waypoint_generator
{   
    WaypointGenerator::WaypointGenerator()
    {
        /* Loading parameters  */
        ros::NodeHandle node, private_nh;
        // subscribe to odometry from 
        odom_sub = node.subscribe("odometry", 10, &WaypointGenerator::odomCallBack, this);
        ROS_INFO("waypoint generator initialized");
        _odom_ready = false;
    };

    WaypointGenerator::~WaypointGenerator(){};


    void WaypointGenerator::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // ROS_INFO("odometry callback, x: %f, y: %f, z: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        // get current pose
        _current_pose.pose = msg->pose.pose;
        _current_pose.header.frame_id = _map_frame_id;
        _current_pose.header.stamp = ros::Time::now();
        _odom_ready = true;
    };

    void WaypointGenerator::generateWaypoints(Pose3D& goal, char wp_type, std::vector<Pose3D>& waypoints)
    {
        // generate waypoints
        // check the altitiude difference
        // wait for _odom_ready
        while(!_odom_ready) {
            ROS_INFO("waiting for odom");
            ros::Duration(0.5).sleep();
        }
        double delta_z = _current_pose.pose.position.z- goal.z;
        double delta_x = _current_pose.pose.position.x- goal.x;
        double delta_y = _current_pose.pose.position.y- goal.y;
        double mid_x = (_current_pose.pose.position.x + goal.x)/2;
        double mid_y = (_current_pose.pose.position.y + goal.y)/2;
        double mid_z = (_current_pose.pose.position.z + goal.z)/2;
        // hover 1 metetr above the docking station before docking

        switch(wp_type) {
            case 'H' :
                // waypoints for Hovering on top of the docking station
                // if (delta_z < 0.0)
                ROS_INFO("current pose: %f, %f, %f", _current_pose.pose.position.x, _current_pose.pose.position.y, _current_pose.pose.position.z);
                waypoints.push_back({_current_pose.pose.position.x, _current_pose.pose.position.y, goal.z+abs(delta_z), goal.psi});
                waypoints.push_back({mid_x, mid_y, goal.z+abs(delta_z), goal.psi});
                waypoints.push_back({goal.x, goal.y, goal.z+abs(delta_z), goal.psi});
                break;
            case 'L' :
                // waypoints for Landing on the docking station
                ROS_INFO("current pose: %f, %f, %f", _current_pose.pose.position.x, _current_pose.pose.position.y, _current_pose.pose.position.z);
                waypoints.push_back({goal.x, goal.y, goal.z+1, goal.psi});
                waypoints.push_back({goal.x, goal.y, goal.z+0.5, goal.psi});
                waypoints.push_back({goal.x, goal.y, goal.z+0.25, goal.psi});
                waypoints.push_back({goal.x, goal.y, goal.z, goal.psi});
                break;
            case 'M' :
                // use motion planner to generate waypoints for moving around
                waypoints.push_back({mid_x, mid_y, mid_z, goal.psi});
                waypoints.push_back({goal.x, goal.y, goal.z, goal.psi});
                break;
        }
        
        
    };

    void WaypointGenerator::publishWaypoints()
    {
        // publish waypoints
    };

    void WaypointGenerator::getGoalPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // get goal pose

    };
}