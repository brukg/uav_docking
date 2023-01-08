#include "uav_docking/behavior_tree/land.h"

namespace uav_docking
{
  LandUAV::LandUAV( const std::string& name, const BT::NodeConfiguration& config)
          : BT::SyncActionNode(name, config) 
  {
    //controller status subscriber
    _cntrlr_status_sub = _nh.subscribe("tracker/status", 1, &LandUAV::controllerStatusCallback, this);
    
    // controller input pose publisher
    _pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("tracker/input_pose", 1);
    
  }


  BT::NodeStatus LandUAV::tick() 
  {

    // Take the goal from the InputPort of the Node
    Pose3D goal;
    if (!getInput<Pose3D>("goal", goal)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [goal]");
    }
    // _cntrlr_status_sub = _nh.subscribe("tracker/status", 1, &LandUAV::controllerStatusCallback, this);
    // _pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("tracker/input_pose", 1);
    
    // Reset this flag
    _aborted = false;
    std::vector<Pose3D> wps;
    _waypoint_generator.generateWaypoints(goal, 'L', wps);
    ros::Duration(0.2).sleep();
    ROS_INFO(" goal %f %f %f %f", goal.x, goal.y, goal.z, goal.psi);

    // Build the message from waypoints
    geometry_msgs::PoseStamped msg;
    while (wps.size() > 0) {
      ROS_INFO("controller status: %s", controller_status_.c_str());
      ROS_INFO("Current Landing goal poses %f %f %f %f", wps.front().x, wps.front().y, wps.front().z, wps.front().psi);
      // check if the controller is in ACCEPT state
      if (controller_status_ != "ACTIVE" && controller_status_ == "ACCEPT")
      {
        // ROS_INFO("waypoint %d", wps.front());
        msg.header.frame_id = "";
        msg.header.stamp = ros::Time::now();

        msg.pose.position.x = wps.front().x;
        msg.pose.position.y = wps.front().y;
        msg.pose.position.z = wps.front().z;

      _pose_pub.publish(msg);
      wps.erase(wps.begin());
      }
      ros::Duration(2.5).sleep();

      while (controller_status_ == "ACTIVE") {
        ros::Duration(2).sleep();
        if (controller_status_ == "ACCEPT") break;
      }
    }

    if (_aborted) {
      ROS_ERROR("LandUAV aborted");
      return BT::NodeStatus::FAILURE;
    }

    ROS_INFO("Target reached");
    return BT::NodeStatus::SUCCESS;
  }


  void LandUAV::controllerStatusCallback(const std_msgs::String::ConstPtr& msg)
  {
          // ROS_INFO("Controller status: %s", msg->data.c_str());
          controller_status_ = msg->data;
  }

} // namespace uav_docking