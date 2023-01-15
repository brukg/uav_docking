#include "uav_docking/behavior_tree/hover_uav.h"


namespace uav_docking
{
  HoverUAV::HoverUAV( const std::string& name, const BT::NodeConfiguration& config)
          : BT::AsyncActionNode(name, config) {
    //controller status subscriber
    _cntrlr_status_sub = _nh.subscribe("tracker/status", 1, &HoverUAV::controllerStatusCallback, this);
    
    // controller input pose publisher
    _pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("tracker/input_pose", 1);
    // ROS_INFO_STREAM(name() << " Started!");
  }

  BT::NodeStatus HoverUAV::tick() {
    
    setStatus(BT::NodeStatus::RUNNING);


    // Take the goal from the InputPort of the Node
    Pose3D goal;
    if (!getInput<Pose3D>("goal", goal)) {  
      throw BT::RuntimeError("missing required input [goal]");
    }



    // Reset this flag
    _aborted = false;
    std::vector<Pose3D> wps;
    _waypoint_generator.generateWaypoints(goal, 'H', wps);
    ROS_INFO("hovering goal %f %f %f %f", goal.x, goal.y, goal.z, goal.psi);
    // ROS_INFO("wps 0 %f %f %f %f", wps[0].x, wps[0].y, wps[0].z, wps[0].psi);
    // wps.push_back({2, 2, 4, 0});
    
    // Build the message from waypoints
    geometry_msgs::PoseStamped msg;
    while (wps.size() > 0) {
      ROS_INFO("controller status: %s", controller_status_.c_str());
      ROS_INFO("Current Moving goal %f %f %f %f", wps.front().x, wps.front().y, wps.front().z, wps.front().psi);
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
      // if (controller_status_ == "ACTIVE") break;
      // }
      ros::Duration(2.5).sleep();
      
      // sleep untilt the controller is ready to accept new waypoints
      while (controller_status_ == "ACTIVE") {
        ros::Duration(2).sleep();
        if (controller_status_ == "ACCEPT") break;
      }
    }



    if (_aborted) {
      ROS_ERROR("HoverUAV aborted");
      return BT::NodeStatus::FAILURE;
    }
    ROS_INFO("Target reached");
    // return BT::NodeStatus::IDLE;
    return BT::NodeStatus::SUCCESS;
  }

  void HoverUAV::controllerStatusCallback(const std_msgs::String::ConstPtr& msg)
  {
          // ROS_INFO("Controller status: %s", msg->data.c_str());
          controller_status_ = msg->data;
  }


}// namespace uav_docking