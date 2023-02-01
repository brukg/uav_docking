#include "uav_docking/behavior_tree/takeoff_uav.h"

namespace uav_docking
{
  TakeOffUAV::TakeOffUAV( const std::string& name, const BT::NodeConfiguration& config)
          : BT::SyncActionNode(name, config) 
  {
    //controller status subscriber
    _cntrlr_status_sub = _nh.subscribe("tracker/status", 1, &TakeOffUAV::controllerStatusCallback, this);
    
    // controller input pose publisher
    _pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("tracker/input_pose", 1);
    _joy_pub = _nh.advertise<sensor_msgs::Joy>("joy", 1);

    // create ros service client for takeoff
    _takeoff_client = _nh.serviceClient<uav_ros_msgs::TakeOff>("takeoff", true);
    _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming", true);
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode", true);

    
  }


  BT::NodeStatus TakeOffUAV::tick() 
  {
    _aborted = false;

    // Take the goal from the InputPort of the Node
    Pose3D goal;
    if (!getInput<Pose3D>("goal", goal)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [goal]");
    }
    
    // setting mode
    mavros_msgs::SetMode gngset_mode;
    gngset_mode.request.custom_mode = "GUIDED_NOGPS";
    gngset_mode.request.base_mode = 0;
    if (_set_mode_client.call(gngset_mode) && gngset_mode.response.mode_sent) {
      ROS_INFO_STREAM(name() << "Offboard enabled");
    }
    else {
      ROS_ERROR_STREAM(name() << "Offboard not enabled");
      return BT::NodeStatus::FAILURE;
    }

    // publish joy sensor message
    sensor_msgs::Joy joy_msg;
    joy_msg.buttons.push_back(0);
    joy_msg.buttons.push_back(0);
    joy_msg.buttons.push_back(0);
    joy_msg.buttons.push_back(0);
    joy_msg.buttons.push_back(0);
    joy_msg.buttons.push_back(1);
    _joy_pub.publish(joy_msg);

    // arming
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (_arming_client.call(arm_cmd) && arm_cmd.response.success) {
      ROS_INFO_STREAM(name() << "UAV armed");
    }
    else {
      ROS_ERROR_STREAM(name() << "UAV not armed");
      return BT::NodeStatus::FAILURE;
    }
    
    uav_ros_msgs::TakeOff req;
    uav_ros_msgs::TakeOffResponse res;
    req.request.rel_alt = goal.z+0.5;
    
    if (_takeoff_client.call(req)) {
      ROS_INFO_STREAM(name() << "TakeOffing service called" << res.success);
    }
    else
    {
      ROS_ERROR_STREAM(name() << "Failed to call takeoff service");
      return BT::NodeStatus::FAILURE;
    }



    if (_aborted) {
      ROS_ERROR_STREAM(name() << "aborted");
      return BT::NodeStatus::FAILURE;
    }

    ROS_INFO_STREAM(name() << "Target reached");
    
    
    _battery_charging = false;
    _nh.setParam("battery_charging", _battery_charging);
    return BT::NodeStatus::SUCCESS;
  }


  void TakeOffUAV::controllerStatusCallback(const std_msgs::String::ConstPtr& msg)
  {
          // ROS_INFO("Controller status: %s", msg->data.c_str());
          controller_status_ = msg->data;
  }

} // namespace uav_docking