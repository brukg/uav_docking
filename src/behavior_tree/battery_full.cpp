#include "uav_docking/behavior_tree/battery_full.h"

namespace uav_docking
{
  FullBattery::FullBattery(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
          : BT::StatefulActionNode(name, config), _nh(nh)
  {
    _battery_sub = _nh.subscribe("mavros/battery", 1, &FullBattery::batteryCallback, this);
    // get ros param for battery charging status
    _battery_read = 0;
    _battery_charged = false;
    current_battery_voltage_ = 0.0;
    // optimal_battery_voltage_ = -100.0;

  }

  BT::NodeStatus FullBattery::onStart()
  {

    // Get the parameters from the XML file
    if (!getInput<double>("optimal_battery_voltage", optimal_battery_voltage_))
    {
      throw BT::RuntimeError("missing required input [optimal_battery_voltage]");
    }
    // _battery_sub = _nh.subscribe("mavros/battery", 10, &FullBattery::batteryCallback, this);
    ROS_INFO_STREAM(name() << " Started! "<<_battery_read);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus FullBattery::onRunning()
  { 
    ROS_INFO_STREAM(name() << " Running!");
    ROS_INFO_STREAM(name() << " Optimal battery voltage: " << optimal_battery_voltage_);

    _nh.getParam("battery_charged", _battery_charged);
    ROS_INFO_STREAM(name() << " Charged: " << _battery_charged);
    ROS_INFO_STREAM(name() << " Battery read: " << _battery_read);

    // Check if the battery is low
    if (_battery_charged && _battery_read) {   
      ROS_INFO_STREAM(name() << " Battery read! " << current_battery_voltage_);
      if (current_battery_voltage_ >= optimal_battery_voltage_){
          // If the battery is Full, return SUCCESS
          ROS_INFO("Battery Ready %f", current_battery_voltage_);
          return BT::NodeStatus::SUCCESS;
      }
      else{
          // If the battery is not Full, return FAILURE
          ROS_INFO("Battery Low %f", current_battery_voltage_);
          return BT::NodeStatus::FAILURE;
      }
    }
    else if (!_battery_read) {
      ROS_INFO_STREAM(name() << " Battery not read! " << current_battery_voltage_);

      return BT::NodeStatus::RUNNING;
    } 
    else if(!_battery_charged){
      ROS_INFO_STREAM(name() << " charging" << current_battery_voltage_);

      return BT::NodeStatus::RUNNING;
    }
  }

 
  void FullBattery::onHalted()
  {
    ROS_INFO_STREAM(name() << " Halted!");
  }

  void FullBattery::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
  { 
    // ROS_INFO_STREAM(name() << "callback "<< msg->voltage);
    current_battery_voltage_ = msg->voltage;
    _battery_read = true;
  }
}