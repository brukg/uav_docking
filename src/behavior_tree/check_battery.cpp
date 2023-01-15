#include "uav_docking/behavior_tree/check_battery.h"

namespace uav_docking
{
  CheckBattery::CheckBattery(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
          : BT::StatefulActionNode(name, config), _nh(nh)
  {
    // _battery_sub = _nh.subscribe("/mavros/battery", 1, &CheckBattery::batteryCallback, this);

  }

  BT::NodeStatus CheckBattery::onStart()
  {

    _battery_read = 0;
    // minimum_battery_voltage_ = -100.0;
    // Get the parameters from the XML file
    if (!getInput<double>("minimum_battery_voltage", minimum_battery_voltage_))
    {
      throw BT::RuntimeError("missing required input [minimum_battery_voltage]");
    }
    current_battery_voltage_ = 0.0;
    _battery_sub = _nh.subscribe("mavros/battery", 10, &CheckBattery::batteryCallback, this);
    // get ros param for battery charging status
    _nh.param<bool>("battery_charging", _battery_charging, false);
    ROS_INFO_STREAM(name() << " Started! "<<_battery_read);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus CheckBattery::onRunning()
  { 
    ROS_INFO_STREAM(name() << " Running!");
    ROS_INFO_STREAM(name() << " Minimum battery voltage: " << minimum_battery_voltage_);

    // Check if the battery is low
    if (!_battery_charging && _battery_read) {   
      ROS_INFO_STREAM(name() << " Battery read! " << current_battery_voltage_);
      if (current_battery_voltage_ < minimum_battery_voltage_){
          // If the battery is low, return FAILURE
          ROS_INFO("Battery LOW %f", current_battery_voltage_);
          return BT::NodeStatus::FAILURE;
      }
      else{
          // If the battery is not low, return SUCCESS
          ROS_INFO("Battery OK %f", current_battery_voltage_);
          return BT::NodeStatus::SUCCESS;
      }
    }
    else if ((!_battery_charging && !_battery_read) || _battery_charging) {
      ROS_INFO_STREAM(name() << " Battery not read! or already charging" << current_battery_voltage_);

      return BT::NodeStatus::RUNNING;
    } 
    else {
      ROS_INFO_STREAM(name() << " charging" << current_battery_voltage_);

      return BT::NodeStatus::RUNNING;
    }
  }

 
  void CheckBattery::onHalted()
  {
    ROS_INFO_STREAM(name() << " Halted!");
  }

  void CheckBattery::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
  { 
    // ROS_INFO_STREAM(name() << "callback "<< msg->voltage);
    current_battery_voltage_ = msg->voltage;
    _battery_read = true;
  }
}