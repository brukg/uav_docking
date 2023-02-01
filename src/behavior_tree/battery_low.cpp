#include "uav_docking/behavior_tree/battery_low.h"

namespace uav_docking
{
  LowBattery::LowBattery(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration& config)
          : BT::StatefulActionNode(name, config), _nh(nh)
  {
    _battery_sub = _nh.subscribe("mavros/battery", 1, &LowBattery::batteryCallback, this);
    _battery_read = false; // set to false to start with
    // _battery_charging = false;
    current_battery_voltage_ = 0.0;// set to -100 to start with
    // minimum_battery_voltage_ = -100.0;

  }

  BT::NodeStatus LowBattery::onStart()
  {


    // Get the parameters from the XML file
    if (!getInput<double>("minimum_battery_voltage", minimum_battery_voltage_))
    {
      throw BT::RuntimeError("missing required input [minimum_battery_voltage]");
    }
    // _battery_sub = _nh.subscribe("mavros/battery", 10, &LowBattery::batteryCallback, this);
    
    
    ROS_INFO_STREAM(name() << " Started! "<<_battery_read);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus LowBattery::onRunning()
  { 
    ROS_INFO_STREAM(name() << " Running!");
    ROS_INFO_STREAM(name() << " Minimum battery voltage: " << minimum_battery_voltage_);
    
    // get ros param for battery charging status
    _nh.getParam("battery_charging", _battery_charging);
    
    ROS_INFO_STREAM(name() << " Charging: " << _battery_charging);
    ROS_INFO_STREAM(name() << " Battery read: " << _battery_read);
    
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
    else if (!_battery_read) {
      ROS_INFO_STREAM(name() << " Battery not read! or already charging" << current_battery_voltage_);

      return BT::NodeStatus::RUNNING;
    } 
    else if(_battery_charging){
      ROS_INFO_STREAM(name() << " charging " << current_battery_voltage_);
      return BT::NodeStatus::SUCCESS;
    }
  }

 
  void LowBattery::onHalted()
  {
    ROS_INFO_STREAM(name() << " Halted!");
  }

  void LowBattery::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
  { 
    // ROS_INFO_STREAM(name() << "callback "<< msg->voltage);
    current_battery_voltage_ = msg->voltage;
    _battery_read = true;
  }
}