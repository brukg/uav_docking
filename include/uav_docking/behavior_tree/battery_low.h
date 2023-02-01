#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>


namespace uav_docking
{
  class LowBattery : public BT::StatefulActionNode
  {
    public:
      LowBattery(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &config);
      

      // It is mandatory to define this static method.
      static BT::PortsList providedPorts()
      {
          return { BT::InputPort<double>("minimum_battery_voltage") };
      }

      // virtual BT::NodeStatus tick() override;

      // virtual void halt() override
      // {
      //     _aborted = true;
      // }

    protected:
      void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
      /// method to be called at the beginning.
      /// If it returns RUNNING, this becomes an asychronous node.
      BT::NodeStatus onStart() override;

      /// method invoked by a RUNNING action.
      BT::NodeStatus onRunning() override;

      /// when the method halt() is called and the action is RUNNING, this method is invoked.
      /// This is a convenient place todo a cleanup, if needed.
      void onHalted() override;

    // private:
      ros::NodeHandle _nh;
      ros::Subscriber _battery_sub;
      double current_battery_voltage_, minimum_battery_voltage_;
      bool _aborted;
      bool _battery_read, _battery_charging;
  };
}