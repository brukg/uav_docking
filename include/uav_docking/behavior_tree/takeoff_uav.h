#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <uav_ros_msgs/TakeOff.h>
#include "uav_docking/waypoint_generator/waypoint_generator.h"
using namespace waypoint_generator;     

namespace uav_docking
{
  class TakeOffUAV : public BT::SyncActionNode
  {
    public:
        TakeOffUAV(const std::string &name, const BT::NodeConfiguration &config);
            
        void controllerStatusCallback(const std_msgs::String::ConstPtr &msg);

        // It is mandatory to define this static method.
        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<Pose3D>("goal") };
        }

        virtual BT::NodeStatus tick() override;

        // virtual void halt() override
        // {
        //     _aborted = true;
        // }

    private:
        ros::NodeHandle _nh;
        ros::Publisher _pose_pub, _joy_pub;
        ros::Subscriber _cntrlr_status_sub;
        ros::ServiceClient _takeoff_client, _arming_client, _set_mode_client;
        bool _aborted, _battery_charging;
        WaypointGenerator _waypoint_generator;
        std::string controller_status_;
  };


}