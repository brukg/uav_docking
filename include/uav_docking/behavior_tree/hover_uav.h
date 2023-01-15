#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "uav_docking/waypoint_generator/waypoint_generator.h"
#include <std_msgs/String.h>


using namespace waypoint_generator;     
namespace uav_docking
{
class HoverUAV : public BT::AsyncActionNode
{
public:
    HoverUAV(const std::string &name, const BT::NodeConfiguration &config);
        
    void controllerStatusCallback(const std_msgs::String::ConstPtr &msg);

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<Pose3D>("goal") };
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override
    {
        _aborted.store(true);
    }

private:
    ros::NodeHandle _nh;
    ros::Publisher _pose_pub;
    ros::Subscriber _cntrlr_status_sub;
    std::atomic_bool _aborted;
    WaypointGenerator _waypoint_generator;
    std::string controller_status_;
};


}//namespace uav_docking