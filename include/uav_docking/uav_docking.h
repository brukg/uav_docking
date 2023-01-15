#ifndef UAV_DOCKING_H
#define UAV_DOCKING_H
#include <ros/ros.h>
#include <ros/package.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include <std_msgs/String.h>


#include "uav_docking/behavior_tree/move_to_pose.h"
#include "uav_docking/behavior_tree/land_uav.h"
#include "uav_docking/behavior_tree/hover_uav.h"
#include "uav_docking/behavior_tree/check_battery.h"


using namespace BT;


namespace uav_docking
{
    class UAVDocking
    {
        public:
            UAVDocking(ros::NodeHandle node, ros::NodeHandle private_nh);
            ~UAVDocking(){};
            void init();
            void run(); 
        private:
            ros::NodeHandle _nh;
            ros::NodeHandle _private_nh;

            ros::Subscriber battery_sub_;
            ros::Subscriber cntrler_sub_;
            std::string _behavior_tree_xml_path;
            std::string _behavior_tree_xml;
            BehaviorTreeFactory factory;
            Tree tree;
            NodeStatus status;
    };
};

#endif