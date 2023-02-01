// 
#include "uav_docking/uav_docking.h"


namespace uav_docking
{
  UAVDocking::UAVDocking(ros::NodeHandle node, ros::NodeHandle private_nh)
  : _nh(node), _private_nh(private_nh)
  {
    init();
  }


  void UAVDocking::init()
  {   
    // load parameters and tree
    _private_nh.param<std::string>("behavior_tree_xml_path", _behavior_tree_xml_path, ros::package::getPath("uav_docking") + "/config/tree/docking.xml");

    ROS_INFO("Loading XML : %s", _behavior_tree_xml_path.c_str());

    factory.registerNodeType<MoveUAV>("MoveUAV");
    factory.registerNodeType<HoverUAV>("HoverUAV");
    factory.registerNodeType<LandUAV>("LandUAV");
    factory.registerNodeType<TakeOffUAV>("TakeOffUAV");
    RegisterROSNode<LowBattery>(factory, "LowBattery", _nh);
    RegisterROSNode<FullBattery>(factory, "FullBattery", _nh);

    tree = factory.createTreeFromFile(_behavior_tree_xml_path);
    
    BT::PublisherZMQ publisher_zmq(tree);

    ros::Duration(1.0).sleep();
    run();
  }

  void UAVDocking::run()
  {
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    std::cout << "--- status: " << toStr(status) << "\n\n";
    while(ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING))
    {   
        ros::spinOnce();
        // Sleep to avoid busy loops.
        // do NOT use other sleep functions!
        // Small sleep time is OK, here we use a large one only to
        // have less messages on the console.
        tree.sleep(std::chrono::milliseconds(500));

        std::cout << "--- ticking\n";
        status = tree.tickRoot();
        std::cout << "--- status: " << toStr(status) << "\n\n";
    }
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "docking_node");
  ros::NodeHandle nh, private_nh("~");
  std::string sampling_param;
  uav_docking::UAVDocking uav_docking(nh, private_nh);

  // ros::spin();
  return 0;
}