#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>


// Custom type
struct Pose3D
{
  double x, y, z, psi;
};

namespace BT
{
template <> inline
Pose3D convertFromString(StringView key)
{
  // 4 real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4)
  {
    throw BT::RuntimeError("invalid input)");
  }
  else
  {
    Pose3D output;
    output.x     = convertFromString<double>(parts[0]);
    output.y     = convertFromString<double>(parts[1]);
    output.z     = convertFromString<double>(parts[2]);
    output.psi   = convertFromString<double>(parts[3]);
    return output;
  }
}

template <class DerivedT>
static void RegisterROSNode(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                            ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle, name, config);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}

} // end namespace BT


