#include "behaviortree_ros2/bt_action_node.hpp"
#include "btcpp_ros2_interfaces/action/photo.hpp"

using namespace BT;

class PhotoAction : public RosActionNode<btcpp_ros2_interfaces::action::Photo>
{
public:
  PhotoAction(const std::string& name, const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<btcpp_ros2_interfaces::action::Photo>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ InputPort<unsigned>("photo_number") });
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
