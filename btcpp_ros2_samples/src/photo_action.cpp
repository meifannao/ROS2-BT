#include "photo_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool PhotoAction::setGoal(RosActionNode::Goal& goal)
{
  auto timeout = getInput<unsigned>("photo_number");
  goal.photo_number = timeout.value();
  return true;
}

NodeStatus PhotoAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              wr.result->done ? "true" : "false");

  return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus PhotoAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void PhotoAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Plugin registration.
// The class SleepAction will self register with name  "PhotoAction".
CreateRosNodePlugin(PhotoAction, "PhotoAction");
