#pragma once

#include "behaviortree_cpp/bt_factory.h"

class demo
{
public:
  void registerNodes(BT::BehaviorTreeFactory& factory);

  void reset();

  // SUCCESS if _door_open == true
  BT::NodeStatus isFly();

  // SUCCESS if _door_open == true
  BT::NodeStatus Fly();

  BT::NodeStatus isCameraOpen();

  BT::NodeStatus openCamera();

  BT::NodeStatus takePhoto();

private:
  bool _number_fly= false;
  bool _number_camera= false;
};
