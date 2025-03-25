#pragma once

#include "behaviortree_cpp/bt_factory.h"

class Fly
{
public:
  void registerNodes(BT::BehaviorTreeFactory& factory);

  void reset();

  // SUCCESS if _door_open == true
  BT::NodeStatus isFly();

  // SUCCESS if _door_open == true
  BT::NodeStatus fly();



private:
  bool _number= false;
};
