#include "fly.h"

inline void SleepMS(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

using BT::NodeStatus;

NodeStatus Fly::isFly()
{
  SleepMS(200);
  return !_number ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}



NodeStatus Fly::fly()
{
  std::cout << "fly!" << std::endl;
  return NodeStatus::SUCCESS;
}

void Fly::registerNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerSimpleCondition("isFlyh",
                                  std::bind(&Fly::isFly, this));

  factory.registerSimpleAction("fly",
                               std::bind(&Fly::fly, this));

}

void Fly::reset()
{
   _number= false;
}

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
  static Fly cross_door;
  cross_door.registerNodes(factory);
}
