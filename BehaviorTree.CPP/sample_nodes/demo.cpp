#include "demo.h"

inline void SleepMS(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

using BT::NodeStatus;

NodeStatus demo::isFly()
{
  SleepMS(200);
  return !_number_fly ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}


NodeStatus demo::Fly()
{
  std::cout << "fly!" << std::endl;
  return NodeStatus::SUCCESS;
}


NodeStatus demo::isCameraOpen()
{
  SleepMS(200);
  return !_number_camera ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}


NodeStatus demo::openCamera()
{
  std::cout << "cameraopen!" << std::endl;
  return NodeStatus::SUCCESS;
}


NodeStatus demo::takePhoto()
{
  std::cout << "takePhoto!" << std::endl;
  return NodeStatus::SUCCESS;
}


void demo::registerNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerSimpleCondition("isFly",
                                  std::bind(&demo::isFly, this));

  factory.registerSimpleAction("Fly",
                               std::bind(&demo::Fly, this));

  factory.registerSimpleCondition("isCameraOpen",
                                  std::bind(&demo::isCameraOpen, this));

  factory.registerSimpleAction("openCamera",
                               std::bind(&demo::openCamera, this));

  factory.registerSimpleAction("takePhoto",
                               std::bind(&demo::takePhoto, this));
}

void demo::reset()
{
   _number_fly= false;
   _number_camera= false;
}

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
  static demo cross_demo;
  cross_demo.registerNodes(factory);
}
