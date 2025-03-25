#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "behaviortree_ros2/plugins.hpp"

#ifndef USE_PHOTO_PLUGIN
#include "photo_action.hpp"
#endif

using namespace BT;

//-------------------------------------------------------------
// Simple Action to print a number
//-------------------------------------------------------------

class PrintValue : public BT::SyncActionNode
{
public:
  PrintValue(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    std::string msg;
    if(getInput("message", msg))
    {
      std::cout << "PrintValue: " << msg << std::endl;
      return NodeStatus::SUCCESS;
    }
    else
    {
      std::cout << "PrintValue FAILED " << std::endl;
      return NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }
};

//-----------------------------------------------------

// Simple tree, used to execute once each action.
static const char* xml_text = R"(
 <root BTCPP_format="4">
     <BehaviorTree>
        <Sequence>
            <PrintValue message="start"/>
            <SleepAction name="sleepA" msec="2000"/>
            <PrintValue message="sleep completed"/>
            <Fallback>
                <Timeout msec="1500">
                   <SleepAction name="sleepB" action_name="sleep_service" msec="2000"/>
                </Timeout>
                <PrintValue message="sleep aborted"/>
            </Fallback>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("photo_client");

  BehaviorTreeFactory factory;

  factory.registerNodeType<PrintValue>("PrintValue1");

  RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "photo_service";

#ifdef USE_PHOTO_PLUGIN
  RegisterRosNode(factory, "../lib/libphoto_action_plugin.so", params);
#else
  factory.registerNodeType<PhotoAction>("PhotoAction", params);
#endif

  auto tree = factory.createTreeFromText(xml_text);

  for(int i = 0; i < 5; i++)
  {
    tree.tickWhileRunning();
  }

  return 0;
}
