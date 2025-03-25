#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "btcpp_ros2_interfaces/action/photo.hpp"
#include "psdk_interfaces/srv/camera_setup_streaming.hpp"
#include "psdk_interfaces/srv/camera_shoot_single_photo.hpp"

class PhotoActionServer : public rclcpp::Node
{
public:
  using Photo = btcpp_ros2_interfaces::action::Photo;
  using GoalHandlePhoto = rclcpp_action::ServerGoalHandle<Photo>;

  explicit PhotoActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("photo_action_server", options)
  {
    using namespace std::placeholders;

    // 创建 camera_setup_streaming 服务客户端
    camera_client_ = this->create_client<psdk_interfaces::srv::CameraSetupStreaming>("/wrapper/psdk_ros2/camera_setup_streaming");

    // 创建 camera_shoot_single_photo 服务客户端
    shoot_photo_client_ = this->create_client<psdk_interfaces::srv::CameraShootSinglePhoto>("psdk_ros2/camera_shoot_single_photo");

    // 创建动作服务器
    this->action_server_ = rclcpp_action::create_server<Photo>(
        this, "photo_service", std::bind(&PhotoActionServer::handle_goal, this, _1, _2),
        std::bind(&PhotoActionServer::handle_cancel, this, _1),
        std::bind(&PhotoActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Photo>::SharedPtr action_server_;
  rclcpp::Client<psdk_interfaces::srv::CameraSetupStreaming>::SharedPtr camera_client_;
  rclcpp::Client<psdk_interfaces::srv::CameraShootSinglePhoto>::SharedPtr
      shoot_photo_client_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const Photo::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with Photo time %d",
                goal->photo_number);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandlePhoto> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePhoto> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{ std::bind(&PhotoActionServer::execute, this, _1), goal_handle }.detach();
  }

  bool call_camera_service(bool start_stop)
  {
    auto request =
        std::make_shared<psdk_interfaces::srv::CameraSetupStreaming::Request>();
    request->payload_index = 1;
    request->camera_source = 1;
    request->start_stop = start_stop;
    request->decoded_output = true;

    auto future = camera_client_->async_send_request(request);
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
       rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call camera setup streaming service");
      return false;
    }
    return true;
  }

  void execute(const std::shared_ptr<GoalHandlePhoto> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(5);  // 每秒 5 次循环
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Photo::Feedback>();
    auto result = std::make_shared<Photo::Result>();

    // 等待 camera_setup_streaming 服务
    while(!camera_client_->wait_for_service(std::chrono::seconds(1)))
    {
      if(!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for "
                                         "camera_setup_streaming service.");
        result->done = false;
        goal_handle->succeed(result);
        return;
      }
      RCLCPP_WARN(this->get_logger(), "camera_setup_streaming service unavailable, "
                                      "waiting...");
    }

    // 等待 camera_shoot_single_photo 服务
    while(!shoot_photo_client_->wait_for_service(std::chrono::seconds(1)))
    {
      if(!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for "
                                         "camera_shoot_single_photo service.");
        result->done = false;
        goal_handle->succeed(result);
        return;
      }
      RCLCPP_WARN(this->get_logger(), "camera_shoot_single_photo service unavailable, "
                                      "waiting...");
    }

    // 开始流式传输
    if(!call_camera_service(true))
    {
      result->done = false;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal failed due to service call error");
      return;
    }

    int cycle = 0;
    while(cycle < goal->photo_number)
    {
      // 检查是否收到取消请求
      if(goal_handle->is_canceling())
      {
        call_camera_service(false);  // 停止流式传输
        result->done = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // 调用 camera_shoot_single_photo 服务
      auto request =
          std::make_shared<psdk_interfaces::srv::CameraShootSinglePhoto::Request>();
      request->payload_index = 1;  // 设置 payload_index 为 1

      auto future = shoot_photo_client_->async_send_request(request);
      if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
         rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to call camera_shoot_single_photo "
                                         "service");
        call_camera_service(false);  // 停止流式传输
        result->done = false;
        goal_handle->succeed(result);
        return;
      }

      // 检查服务响应
      auto response = future.get();
      if(!response->success)
      {
        RCLCPP_ERROR(this->get_logger(), "camera_shoot_single_photo service call failed");
        call_camera_service(false);  // 停止流式传输
        result->done = false;
        goal_handle->succeed(result);
        return;
      }

      // 更新反馈
      feedback->photo_number_done = cycle++;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();  // 控制循环频率
    }

    // 停止流式传输
    call_camera_service(false);

    // 设置目标成功
    if(rclcpp::ok())
    {
      result->done = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PhotoActionServer>();
  rclcpp::spin(node);
  return 0;
}