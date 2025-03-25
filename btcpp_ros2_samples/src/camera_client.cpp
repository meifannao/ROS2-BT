#include "rclcpp/rclcpp.hpp"
#include "psdk_interfaces/srv/camera_setup_streaming.hpp"

using namespace std::chrono_literals;

class CameraClient : public rclcpp::Node {
public:
    CameraClient() : Node("camera_service_client") {
        // 1. Create service client
        client_ = this->create_client<psdk_interfaces::srv::CameraSetupStreaming>(
            "/wrapper/psdk_ros2/camera_setup_streaming"
        );
    }

    void send_request() {
        // 2. Wait for service availability
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "Service unavailable, retrying...");
        }

        // 3. Prepare request data
        auto request = std::make_shared<psdk_interfaces::srv::CameraSetupStreaming::Request>();
        request->payload_index = 1;
        request->camera_source = 0;
        request->start_stop = true;   // Use boolean true (not "True")
        request->decoded_output = true;

        // 4. Send request asynchronously
        auto result_future = client_->async_send_request(request);

        // 5. Wait for and process response
        if (rclcpp::spin_until_future_complete(
                this->get_node_base_interface(),
                result_future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service response received");
            // Handle response data here (if any)
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

private:
    rclcpp::Client<psdk_interfaces::srv::CameraSetupStreaming>::SharedPtr client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<CameraClient>();
    client_node->send_request();
    rclcpp::shutdown();
    return 0;
}