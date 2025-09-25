#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalCppPublisher : public rclcpp::Node
{
  public:
    MinimalCppPublisher() : Node("minimal_cpp_publisher"), count_(0)
    {
      publisher_ = create_publisher<std_msgs::msg::String>(
        "/cpp_example_topic", 10
      );
      timer_ = create_wall_timer(500ms, std::bind(&MinimalCppPublisher::timer_callback, this));

      RCLCPP_INFO(get_logger(), "Publisher node has been started.");
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world: " + std::to_string(count_++);
      RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

    size_t count_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto minimal_cpp_publisher = std::make_shared<MinimalCppPublisher>();
  rclcpp::spin(minimal_cpp_publisher);

  rclcpp::shutdown();
  return 0;
}