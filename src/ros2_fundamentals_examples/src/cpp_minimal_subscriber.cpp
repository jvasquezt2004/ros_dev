#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalCppSubscriber : public rclcpp::Node
{
  public:
    MinimalCppSubscriber() : Node("minimal_cpp_subscriber")
    {
      subscription_ = create_subscription<std_msgs::msg::String>(
        "/cpp_example_topic", 10, std::bind(&MinimalCppSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalCppSubscriber>());
  rclcpp::shutdown();
  return 0;
}