#include <chrono>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class BimanualDemoPublisher : public rclcpp::Node
{
public:
  BimanualDemoPublisher()
  : Node("bimanual_demo_publisher"), t0_(now())
  {
    left_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/wbc/left_ee_target", 10);
    right_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/wbc/right_ee_target", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&BimanualDemoPublisher::publishTargets, this));

    RCLCPP_INFO(get_logger(), "Publishing bimanual figure-eight targets to /wbc/{left,right}_ee_target");
  }

private:
  void publishTargets()
  {
    const auto stamp = now();
    const double t = (stamp - t0_).seconds();

    const double dx = 0.15 * std::sin(0.5 * t);
    const double dy = 0.25 * std::sin(0.25 * t);
    const double dz = 0.12 * std::cos(0.5 * t);

    geometry_msgs::msg::PoseStamped left_msg;
    left_msg.header.stamp = stamp;
    left_msg.header.frame_id = "base_footprint";
    left_msg.pose.position.x = 0.65 + dx;
    left_msg.pose.position.y = 0.30 + dy;
    left_msg.pose.position.z = 0.95 + dz;
    left_msg.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped right_msg = left_msg;
    right_msg.pose.position.y = -0.30 + dy;

    left_pub_->publish(left_msg);
    right_pub_->publish(right_msg);
  }

  rclcpp::Time t0_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BimanualDemoPublisher>());
  rclcpp::shutdown();
  return 0;
}