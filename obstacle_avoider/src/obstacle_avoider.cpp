#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ObstacleAvoider : public rclcpp::Node
{
public:
    ObstacleAvoider() : Node("obstacle_avoider")
    {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ObstacleAvoider::laserCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Avoider Node Started");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float front = msg->ranges[msg->ranges.size() / 2];

        geometry_msgs::msg::Twist cmd;

        if (front < 0.5) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5;
        } else {
            cmd.linear.x = 0.3;
            cmd.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoider>());
    rclcpp::shutdown();
    return 0;
}
