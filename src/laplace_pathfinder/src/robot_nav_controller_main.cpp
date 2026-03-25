#include "laplace_pathfinder/robot_nav_controller.hpp"

#include <memory>

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<laplace_pathfinder::RobotNavController>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
