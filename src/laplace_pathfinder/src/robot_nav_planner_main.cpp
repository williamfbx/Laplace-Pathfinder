#include "laplace_pathfinder/robot_nav_planner.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<laplace_pathfinder::RobotNavPlanner>());
	rclcpp::shutdown();
	return 0;
}
