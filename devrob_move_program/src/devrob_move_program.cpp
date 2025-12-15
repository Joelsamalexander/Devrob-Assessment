#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>
  (
    "devrob_move_program",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("devrob_move_program");
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface_arm = MoveGroupInterface(node, "arm");
  
  move_group_interface_arm.setPlanningPipelineId("ompl");
  move_group_interface_arm.setPlannerId("RRTConnectkConfigDefault");
  move_group_interface_arm.setPlanningTime(30.0);


move_group_interface_arm.setStartStateToCurrentState();


geometry_msgs::msg::Pose start_pose =
  move_group_interface_arm.getCurrentPose().pose;

std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(start_pose);


geometry_msgs::msg::Pose target_pose = start_pose;

target_pose.position.z -= 0.02;
waypoints.push_back(target_pose);  // down

target_pose.position.y -= 0.00;
waypoints.push_back(target_pose);  // right

target_pose.position.z += 0.02;
target_pose.position.y += 0.0;
target_pose.position.x -= 0.0;
waypoints.push_back(target_pose);  // up and left


moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
const double eef_step = 0.01; 
const double jump_threshold = 0.0; 

double fraction = move_group_interface_arm.computeCartesianPath(
  waypoints,
  eef_step,
  jump_threshold,
  cartesian_trajectory
);

RCLCPP_INFO(
  logger,
  "Cartesian path achieved %.2f%%",
  fraction * 100.0
);


if (fraction > 0.9)
{
  move_group_interface_arm.execute(cartesian_trajectory);
}
else
{
  RCLCPP_WARN(logger, "Cartesian path planning incomplete");
}

  rclcpp::shutdown();
  return 0;
}

