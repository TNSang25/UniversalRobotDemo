#include <memory>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("s_drawer_node");

  // Create node with Automatically generated node name
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("s_drawer_node", node_options);

  // We spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Create publisher for RViz trajectory visualization
  rclcpp::QoS qos(10);
  qos.transient_local();
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", qos);

  // Explicitly clear pipeline and planner IDs so it uses the default one configured in move_group
  move_group_interface.setPlanningPipelineId("");
  move_group_interface.setPlannerId("");
  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);
  move_group_interface.setEndEffectorLink("tool0");

  // Get current pose
  auto current_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Current End Effector Pose: x=%f, y=%f, z=%f",
              current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

  // Wait a bit before starting
  rclcpp::sleep_for(std::chrono::seconds(2));

  // 1. Move to a safe "home" joint configuration to ensure we are not near singularities
  RCLCPP_INFO(logger, "Moving to safe joint configuration...");
  std::vector<double> joint_group_positions;
  bool success_joint_state = move_group_interface.getActiveJoints().size() > 0;
  
  if (success_joint_state) {
      // Common safe position (e.g. elbow bent)
      joint_group_positions = {
          0.0,            // shoulder_pan_joint
          -1.5708,        // shoulder_lift_joint
          1.5708,         // elbow_joint
          -1.5708,        // wrist_1_joint
          -1.5708,        // wrist_2_joint
          0.0             // wrist_3_joint
      };
      move_group_interface.setJointValueTarget(joint_group_positions);
      moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
      bool success = (move_group_interface.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success) {
          move_group_interface.execute(joint_plan);
          RCLCPP_INFO(logger, "Reached safe joint configuration!");
      } else {
          RCLCPP_WARN(logger, "Failed to plan path to safe joint configuration! Will attempt to draw from current pose anyway.");
      }
  }

  // Get updated pose
  current_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Updated End Effector Pose: x=%f, y=%f, z=%f",
              current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

  // 2. Generate Cartesian waypoints for an S shape in the XY plane
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  // Center of the S
  double center_x = current_pose.pose.position.x;
  double center_y = current_pose.pose.position.y;
  double z_fixed = current_pose.pose.position.z;
  
  double radius = 0.05; // 5 cm radius for each half of the S
  int num_points = 30; // Number of points per half

  // First, move to the S starting point (top right)
  geometry_msgs::msg::Pose target_pose = current_pose.pose;
  target_pose.position.x = center_x + radius;
  target_pose.position.y = center_y + radius;
  target_pose.position.z = z_fixed;

  RCLCPP_INFO(logger, "Moving to start point of the S...");
  move_group_interface.setPoseTarget(target_pose);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    move_group_interface.execute(my_plan);
    RCLCPP_INFO(logger, "Reached start point!");
  } else {
    RCLCPP_ERROR(logger, "Failed to plan path to start point!");
    rclcpp::shutdown();
    return 1;
  }

  // Generate Cartesian waypoints for the S shape
  RCLCPP_INFO(logger, "Generating S trajectory waypoints...");
  
  // Top half: center at (center_x, center_y + radius)
  // Angle from 0 to 3*pi/2 (counter-clockwise)
  double cx1 = center_x;
  double cy1 = center_y + radius;
  for (int i = 0; i <= num_points; ++i) {
    geometry_msgs::msg::Pose wp = target_pose;
    double angle = 0.0 + (1.5 * M_PI * i) / num_points;
    wp.position.x = cx1 + radius * cos(angle);
    wp.position.y = cy1 + radius * sin(angle);
    wp.position.z = z_fixed;
    wp.orientation = current_pose.pose.orientation;
    waypoints.push_back(wp);
  }

  // Bottom half: center at (center_x, center_y - radius)
  // Angle from pi/2 to -pi (clockwise)
  double cx2 = center_x;
  double cy2 = center_y - radius;
  for (int i = 1; i <= num_points; ++i) { // start at 1 to avoid duplicate middle point
    geometry_msgs::msg::Pose wp = target_pose;
    double angle = (0.5 * M_PI) - (1.5 * M_PI * i) / num_points;
    wp.position.x = cx2 + radius * cos(angle);
    wp.position.y = cy2 + radius * sin(angle);
    wp.position.z = z_fixed;
    wp.orientation = current_pose.pose.orientation;
    waypoints.push_back(wp);
  }

  // Publish waypoints to RViz as a Marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = move_group_interface.getPlanningFrame();
  marker.header.stamp = node->now();
  marker.ns = "trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01; // 1 cm line width
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.pose.orientation.w = 1.0; // CRITICAL: valid quaternion
  
  for (const auto& wp : waypoints) {
    marker.points.push_back(wp.position);
  }
  
  RCLCPP_INFO(logger, "Publishing trajectory marker to RViz...");
  // Publish multiple times to ensure RViz receives it
  for (int i = 0; i < 5; ++i) {
    marker_pub->publish(marker);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  // Compute Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  
  RCLCPP_INFO(logger, "Computing Cartesian path...");
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  
  RCLCPP_INFO(logger, "Path computed (%.2f%% achieved)", fraction * 100.0);

  if (fraction > 0.9) {
    RCLCPP_INFO(logger, "Executing Cartesian path...");
    move_group_interface.execute(trajectory);
    RCLCPP_INFO(logger, "Execution completed!");
  } else {
    RCLCPP_WARN(logger, "Failed to compute complete Cartesian path. Skipping execution.");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
