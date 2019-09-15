#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "arm_traj_controller.hpp"
#include <control_msgs/FollowJointTrajectoryAction.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_example");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  
  control_msgs::FollowJointTrajectoryGoal goal;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::RobotTrajectory robot_trajectory;
  if (1)
  {

    ROS_INFO("Visualizing plan 1 (again)");
    goal.trajectory = display_trajectory.trajectory;
    //display_trajectory.trajectory_start = my_plan.start_state_;
    //display_trajectory.trajectory.push_back(my_plan.trajectory_);
    //display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    //sleep(5.0);
  }

  //arm_traj_controller arm("robot1/arm_controller/follow_joint_trajectory")
  control_msgs::FollowJointTrajectoryGoal goal;


  ros::shutdown();
  return 0;
}