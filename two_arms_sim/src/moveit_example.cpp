#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "mongodb_store/message_store.h"

using namespace mongodb_store;
using namespace std;

int main(int argc, char** argv)
{
    // ROS INITIALIZARION
    ros::init(argc, argv, "moveit_example");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // MOVE GROUP STUFF
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    // PLANNING
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    
    // EXTRACTING TRAJECTORY FROM MOVE IT PLAN
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = my_plan.trajectory_.joint_trajectory;


    //--------------------------------MONGO DB STORE-------------------------------------------//
    //Create object which does the work for us.
    MessageStoreProxy messageStore(nh);
    string name("my goal");
    string id(messageStore.insertNamed(name, goal));
    cout<<"Tracjectory goal \""<<name<<"\" inserted with id "<<id<<endl;
    // update stamp
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    // store
    messageStore.updateID(id, goal);

    ros::shutdown();
    return 0;
}