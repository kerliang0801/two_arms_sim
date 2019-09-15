#include "arm_traj_controller.hpp"


arm_traj_controller::arm_traj_controller(std::string action_server_name)
{

    // arm_controller/follow_joint_tracjectory is UR's action server's name.
    // I'm guessing in the end, you'll need to namespace the action server's name for diff arms
    // coffebot has a load_param function for retrieving server's name, find in AclrmActionController.cpp
    traj_client_ = new TrajClient(action_server_name, true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for tracjectory action server bring up");
    }
}

// NUMBER 2
// deconstructor, cleanup
arm_traj_controller::~arm_traj_controller()
{
    delete traj_client_;
}


// NUMBER 3
// Sends the command to start a given trajectory
void arm_traj_controller::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    // send goal 
    // alternatively, coffebot uses sendGoalandWait()
    traj_client_->sendGoal(goal);
}

//! Generates a simple trajectory with two waypoints, used as an example
/*! Note that this trajectory contains two waypoints, joined together
    as a single trajectory. Alternatively, each of these waypoints could
    be in its own trajectory - a trajectory can have one or more waypoints
    depending on the desired application.
*/
control_msgs::FollowJointTrajectoryGoal arm_traj_controller::arm_trajectory()
{
    // create a follow joint trajectory goal
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("elbow_joint");
    goal.trajectory.joint_names.push_back("wrist_1_joint");
    goal.trajectory.joint_names.push_back("wrist_2_joint");
    goal.trajectory.joint_names.push_back("wrist_3_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 0.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;
    goal.trajectory.points[ind].positions[6] = 0.0;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
    goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = -0.3;
    goal.trajectory.points[ind].positions[1] = 0.2;
    goal.trajectory.points[ind].positions[2] = -0.1;
    goal.trajectory.points[ind].positions[3] = -1.2;
    goal.trajectory.points[ind].positions[4] = 1.5;
    goal.trajectory.points[ind].positions[5] = -0.3;
    goal.trajectory.points[ind].positions[6] = 0.5;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
    goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState arm_traj_controller::getState()
{
    return traj_client_->getState();
}


// A. Need to test if ID not deleted, will it still be retrieved by a second node
// B. Test how would the arm move from msgs retrieved from the DB
// C. Namespacing two arms
// D. Need to check if the "try to move the arm part" has a better way for declaration, refer coffeebot's
// E, Do a remapping example??