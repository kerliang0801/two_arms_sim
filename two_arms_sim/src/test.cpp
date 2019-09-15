#include "mongodb_store/message_store.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/foreach.hpp>
#include "arm_traj_controller.hpp"

#include <sstream>
#include <cassert>

using namespace mongodb_store;
using namespace std;

int main(int argc, char **argv)
{
        ros::init(argc, argv, "example_mongodb_store_cpp_client");
        ros::NodeHandle nh;

        //Create object which does the work for us.
        MessageStoreProxy messageStore(nh);

        //This is the message we want to store
        control_msgs::FollowJointTrajectoryGoal goal;

        string name("my goal");
        //Insert something with a name, storing id too
        string id(messageStore.insertNamed(name, goal));
        cout<<"Tracjectory goal \""<<name<<"\" inserted with id "<<id<<endl;

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

        // maybe this????????????????????????
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);


        messageStore.updateID(id, goal);

        // now test it worked
        // assert(messageStore.queryID<control_msgs::FollowJointTrajectoryGoal>(id).first->position.z == 666);

        vector< boost::shared_ptr<control_msgs::FollowJointTrajectoryGoal> > results;

        //Get it back, by default get one
        if(messageStore.queryNamed<control_msgs::FollowJointTrajectoryGoal>(name, results)) {

                BOOST_FOREACH( boost::shared_ptr<control_msgs::FollowJointTrajectoryGoal> goal,  results)
                {
                        ROS_INFO_STREAM("Got by name: " << *goal);
                }
        }

        results.clear();
        // Need to delete ID.
        if(messageStore.queryID<control_msgs::FollowJointTrajectoryGoal>(id, results)) {

                BOOST_FOREACH( boost::shared_ptr<control_msgs::FollowJointTrajectoryGoal> goal,  results)
                {
                        ROS_INFO_STREAM("Got by ID: " << *goal);

                        // Try to move the arm here
                        // Create instance of the arm
                        ROS_INFO_STREAM("robot1/arm_controller/follow_joint_tracjectory");
                        arm_traj_controller arm("robot1/arm_controller/follow_joint_trajectory");
                        arm_traj_controller arm2("robot2/arm_controller/follow_joint_trajectory");
                        arm.startTrajectory(*goal);
                        while(!arm.getState().isDone() && ros::ok())
                        {
                            usleep(50000);
                        }
                        arm2.startTrajectory(*goal);
                        while(!arm2.getState().isDone() && ros::ok())
                        {
                            usleep(50000);
                        }
                }
        }

        messageStore.deleteID(id);

        results.clear();
        ROS_INFO_STREAM("THIS SHOULD BE EMPTY");
        if(messageStore.queryID<control_msgs::FollowJointTrajectoryGoal>(id, results)) {

                BOOST_FOREACH( boost::shared_ptr<control_msgs::FollowJointTrajectoryGoal> goal,  results)
                {
                        ROS_INFO_STREAM("Got by ID: " << *goal);
                        
                }

        }
        return 0;
}
