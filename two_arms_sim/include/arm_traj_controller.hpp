
#ifndef ARM_TRAJ_CONTROLLER
#define ARM_TRAJ_CONTROLLER


#include "mongodb_store/message_store.h"
#include "geometry_msgs/Pose.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/foreach.hpp>

#include <sstream>
#include <cassert>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class arm_traj_controller{
    
    private:

    // simple action client for tracjectory action
    // used to trigger arm movement action
    // coffeebot's uses std::unique_ptr, find his instance in ArmActionController.hpp 
    TrajClient* traj_client_;

    public:
    // constructor: creates an action server's client instance
    arm_traj_controller(std::string action_server_name);
    // deconstructor: clean up the client instance
    ~arm_traj_controller();

    // adds a header stamp to FollowJointTrajectoryAction
    void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal);

    //! Generates a simple trajectory with two waypoints, used as an example
    /*! Note that this trajectory contains two waypoints, joined together
        as a single trajectory. Alternatively, each of these waypoints could
        be in its own trajectory - a trajectory can have one or more waypoints
        depending on the desired application.
    */
    control_msgs::FollowJointTrajectoryGoal arm_trajectory();

    // Returns the current state of the action
    actionlib::SimpleClientGoalState getState();
};

#endif