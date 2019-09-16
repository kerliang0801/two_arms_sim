#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/foreach.hpp>
#include "arm_traj_controller.hpp"
#include "mongodb_store/message_store.h"

#include <sstream>
#include <cassert>

using namespace mongodb_store;
using namespace std;

int main(int argc, char **argv)
{
        // ROS INITIALIZATION
        ros::init(argc, argv, "mongo_db_extract");
        ros::NodeHandle nh;

        // MONGODB, NOTE: NAMESPACE MONGODB_STORE
        MessageStoreProxy messageStore(nh);

        
        control_msgs::FollowJointTrajectoryGoal goal;
        string name("my goal"); 
        vector< boost::shared_ptr<control_msgs::FollowJointTrajectoryGoal> > results;

        //Get it back, by default get one, QUERY BY NAME WE STORED AS
        if(messageStore.queryNamed<control_msgs::FollowJointTrajectoryGoal>(name, results)) {

                BOOST_FOREACH( boost::shared_ptr<control_msgs::FollowJointTrajectoryGoal> goal,  results)
                {
                        ROS_INFO_STREAM("Got by name: " << *goal);
                        // Try to move the arm here
                        // Create instance of the arm
                        // MOVE IT!
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

        results.clear();
        return 0;
}
