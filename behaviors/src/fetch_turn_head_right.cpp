#include <algorithm>
#include <functional>
#include <atomic>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <sensor_msgs/JointState.h>


//Namespaces
using namespace std;

/**
  \brief Captures the current joint state of the robot and stores it in a global variable to be accesible by the rest of functions
  */

std::atomic<bool> moving(false);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg_state, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &client)
{
  if (msg_state->name.size()!= 13)                                // not the gripper joint states
  {
    return;
  }
  moving = true;

  double last_pan_ ;
  double last_tilt_ ;
  double desired_pan_ ;
  double desired_tilt_ ;

  // Update head joints positions
  for( unsigned i = 0 ; i < msg_state->name.size() ; i++ )
  {
    if (msg_state->name[i] == "head_pan_joint"){
      last_pan_ = msg_state->position[i];
      desired_pan_ = last_pan_ - 1.2 ;                        //incr?
    }

    if (msg_state->name[i] == "head_tilt_joint"){
      last_tilt_ = msg_state->position[i];
      desired_tilt_ = last_tilt_ + 1 ;
    }
  }


   //Joint limits
       double max_pos_pan_ = 1.57;
       double min_pos_tilt_ = -0.76;
       double max_pos_tilt_ = 1.45;
       double max_vel_ = 1.4;                   // limit velocity for both pan and tilt : 1.57 ;
       double max_acc_pan_ = 3.0;
       double max_acc_tilt_ = 3.0;

   // Fill in message
        double pan = std::min(std::max(desired_pan_, -max_pos_pan_), max_pos_pan_);

        double tilt = std::min(std::max(desired_tilt_, min_pos_tilt_), max_pos_tilt_);

   // Action
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("head_pan_joint");
        goal.trajectory.joint_names.push_back("head_tilt_joint");

        trajectory_msgs::JointTrajectoryPoint p;
        p.positions.push_back(pan);
        p.positions.push_back(tilt);
        p.velocities.push_back(max_vel_);
        p.velocities.push_back(max_vel_);
        p.time_from_start = ros::Duration(0.1);

        goal.trajectory.points.push_back(p);
        goal.goal_time_tolerance = ros::Duration(2.0);

        ROS_INFO("Submitting goal");
        client.sendGoal(goal);
        ROS_INFO("Waiting");
        client.waitForResult(ros::Duration(5.0));


        ROS_INFO("Shutting down...");
        ros::shutdown();

  }




//-------------------------------------------------------------------------------------------------------------------------------

int main (int argc, char** argv)
{
  //ROS Initialization
  ros::init(argc, argv, "fetch_turn_head_right");
  ros::NodeHandle nh_("~");

  ROS_INFO("Connected to master");
  // Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client ("head_controller/follow_joint_trajectory", true);
  client.waitForServer(); //will wait for infinite time
  auto cb = std::bind(jointStateCallback, std::placeholders::_1, std::ref(client));
  ROS_INFO("Connected to action server");

  //Subscribing
  ROS_INFO("Subscribing to topic\n");
  ros::Subscriber fetch_joint_state = nh_.subscribe<sensor_msgs::JointState> ("/joint_states", 1, cb);

  ros::Rate rate(5);
  while (ros::ok()) {
    ros::spinOnce();
    if (moving) {
      fetch_joint_state.shutdown();
    }
    rate.sleep();
    ROS_INFO("...");
  }

  ROS_INFO("ROS-Node Terminated\n");
}
