#include <algorithm>
#include <random>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <sensor_msgs/JointState.h>


//Namespaces
using namespace std;

/**
  \brief Look around (mapping task)
  */


int main (int argc, char** argv)
{
  // ROS Initialization
    ros::init(argc, argv, "fetch_random_gaze");
    ros::NodeHandle nh_("~");

  // Action Client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client ("head_controller/follow_joint_trajectory", true);
    client.waitForServer(); //will wait for infinite time

    ros::Rate rate(0.2);            // move head every 5 seconds
    while (ros::ok())
    {

      // Joint limits
          double max_pos_pan_ = 1.57;              // min_pos_pan_ = -1.57;
          double min_pos_tilt_ = -0.76;
          double max_pos_tilt_ = 1.45;
          double max_vel_ = 1.57;                   // limit velocity : 1.57;

      // Random pan and tilt

          double pan = -max_pos_pan_ + rand() % (( max_pos_pan_ + 1 ) + max_pos_pan_);      // random pan

          double tilt = max_pos_tilt_;                                                      // tilt follows a normal distribution
          std::default_random_engine generator;
          std::normal_distribution<double> distr2(1.1,2.0);
          while ((tilt>=max_pos_tilt_)||(tilt<=min_pos_tilt_)) {
              tilt = distr2(generator);
          }

      // Head motion
          control_msgs::FollowJointTrajectoryGoal goal;
          goal.trajectory.joint_names.push_back("head_pan_joint");
          goal.trajectory.joint_names.push_back("head_tilt_joint");

          trajectory_msgs::JointTrajectoryPoint p;
          p.positions.push_back(pan);
          p.positions.push_back(tilt);
          p.time_from_start = ros::Duration(0.1);

          goal.trajectory.points.push_back(p);
          goal.goal_time_tolerance = ros::Duration(2.0);

          ROS_INFO("Submitting goal");
          client.sendGoal(goal);
          ROS_INFO("Waiting");
          client.waitForResult(ros::Duration(2.0));

          ros::spinOnce();
          rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}
