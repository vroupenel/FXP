#include <algorithm>

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
  //ROS Initialization
      ros::init(argc, argv, "fetch_look_around");
      ros::NodeHandle nh_("~");

  // Action Client
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client ("head_controller/follow_joint_trajectory", true);
      client.waitForServer(); //will wait for infinite time

/*  //Joint limits
       double max_pos_pan_ = 1.57;              // min_pos_pan_ = -1.57;
       double min_pos_tilt_ = -0.76;
       double max_pos_tilt_ = 1.45;
       double max_vel_ = 1.57;                   // limit velocity : 1.57;
*/

   // Head predefined motion
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.joint_names.push_back("head_pan_joint");
      goal.trajectory.joint_names.push_back("head_tilt_joint");

      trajectory_msgs::JointTrajectoryPoint p1;
      p1.positions.push_back(0);                       // pan (radians)
      p1.positions.push_back(0);                       // tilt (radians)
      p1.time_from_start = ros::Duration(0.1);

      trajectory_msgs::JointTrajectoryPoint p2;
      p2.positions.push_back(0);                       // pan (radians)
      p2.positions.push_back(1.4);                    // tilt (radians)
      p2.time_from_start = ros::Duration(2);

      trajectory_msgs::JointTrajectoryPoint p3;
      p3.positions.push_back(-1.5);                    // pan (radians)
      p3.positions.push_back(1.0);                    // tilt (radians)
      p3.time_from_start = ros::Duration(4);

      trajectory_msgs::JointTrajectoryPoint p4;
      p4.positions.push_back(-1.5);                    // pan (radians)
      p4.positions.push_back(-0.5);                    // tilt (radians)
      p4.time_from_start = ros::Duration(6);

      trajectory_msgs::JointTrajectoryPoint p5;
      p5.positions.push_back(0);                       // pan (radians)
      p5.positions.push_back(-0.5);                    // tilt (radians)
      p5.time_from_start = ros::Duration(8);

      trajectory_msgs::JointTrajectoryPoint p6;
      p6.positions.push_back(1.5);                     // pan (radians)
      p6.positions.push_back(-0.5);                    // tilt (radians)
      p6.time_from_start = ros::Duration(10);

      trajectory_msgs::JointTrajectoryPoint p7;
      p7.positions.push_back(1.5);                     // pan (radians)
      p7.positions.push_back(1.0);                    // tilt (radians)
      p7.time_from_start = ros::Duration(12);

      trajectory_msgs::JointTrajectoryPoint p8;
      p8.positions.push_back(0);                       // pan (radians)
      p8.positions.push_back(1.4);                    // tilt (radians)
      p8.time_from_start = ros::Duration(14);

      trajectory_msgs::JointTrajectoryPoint p9;
      p9.positions.push_back(0);                     // pan (radians)
      p9.positions.push_back(0);                    // tilt (radians)
      p9.time_from_start = ros::Duration(16);

      goal.trajectory.points.push_back(p1);
      goal.trajectory.points.push_back(p2);
      goal.trajectory.points.push_back(p3);
      goal.trajectory.points.push_back(p4);
      goal.trajectory.points.push_back(p5);
      goal.trajectory.points.push_back(p6);
      goal.trajectory.points.push_back(p7);
      goal.trajectory.points.push_back(p8);
      goal.trajectory.points.push_back(p9);

      goal.goal_time_tolerance = ros::Duration(2.0);

      ROS_INFO("Submitting goal");
      client.sendGoal(goal);
      ROS_INFO("Waiting");
      client.waitForResult(ros::Duration(17));



    ROS_INFO("ROS-Node Terminated\n");
}
