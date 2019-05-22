#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>


//Namespaces
using namespace std;


/**
  \brief Turn away (left) in case of dynamical obstacle (with a predefined distance/time)
  */


int main (int argc, char** argv)
{
  //ROS Initialization
  ros::init(argc, argv, "fetch_turn_away_left");
  ros::NodeHandle nh_("~");

  //Publishing
  ros::Publisher pub_command = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate rate(100);

  geometry_msgs::Twist twist;

  ros::Duration timeout(0.5);                   //or bool ros::Time::sleepUntil( const Time &end) : false until a specific time has been reached.
  ros::Time start_time = ros::Time::now();

  while((ros::ok())&&(ros::Time::now() - start_time < timeout)){

      // max_vel_x_= 1.0; min_vel_x_= -0.5; max_vel_w_= 3.0

      twist.linear.x = 0;
      twist.linear.y = 0;
      twist.linear.z = 0;

      twist.angular.x = 0;
      twist.angular.y = 0;
      twist.angular.z = -0.5;

      pub_command.publish(twist);
      ros::spinOnce();
      rate.sleep();

  }

  ROS_INFO("ROS-Node Terminated\n");
}
