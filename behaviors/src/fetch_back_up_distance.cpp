#include <algorithm>
//#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>


//Namespaces
using namespace std;

//Global variables
ros::Publisher pub_command ;
geometry_msgs::PoseWithCovariance start;
double step_back = 0.5;

/**
  \brief Move backward in case of dynamical obstacle (with a predefined distance/time)
  */


double linear_distance(geometry_msgs::PoseWithCovariance pose1, geometry_msgs::PoseWithCovariance pose2){
        double dx = pose1.position.x - pose2.position.x;
        double dy = pose1.position.y - pose2.position.y;
        double dz = pose1.position.z - pose2.position.z;
        return sqrt(dx * dx + dy * dy + dz * dz);
 }

void OdomCallback(nav_msgs::Odometry msg_odom)
{
    geometry_msgs::Twist msg_command;
    // start = msg_odom.pose

    // (min_vel_x_=-0.5)

    double distance_from_start = linear_distance(start,msg_odom.pose);

    /* ros::Duration timeout(0.2);                              // or bool ros::Time::sleepUntil( const Time &end) : false until a specific time has been reached.
    ros::Time start_time = ros::Time::now();
    while(ros::Time::now() - start_time < timeout) { */         //use time = don't need to subscribe / or make sure only called once

    while (distance_from_start < step_back) {
        msg_command.linear.x = -0.4;
        msg_command.linear.y = 0;
        msg_command.linear.z = 0;

        msg_command.angular.x = 0;
        msg_command.angular.y = 0;
        msg_command.angular.z = 0;      // modify for obs avoidance/ turning away

        pub_command.publish(msg_command);
        distance_from_start = linear_distance(start,msg_odom.pose);     //need rate.sleep()?
    }
}



//-------------------------------------------------------------------------------------------------------------------------------

int main (int argc, char** argv)
{
  //ROS Initialization
  ros::init(argc, argv, "fetch_back_up");
  ros::NodeHandle nh_("~");


  //Subscribing
  ROS_INFO("Subscribing to topic\n");
  ros::Subscriber fetch_base = nh_.subscribe<nav_msgs::Odometry> ("/odom", 1, OdomCallback);

  //Publishing
  pub_command = nh_.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);



  ros::Rate rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("ROS-Node Terminated\n");
}
