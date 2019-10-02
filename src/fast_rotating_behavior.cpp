#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 
#include <signal.h>

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()

  //ros::init("turn_off_rand_husky");
  ros::NodeHandle nh;

  //Ceates the publisher, and tells it to publish
  //to the husky/cmd_vel topic, with a queue size of 100
  //ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("husky/cmd_vel", 100);

  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("rover_twist", 100);
  geometry_msgs::Twist msg;

  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
	      
  for(int i=0;i<10;i++) pub.publish(msg);
  
  ros::shutdown();
}

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "random_husky_commands", ros::init_options::NoSigintHandler);
     ros::NodeHandle nh;

     signal(SIGINT, mySigintHandler);

     //Ceates the publisher, and tells it to publish
     //to the husky/cmd_vel topic, with a queue size of 100
     //ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("husky/cmd_vel", 100);
     ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("rover_twist", 512);

     //Sets up the random number generator
     //srand(time(0));

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);

     int counter = 0;
     geometry_msgs::Twist msg;
     
     while(ros::ok()) {

       counter++;

       msg.linear.x = 0.2;
       msg.angular.z = 0.5;
       
       //Publish the message
       pub.publish(msg);

       //Delays untill it is time to send another message
       rate.sleep();
     }

     //signal(SIGINT, mySigintHandler);
}

