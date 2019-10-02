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
     ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("rover_twist", 100);

     //Sets up the random number generator
     srand(time(0));

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);

     int counter = 0;
     
       while(ros::ok()) {

	 counter++;
	 //Declares the message to be sent
	 geometry_msgs::Twist msg;

	 const int initPer = 25;

	 const int cylLoop = 100;
	 const int cylPer = 3;
	 const float cylSpeed = 0.04;
	 if(counter < initPer) msg.linear.z = -cylSpeed;
	 else{
	   if(counter % 100 < cylPer) msg.linear.z = cylSpeed;
	   else msg.linear.z = -cylSpeed;
	 }	 
	 //printf("counter = %d", counter);

	 //printf("counter = %d", counter);	 

	 const int linLoop = 200;
	 const int linPer = 9;
	 const float linSpeed = 0.05;	   
	 if(counter < initPer) {msg.linear.x = 0.0; msg.linear.y = 0.0;}
	 else{
	   if(counter % linLoop < linPer) msg.linear.x = linSpeed;
	   else if (counter % linLoop < linLoop *1/4)  msg.linear.x = 0.0;
	   else if (counter % linLoop < linLoop *1/4 +linPer)  msg.linear.x = -linSpeed;
	   else if (counter % linLoop < linLoop *2/4)  msg.linear.x = 0.0;
	   else if (counter % linLoop < linLoop *2/4 + linPer)  msg.linear.y = linSpeed;
	   else if (counter % linLoop < linLoop *3/4)  msg.linear.y = 0.0;
	   else if (counter % linLoop < linLoop *3/4 + linPer)  msg.linear.y = -linSpeed;
	   else if (counter % linLoop < linLoop)  msg.linear.y = 0.0;	   	   
	 }	 

	 const int angLoop = 80;
	 const int angPer = 9;
	 const float angSpeed = 0.5;
	 if(counter < initPer) msg.angular.z = 0.0;
	 else{
	   if(counter % angLoop < angPer) msg.angular.z = angSpeed;
	   else if (counter % angLoop < angLoop /2)  msg.angular.z = 0.0;
	   else if (counter % angLoop < angLoop /2 + angPer)  msg.angular.z = -angSpeed;
	   else if (counter % angLoop < angLoop)  msg.angular.z = 0.0;	   	   
	 }	 
	 
	 //Random x value between -2 and 2
	 //msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
	 //Random y value between -3 and 3
	 //msg.angular.z=6*double(rand())/double(RAND_MAX)-3;



	 
	 //Publish the message
	 pub.publish(msg);

	 //Delays untill it is time to send another message
	 rate.sleep();
       }

       //signal(SIGINT, mySigintHandler);
}

