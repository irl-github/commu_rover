#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 
#include <signal.h>
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h> 
#include <math.h>
//
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/time.h>
//
#define PORT 8079
#define PORT_TALK 8078
#define HOSTNAME "commu-081.local"
#define IPADD "192.168.1.149"
#define saying_num 0
#define cylinder_num 3

double A[8] = {0.08, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04};
float Duration[8] = {0.5, 0.8, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};

void mySigintHandler(int sig)
{
  ros::NodeHandle nh;
  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("rover_twist", 100);
  geometry_msgs::Twist msg;

  msg.linear.x = 0; msg.linear.y = 0; msg.linear.z = 0;
  msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = 0;

  for(int i=0;i<10;i++) pub.publish(msg);
  ros::shutdown();
}

long getMicrotime(){
  struct timeval currentTime;
  gettimeofday(&currentTime, NULL);
  return currentTime.tv_sec * (int)1e6 + currentTime.tv_usec;
}

geometry_msgs::Twist cylinder_movement(double difference){
  float Start = 0.3;
  geometry_msgs::Twist msg;
   
  //up
  if(cylinder_num == 0){
    if((difference < Duration[cylinder_num] )&&(difference > Start)) msg.linear.z = A[cylinder_num]  / Duration[cylinder_num];
    else msg.linear.z = 0;
  }
      
  //down
  else if(cylinder_num == 1){
    if((difference < Duration[cylinder_num] )&&(difference > Start)){
      msg.linear.z = -A[cylinder_num]  / Duration[cylinder_num];
      ROS_INFO("A; %lf, Duration; %lf, time; %lf, difference; %lf ",A[cylinder_num],Duration[cylinder_num],(double)getMicrotime()/ 1000000,difference);
    }
    else{
      msg.linear.z = 0;
      ROS_INFO("else A; %lf, Duration; %lf, time; %lf, difference; %lf ",A[cylinder_num],Duration[cylinder_num],(double)getMicrotime()/ 1000000,difference);
    }
  }

  //up and fast oscillate 
  else if(cylinder_num == 2){
    if(difference < Duration[cylinder_num]  / 2 ) msg.linear.z = A[cylinder_num]  / Duration[cylinder_num] ;
    else if((difference > Duration[cylinder_num] /2) && (difference < Duration[cylinder_num] )) msg.linear.z = -A[cylinder_num]  / Duration[cylinder_num] ;
    else msg.linear.z = 0;
  }
  //down and fast oscillate
  else if(cylinder_num == 3){
    if(difference < (Duration[cylinder_num] /4) ) msg.linear.z = A[cylinder_num]  *2 / Duration[cylinder_num] ;
    else if((difference > Duration[cylinder_num] /4) && (difference < Duration[cylinder_num] /2)) msg.linear.z = -A[cylinder_num]  *2 / Duration[cylinder_num] ;
    else msg.linear.z = 0;
  }

  //fast oscillate
  else if(cylinder_num == 4){
    if(difference < Duration[cylinder_num] ) msg.linear.z = (A[cylinder_num] * Duration[cylinder_num] / M_PI/ 2 ) * cos(difference*M_PI*2 /Duration[cylinder_num] );
    else msg.linear.z = 0;
  }

  //up and slow oscillate
  else if(cylinder_num == 5){
    if(difference < Duration[cylinder_num] ) msg.linear.z = (A[cylinder_num] * M_PI) * cos(difference*M_PI);
    else if((difference > Duration[cylinder_num] /4) && (difference < Duration[cylinder_num] /2)) msg.linear.z = -A[cylinder_num]  *2 / Duration[cylinder_num] ;
    else msg.linear.z = 0;
  }

  //down and slow oscillate
  else if(cylinder_num == 6){
    if(difference < Duration[cylinder_num] ) msg.linear.z = A[cylinder_num]  * M_PI * 2 * sin(difference*M_PI*2);
    else if((difference > Duration[cylinder_num] /4) && (difference < Duration[cylinder_num] /2)) msg.linear.z = -A[cylinder_num]  *2 / Duration[cylinder_num] ;
    else msg.linear.z = 0;
  }

  //slow oscillate
  else if(cylinder_num == 7){
    if(difference < (Duration[cylinder_num] /4) ) msg.linear.z = A[cylinder_num]  *2 / Duration[cylinder_num] ;
    else if((difference > Duration[cylinder_num] /4) && (difference < Duration[cylinder_num] /2)) msg.linear.z = -A[cylinder_num]  *2 / Duration[cylinder_num] ;
    else msg.linear.z = 0;
  }
  
  //do nothing
  else if(cylinder_num == 8) msg.linear.z = 0;

  return msg;
}
		      
int main(int argc, char **argv) {

  int sock = 0;
  int voice_sock = 0; 
  struct sockaddr_in serv_addr, voice_serv_addr;
  struct hostent *server, *voice_server;
  char buffer[1024] = {0};
  char const *gesture1;
  char const *gesture2;
  char voice_buffer[1024] = {0};
  char const *voice_hello;
  struct tm *time_st;
  struct timeval myTime;

  //[saying_num][cylinder_num] 
  //up, down, up and fast oscillate, up and slow oscillate, down and fast oscillate, down and slow oscillate, fast oscillate, slow oscillate, do nothing


  /* double A[7][10] = {{0.08, 0.04, 0.04, 0.04, 0.025, 0.05}, //happy
     		    {0.08, 0.04, 0.04, 0.04, 0.025, 0.015}, //angry
		    {0.08, 0.04, 0.04, 0.04, 0.025, 0.015}, //sad
		    {0.08, 0.04, 0.04, 0.04, 0.025, 0.015}, //pleasant
		    {0.08, 0.04, 0.04, 0.04, 0.025, 0.015}, //surprise
		    {0.08, 0.04, 0.04, 0.04, 0.025, 0.015}, //disgust
		    {0.08, 0.08, 0.04, 0.04, 0.025, 0.015}};//もとのやつ
  
  float Duration[7][10] = {{0.5, 0.8, 1.5, 1.5, 1.0, 1.5, 1.0},
			  {0.5, 0.8, 1.5, 1.5, 1.0, 1.0, 1.0},
			  {0.5, 0.8, 1.5, 1.5, 1.0, 1.5, 1.0},
			  {0.5, 0.8, 1.5, 1.5, 1.0, 1.0, 1.0},
			  {0.5, 0.8, 1.5, 1.5, 1.0, 1.0, 1.0},
			  {0.5, 0.8, 1.5, 1.5, 1.0, 1.0, 1.0},
			  {0.5, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0}}; */
  
 
  //float Start = 0.2; used this before
  
  if(saying_num == 0){
    gesture1 = "/gesture init";
    gesture2 = "/gesture happy";
  }else if(saying_num == 1){
    gesture1 = "/gesture init";
    gesture2 = "/gesture angry";
  }else if(saying_num == 2){
    gesture1 = "/gesture init";
    gesture2 = "/gesture sad";
  }else if(saying_num == 3){
    gesture1 = "/gesture init";
    gesture2 = "/gesture annshinn";
  }else if(saying_num == 4){
    gesture1 = "/gesture surprise_position";
    gesture2 = "/gesture fast_sugoi";    
  }else if(saying_num == 5){
    gesture1 = "/gesture init";
    gesture2 = "/gesture happy2";
  }else if(saying_num == 6){
    gesture1 = "/gesture init";
    gesture2 = "/gesture angry2";
  }else if(saying_num == 7){
    gesture1 = "/gesture init";
    gesture2 = "/gesture sad2";
  }else if(saying_num == 8){
    gesture1 = "/gesture init";
    gesture2 = "/gesture annshinn2";    
  } 

  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0 || (voice_sock = socket(AF_INET, SOCK_STREAM, 0) ) <0){ 
      printf("\n Socket creation error \n"); 
      return -1; 
  } 
   
  serv_addr.sin_family = AF_INET; 
  serv_addr.sin_port = htons(PORT);
  voice_serv_addr.sin_family = AF_INET;
  voice_serv_addr.sin_port = htons(PORT_TALK);

#ifdef HOSTNAME
	
	server = gethostbyname(HOSTNAME);
	voice_server = gethostbyname(HOSTNAME);
	if (server == NULL || voice_server == NULL) {
	  printf("ERROR, no such host\n");
	  return -1;
	}
	bcopy((char *)server->h_addr, 
	      (char *)&serv_addr.sin_addr.s_addr,
	      server->h_length);
	bcopy((char *)voice_server->h_addr, 
	      (char *)&voice_serv_addr.sin_addr.s_addr,
	      voice_server->h_length);
#else  
  // Convert IPv4 and IPv6 addresses from text to binary form 
  if(inet_pton(AF_INET, IPADD, &serv_addr.sin_addr)<=0 || inet_pton(AF_INET, IPADD, &voice_serv_addr.sin_addr)<=0) { 
      printf("\nInvalid address/ Address not supported \n"); 
      return -1; 
  }  
#endif

  //Connet
  if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0 || connect(voice_sock, (struct sockaddr *)&voice_serv_addr, sizeof(voice_serv_addr)) < 0 ){ 
      printf("\nConnection Failed \n"); 
      return -1;
  } 
  
  //Initializes ROS, and sets up a node
  ros::init(argc, argv, "random_husky_commands", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  signal(SIGINT, mySigintHandler);

  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("rover_twist", 100);

  //Sets up the random number generator
  srand(time(0));

  //Sets the loop to publish at a rate of 10Hz
  ros::Rate rate(150);

  int counter = 0;

  time_t firstTime, afterIniTime;
  const float initializationDuration = 10.0; // 10s for initialization of the robot and the rover
  bool timeOfFirstLoop = true;
  const float cylSpeed = 0.04;
  const float linSpeed = 0.15;  //was 0.05
  double difference;
  long after;
  
  //Declares the message to be sent
  geometry_msgs::Twist msg;

  msg.linear.x = 0; msg.linear.y = 0; msg.linear.z = 0;
  msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = 0;
	      
  while(ros::ok()) {

    if(counter==0) {
      firstTime = time(0);
      send(sock , gesture1 , strlen(gesture1) , 0 );
      counter++;
    }

    if(difftime(time(0), firstTime) < initializationDuration ) {
      msg.linear.x = 0;
    }
    else{
      if(timeOfFirstLoop){
	afterIniTime = time(0);
	after = getMicrotime();
	timeOfFirstLoop = false;
	send(sock , gesture2 , strlen(gesture2) , 0 );    
      }
      difference = ((double)getMicrotime() - (double)after) / 1000000;
      msg = cylinder_movement(difference);
    }

    //Publish the message
    pub.publish(msg);

    //Delays untill it is time to send another message
    rate.sleep();
  }

}
