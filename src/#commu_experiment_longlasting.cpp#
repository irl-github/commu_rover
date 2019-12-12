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
#define HOSTNAME "commu-080.local"
#define IPADD "192.168.1.149"
#define saying_num 3
#define cylinder_num 4
#define A_num 0
#define T_num 1

double A[3] = {0.005, 0.005+0.005/2, 0.01};
float oscillate_time = 10;
double T[5] = {0.4, 0.4+1.4/4, 0.4+1.4/4*2, 0.4+1.4/4*3, 1.8};

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

  //fast oscillate
  if(cylinder_num != 10){
    if(difference < oscillate_time ) msg.linear.z = (A[A_num] / T[T_num]*2*M_PI  ) * cos(difference*M_PI*2/T[T_num] );
    else msg.linear.z = 0;
  }
  
  //do nothing
  else if(cylinder_num == 10) msg.linear.z = 0;

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
    gesture1 = "/gesture happy_position";
    gesture2 = "/gesture happy_position";
  }else if(saying_num == 1){
    gesture1 = "/gesture angry_position";
    gesture2 = "/gesture angry_position";
  }else if(saying_num == 2){
    gesture1 = "/gesture sad_position";
    gesture2 = "/gesture sad_position";
  }else if(saying_num == 3){
    gesture1 = "/gesture relieved_position";
    gesture2 = "/gesture relieved_position";
  }

  else if(saying_num == 4){
    gesture1 = "/gesture surprise_position";
    gesture2 = "/gesture fast_sugoi";    
  }

  else if(saying_num == 5){
    gesture1 = "/gesture init";
    gesture2 = "/gesture happy2";
  }else if(saying_num == 6){
    gesture1 = "/gesture init";
    gesture2 = "/gesture long_angry";
  }else if(saying_num == 7){
    gesture1 = "/gesture init";
    gesture2 = "/gesture sad2";
  }else if(saying_num == 8){
    gesture1 = "/gesture init";
    gesture2 = "/gesture annshinn2";    
  }
  

  else if(saying_num == 9){
    gesture1 = "/gesture init";
    gesture2 = "/gesture init";    
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
