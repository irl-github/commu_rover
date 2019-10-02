#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 
#include <signal.h>
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h> 
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

int main(int argc, char **argv) {

  int sock = 0;
  int voice_sock = 0; 
  struct sockaddr_in serv_addr, voice_serv_addr;
  struct hostent *server, *voice_server;
  char const *hello = "/gesture atama_tonton_with_eye"; //ojigi";
  char const *init = "/gesture fast_sugoi";
  char const *voice_hello = "/say_eng { hello }"; 
  char buffer[1024] = {0};
  char voice_buffer[1024] = {0};
  struct tm *time_st;
  struct timeval myTime;
  
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
  ros::Rate rate(100);

  int counter = 0;

  time_t firstTime, afterIniTime;
  const float initializationDuration = 10.0; // 10s for initialization of the robot and the rover
  bool timeOfFirstLoop = true;
  bool tonton = true;
  bool initFlag = true;
  const float cylSpeed = 0.04;
  const float linSpeed = 0.1;  //was 0.05
  const float forwardingDuration = 10.0;
  double difference;
  long after;
  int count1,count2,count3;
  count1=0;
  count2=0;
  count3=0;

  //Declares the message to be sent
  geometry_msgs::Twist msg;

  msg.linear.x = 0; msg.linear.y = 0; msg.linear.z = 0;
  msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = 0;
	      
  while(ros::ok()) {

    if(counter==0) firstTime = time(0);
    counter++;

    if(difftime(time(0), firstTime) < initializationDuration ) {
      msg.linear.x = 0;
      msg.linear.z = -cylSpeed;

    }
    else{
      if(timeOfFirstLoop){
	afterIniTime = time(0);
	after = getMicrotime();
	timeOfFirstLoop = false;
      }
      else{
	if(difftime(time(0), afterIniTime) < forwardingDuration){
	  msg.linear.x = linSpeed;
	  if(difftime(time(0), afterIniTime) == 3.0 && tonton == true){
	    tonton = false;
	    send(sock , hello , strlen(hello) , 0 );
	    ROS_INFO("tonton");
	  }
	}
	else{
	  msg.linear.x = 0;

	  difference = ((double)getMicrotime() - (double)after) / 1000000;

	  if(difference > (forwardingDuration + 1.5) && initFlag == true){
	    initFlag = false;
	    send(sock , init , strlen(init) , 0 );
	    ROS_INFO("init");
	  }
	  
	  if((difference >= (forwardingDuration + 1.8)) && (difference < (forwardingDuration + 2.1))){
	    msg.linear.z = 0.1;
	    count1++;
	    ROS_INFO("count1,count2,count3 = %d , %d , %d",count1,count2,count3);
	    ROS_INFO("diff : %lf",difference);
	  }
	  /* else if((difference >= (forwardingDuration + 2.6)) && (difference < (forwardingDuration + 3.2))){
	    gettimeofday(&myTime, NULL);    // 現在時刻を取得してmyTimeに格納．))){
	    msg.linear.z = -0.05;
	    ROS_INFO("down");
	    count2++;
	    ROS_INFO("count1,count2,count3 = %d , %d , %d",count1,count2,count3);
	    ROS_INFO("diff : %lf",difference);   
	    }*/
	  else {
	    msg.linear.z = 0;
	    count3++;
	    ROS_INFO("count1,count2,count3 = %d , %d , %d",count1,count2,count3);
	    ROS_INFO("diff : %lf",difference);
	  }
	}
      }
    }
	 
    //Publish the message
    pub.publish(msg);

    //Delays untill it is time to send another message
    rate.sleep();
  }

}
