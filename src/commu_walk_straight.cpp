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
#define HOSTNAME "commu-080.local"
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
  char const *hello = "/gesture walk"; //kyorokyoro";
  char const *walk2 = "/gesture walk";
  //char const *voice_hello = "/say_eng { hello }";
  char const *voice_hello = "/say こんにちは";
  char const *hello2 = "/gesture init";
  char const *hello3 = "/gesture ojigi";
  char const *hello4 = "/gesture uemuku";
  char buffer[1024] = {0};
  char voice_buffer[1024] = {0};
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

  time_t firstTime, afterIniTime, kyoroTime;
  const float initializationDuration = 10.0; // 10s for initialization of the robot and the rover
  bool timeOfFirstLoop = true;
  bool kyoroLoop = true;
  bool walk2Loop = true;
  int counteye = 0;
  bool gesture2Loop = true;
  bool gesture3Loop = true;
  bool sayingLoop = true;
  bool gesture4Loop = true;
  const float cylSpeed = 0.04;
  const float linSpeed = 0.15;  //was 0.05
  const float forwardingDuration = 3.0;
  const float cylPeriod = 1;
  double difference;
  long after;
  //Declares the message to be sent
  geometry_msgs::Twist msg;

  msg.linear.x = 0; msg.linear.y = 0; msg.linear.z = 0;
  msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = 0;
	      
  while(ros::ok()) {

    if(counter==0) firstTime = time(0);
    counter++;

    if(difftime(time(0), firstTime) < initializationDuration ) {
      msg.linear.x = 0;
      msg.linear.z = -cylSpeed / 2;
     }
    else{
      if(timeOfFirstLoop){
	afterIniTime = time(0);
	after = getMicrotime();
	timeOfFirstLoop = false;
      }
      else{
	difference = ((double)getMicrotime() - (double)after) / 10000;

	if(difference <= 1000){

	  if(difftime(time(0), firstTime) > (initializationDuration + 3.0)){
	    msg.linear.x = 0.2;
	    //msg.angular.z = 0.5;
	  }

	  if(((int)difference % 100 >= 0) && ((int)difference % 100 <= 15) && (kyoroLoop == true)){
	    kyoroLoop = false;
	    send(sock , hello , strlen(hello) , 0 );
	    ROS_INFO("send\n");
	  }
	
       
	  else if((int)difference % 100 >= 15) kyoroLoop = true;
      
	  if(((int)difference % 100 >= 0) && ((int)difference % 100 < 50)){
	    //kyoroTime = time(0);
	    ROS_INFO("1 diff : %lf",difference);  
	    msg.linear.z =0.04;
	  }
	  else if(((int)difference % 100 >= 50) && ((int)difference % 100 < 100)){
	    //if(difftime(time(0), kyoroTime) > 14){
	    kyoroLoop = true;
	    msg.linear.z =-0.04;
	    ROS_INFO("2 diff : %lf",difference);
	  }
	}
        else{
	  if(gesture2Loop == true){
	    send(sock , hello2 , strlen(hello2) , 0 );
	    gesture2Loop = false;
	    msg.linear.x = 0.1;
	  }
	  if(gesture4Loop == true && difference >= 2200){
	    send(sock , hello4 , strlen(hello4) , 0 );
	    gesture4Loop = false;
	  }
	  if(sayingLoop == true && difference >= 2200){
	    send(voice_sock ,voice_hello , strlen(voice_hello) , 0 );
	    sayingLoop = false;
	  }
	  if(gesture3Loop == true && difference >= 2300){
	    send(sock , hello3 , strlen(hello3) , 0 );
	    gesture3Loop = false;
	    msg.linear.x = 0;
	    msg.linear.z = 0;
	  }
	}
	  
	/*	else if(((int)difference % 100 >= 50) && ((int)difference % 100 < 75)){
	  msg.linear.z =0.04;
	  ROS_INFO("3 diff : %lf",difference);
	}
	else if(((int)difference % 100 >= 75) && ((int)difference % 100 < 100)){
	  msg.linear.z =-0.04;
	  ROS_INFO("4 diff : %lf",difference);
	  }*/
	
	

	//if(difftime(time(0), kyoroTime)>0.5) msg.linear.z = -0.04;
	//else if(difftime(time(0), kyoroTime)>=1.0) msg.linear.z = 0;
      }
    }
	 
    //Publish the message
    pub.publish(msg);

    //Delays untill it is time to send another message
    rate.sleep();
  }

}
