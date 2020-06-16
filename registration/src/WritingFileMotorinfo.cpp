#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include "athomerobot_msgs/arm.h"
#include "athomerobot_msgs/omnidata.h"
#include "athomerobot_msgs/head.h"
#include "athomerobot_msgs/irsensor.h"
#include "athomerobot_msgs/motortorques.h"


#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/JointState.h>

#include <math.h>
#include <sstream>
#include <boost/thread.hpp>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <fstream>
#include <unistd.h>

#include <boost/thread.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;


  float shoulderR,elbowR,wristR,handR; 
  float shoulderL,elbowL,wristL,handL;
  
  clock_t begin, end;


int s_Motor[17] = {0};//defualtesh avaz she ya chand khate avval unjakhunde nashe
int p_Motor[17] = {0};

void chatterCallbackw(const dynamixel_msgs::MotorStateList::ConstPtr& msg)
{
  for ( int i = 0 ; i < 17 ; i++ )
  {
      s_Motor[i] = msg->motor_states[i].speed;
      p_Motor[i] = msg->motor_states[i].position;
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "writing_to_file");
  ros::Time::init();

  // ros::ServiceClient client[4];
  // dynamixel_controllers::TorqueEnableRequest srv;



  //  ros::NodeHandle n_client;
  //   client[0] = n_client.serviceClient<dynamixel_controllers::TorqueEnableRequest>("/rightshoulderPitch_controller/torque_enable");
  //   client[1] = n_client.serviceClient<dynamixel_controllers::TorqueEnableRequest>("/rightshoulderRoll_controller/torque_enable");
  //   client[2] = n_client.serviceClient<dynamixel_controllers::TorqueEnableRequest>("/rightelbow_controller/torque_enable");
  //   client[3] = n_client.serviceClient<dynamixel_controllers::TorqueEnableRequest>("/rightwristPitch_controller/torque_enable");
  //   client[4] = n_client.serviceClient<dynamixel_controllers::TorqueEnableRequest>("/rightwristRoll_controller/torque_enable");
   



  //   for (int i=0 ; ++i;i<4)
  //   {
  //        srv.torque_enable= false;
  //       // client[i].call(srv);
  //   }

  // ofstream motors_info;
   //motors_info.open("/home/autathome/catkinws/src/athomerobot/handDetectWithProcessing/src/motors_info.txt", ios_base::in);
  
  ofstream motors_info (argv[1]);//
  //ofstream motors_info ("/home/autathome/catkinws/src/athomerobot/handDetectWithProcessing/src/motors_info0.txt");  
  ros::Rate loop_rate(20);
    
    float tic = cv::getTickCount();
    float toc = (cv::getTickCount() - tic) / cv::getTickFrequency();
	  printf("Normalization Time: %f milliseconds\n",toc );

    ros::NodeHandle n_motors;
    ros::Subscriber sub = n_motors.subscribe("/motor_states/dx_port", 1, chatterCallbackw);
    usleep(1000000);
    //Comment motor 6 va 7 baraye dast hast moghe ejra havaset bashe 
    while (ros::ok() )
    { 
    if (motors_info.is_open())
      {
        //9 kinect
	      toc = (cv::getTickCount() - tic) / cv::getTickFrequency();
	      motors_info <<toc<<" "<<p_Motor[0]<<" "<<p_Motor[1]<<" "<<p_Motor[2]<<" "<<p_Motor[3]<<" "<<p_Motor[4]<<" "<<p_Motor[7]<<" "<<p_Motor[9]<<"\n";	         
      }
    else 
      cout << "Unable to open file";
      ros::spinOnce();
      loop_rate.sleep();
    } 
  motors_info.close();
  return 0;    
}




    
