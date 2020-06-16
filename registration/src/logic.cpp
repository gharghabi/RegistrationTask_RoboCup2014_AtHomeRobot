#include "ros/ros.h"
#include <stdio.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include "athomerobot_msgs/arm.h"
#include "athomerobot_msgs/omnidata.h"
#include "athomerobot_msgs/head.h"
#include "athomerobot_msgs/irsensor.h"
#include "athomerobot_msgs/motortorques.h"

#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/JointState.h>

#include <string>
#include <iostream>



using namespace std;

ros::Publisher Text_pub;
std_msgs::String TextForSpeech;

void shutdown_robot()
{
    /*
      Dast Paeen, Roo Be Davar, Kinect Paeen, Badaneh Paeen,Be Halat Shutdown
    */
}

void init_robot(){
    /*
      Dast Ha Paeen, Kinect Bala, Badaneh Bala
    */


}

void function1(){

    //My Name Is Sepanta
    // Sare Kinect Ro Boland Kone Va Roo Be Davar Ha Bashe.
    TextForSpeech.data = "start_state_1";
    Text_pub.publish(TextForSpeech);
    cout<<"State One Start"<<endl<<endl;
    usleep(1000000);
}
void function2(){
    //
    //Dast Ro Be Samte Bala Bebare Va Bege, Dota Dast Daram

    TextForSpeech.data = "start_state_2";
    Text_pub.publish(TextForSpeech);
    cout<<"State two Start"<<endl<<endl;
    usleep(1000000);
}
void function3(){

    //Dast Ro Be Samte Kinect Bala Bebare , Cheshm Daram
    TextForSpeech.data = "start_state_3";
    Text_pub.publish(TextForSpeech);
    cout<<"State three Start"<<endl<<endl;
    usleep(1000000);
}
void function4(){
    //Dast Ro Bebare Bala Samte Microphone , Goosh Daram
    TextForSpeech.data = "start_state_4";
    Text_pub.publish(TextForSpeech);
    cout<<"State Four Start"<<endl<<endl;
    usleep(1000000);
}
void function5(){
    //Ye Charkh Bezane Ba Charkh hash, Bege 4 Ta Charkh Daram
    TextForSpeech.data = "start_state_5";
    Text_pub.publish(TextForSpeech);
    cout<<"State Five Start"<<endl<<endl;
    usleep(1000000);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "InTheLogic");
    ros::NodeHandle SpeechNodehandler;
    Text_pub = SpeechNodehandler.advertise<std_msgs::String>("/AUTROBOTOUT_speak", 1);
    const int MAXITEMS = 5;
    string lastItem;

     /*init_robot()
     *
     * Reset State Gusture Of Robot
     */

    init_robot();
    string inventory[MAXITEMS] = {"function5", "function1", "function4", "function2"};
    //string inventory[MAXITEMS] = {"function5", "function1", "function4", "function2", "function3"};
     for (int i = 0; i < MAXITEMS; ++i)
     {
         lastItem = inventory[i];
         if(lastItem=="function1"){
             function1();
             continue;
         }
         if(lastItem=="function2"){
             function2();
             continue;
         }
         if(lastItem=="function3"){
             function3();
             continue;
         }
         if(lastItem=="function4"){
             function4();
             continue;
         }
         if(lastItem=="function5")
         {
             function5();
             continue;
         }
     }
}
