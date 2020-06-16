#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <athomerobot_msgs/gripAction.h>
#include <athomerobot_msgs/objectAction.h>
#include <athomerobot_msgs/sepantaAction.h>
#include "athomerobot_msgs/arm.h"
#include "athomerobot_msgs/head.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include <boost/thread.hpp>
#include <math.h>
#include <iostream>
#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/JointState.h>
using namespace std;
#define MOVE_TIME 70

typedef actionlib::SimpleActionServer<athomerobot_msgs::gripAction> grip_Server;
typedef actionlib::SimpleActionClient<athomerobot_msgs::objectAction> object_Client;
typedef actionlib::SimpleActionClient<athomerobot_msgs::sepantaAction> move_client;
typedef athomerobot_msgs::arm ArmMsg;
typedef std_msgs::Int32 int_msg;
typedef athomerobot_msgs::head head_msg;

int head_tilt_gl = 1800;
ros::Publisher arm_right;
ros::Publisher gripper_right;
ros::Publisher head;
ros::Publisher desired_z;

float omni_y;
float omni_x;
bool flag_check1 = false;
bool flag_check2 = false;
bool flag_check3 = false;

int  objectZ = 0; 
int objecty = 0;
struct position {
    float x;
    float y;
    float z;
};

float L1 = 0.255;
float L2 = 0.225;
float L3 = 0.17; // length of griper
float kinect2arm_z = 0.22; // offset bitween kinect and arm
float kinect2arm_y = 0.19; // offset bitween kinect and arm
float h_table=0.73;//az zamin 74 ast 5 mil ezafe mizare
float h_arm=1.14;
float h_obj=0.12;

head_msg my_head_msg;
int_msg z_msg;
bool globalRuningFlag_grip = true ;
int state = -1;

grip_Server *globalServer_grip;
object_Client *globalClient_object;
move_client *globalClient_sepantaMove;
// vector <pair<std::string , std::string>> place_name = { {"coca", "PPG"}, {"fix", "PPG"}};
position object_function(string cmd_object);

void PreemptThreadGrip() {

    while (globalRuningFlag_grip) {
        //   // check that preempt has not been requested by the client
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        if (globalServer_grip->isPreemptRequested() || !ros::ok()) {
            ROS_INFO(" Grip Preempted\n");
            // set the action state to preempted
            globalClient_object->cancelGoal();
            globalClient_sepantaMove->cancelGoal();
            globalRuningFlag_grip = false;
            globalServer_grip->setPreempted();
            ros::shutdown();
        }

    }
}


position object_function(string cmd_object) {
    position object_result;

    object_Client ac_object("Object_Reg_action", true);
    globalClient_object = &ac_object;
    ac_object.waitForServer();
    athomerobot_msgs::objectGoal goal_object;
    goal_object.input = cmd_object;

    ROS_INFO("send goal to reg for obj");
    ac_object.sendGoal(goal_object);
    //wait for the action to return

    bool finished_before_timeout = ac_object.waitForResult();//(ros::Duration(1));
    ROS_INFO("wait for object to be done");
    actionlib::SimpleClientGoalState state_ac_object = ac_object.getState();
    if (state_ac_object == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Action_obj finished\n");
        athomerobot_msgs::objectResultConstPtr result_object;
        result_object = ac_object.getResult();

        object_result.x = result_object->x;
        object_result.y = result_object->y;
        object_result.z = result_object->z;
        cout << "xk1......: " << object_result.x << endl;
        cout << "yk1......: " << object_result.y << endl;
        cout << "zk1......: " << object_result.z << endl;

    } else {
        ROS_INFO("Finished current State: %s\n", ac_object.getState().toString().c_str());
    }
    return object_result;
}

ArmMsg arm_ready_right() {
    ArmMsg my_arm_msg;
    // mode=1;
    my_arm_msg.shoulder_pitch = 3400;
    my_arm_msg.shoulder_roll = 2048;
    my_arm_msg.elbow = 750;
    my_arm_msg.wrist_pitch = 1900;
    my_arm_msg.wrist_roll = 2048;

    return my_arm_msg;

}
ArmMsg arm_pick(ArmMsg arm) {
    ArmMsg my_arm_msg;
    // mode=1;
    my_arm_msg.shoulder_pitch = arm.shoulder_pitch+100;
    my_arm_msg.shoulder_roll = 2048;
    my_arm_msg.elbow = arm.elbow - 200;
    my_arm_msg.wrist_pitch = arm.wrist_pitch - 200;
    my_arm_msg.wrist_roll = 2048;

    return my_arm_msg;

}

void sepanta_move(string cmd_move, int value) {

std::cout<<"value "<< value<< "cmd_move "<<cmd_move<< std::endl;
    move_client move_action("sepanta_action", true);//******************
    globalClient_sepantaMove = &move_action;
    move_action.waitForServer();

    ROS_INFO("sepanta_move Action server started, sending goal.");
    athomerobot_msgs::sepantaGoal moveGoal;

    moveGoal.type = cmd_move;
    moveGoal.value = value;
    move_action.sendGoal(moveGoal);
    //wait for the action to return
    ROS_INFO("Waiting for sepanta_move to be dONE!");
    bool finished_before_timeout = move_action.waitForResult(ros::Duration(MOVE_TIME));
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = move_action.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("sepanta_move Action finished\n");
        } else {
            ROS_INFO("Finished sepanta_move with current State: %s\n",
                     move_action.getState().toString().c_str());
        }

    } else {
        ROS_INFO("sepanta_move did not finish before the time out.");

        move_action.cancelGoal();
        move_action.stopTrackingGoal();
    }

}
position convertKinectPosition(position pos) {
    float teta = -0.00511 * (head_tilt_gl - 512);

    position world_position;
    world_position.x = pos.x;
    world_position.z =  cos(0.81) * (pos.z+.04);
    world_position.y = cos(-teta) * pos.y - sin(-teta) * pos.z;

    return world_position;
}

void check_constraints(position pos) {

    int odom_value;
    string direction;

    if (pos.y  > 0.03 ) { //???????????
        cout << "move left :" << (pos.y - 0.02) * 100 << endl;
        omni_y = (pos.y - 0.02);
        odom_value = (int)((pos.y - 0.02) * 100);
        direction = "movey";
        objecty = odom_value;
        sepanta_move(direction, odom_value);

    }

    else if (pos.y  < -0.08 ) { ///??????????
        //move right
        cout << "move right :" << (pos.y + 0.07) * 100 << endl;
        omni_y = (pos.y + 0.07);
        odom_value = (int)((pos.y + 0.07) * 100);
        direction = "movey";
        objecty = odom_value;
        sepanta_move(direction, odom_value);

    }

    ///------------- check kardane x---------------
    if (pos.x > ((L2 + L1)*sin(M_PI_2 / 2))) {

        cout << "move forward :" << (int)((pos.x - (((L2 + L1)*sin(M_PI_2 / 2)))) * 100) << endl;
        // head_tilt_gl = head_tilt_gl - floor((int)((pos.x - (((L2 + L1) * sin(M_PI_2 / 2)))) * 100) + 10);
        // my_head_msg.pan = 0;
        // my_head_msg.tilt = head_tilt_gl;
        // head.publish(my_head_msg);

        odom_value = (int)((pos.x - (((L2 + L1) * sin(M_PI_2 / 2)))) * 100);
        omni_x = (pos.x - (((L2 + L1) * sin(M_PI_2 / 2))));
        direction = "movex";
        sepanta_move(direction, odom_value);
        objectZ = odom_value;

    }
} //end constraints function

position fun(position pos) {
    position Inv_pos;
    Inv_pos.x = pos.x - omni_x + 0.02; // length griper
    Inv_pos.y = pos.y - omni_y;
    Inv_pos.z = pos.z ;
    return Inv_pos;
}

ArmMsg inverse_kinematic(position pos) {

    float Sh_roll = atan2(pos.y, pos.z);
    cout << "Sh_roll  :" << Sh_roll << endl;
    float Wrist_roll = -1.0 * Sh_roll;
    cout << "Wrist_roll  :" << Wrist_roll << endl;
    float obj_posZ2 = sqrt(pow(pos.y, 2) + pow(pos.z, 2));

    float d = sqrt(pow(pos.x, 2) + pow(obj_posZ2, 2));


    float beta1 = acos((pow(L1, 2) + pow(d, 2) - pow(L2, 2)) / (2 * L1 * d));
    float beta2 = acos((pow(L2, 2) + pow(d, 2) - pow(L1, 2)) / (2 * L2 * d));

    float alpha = atan2(pos.x, obj_posZ2);
    float Sh_pitch = alpha - beta1;
    cout << "Sh_pitch  :" << Sh_pitch << endl;
    float elbow = (beta1 + beta2);
    cout << "elbow  :" << elbow << endl;
    float Wrist_pitch = M_PI_2 - alpha - beta2;
    cout << "Wrist_pitch  :" << Wrist_pitch << endl;
    ArmMsg my_arm_msg;

    int teta1 = (int)(-1136.36 * Sh_pitch + 3000);
    int teta2 = (int)(-654.44 * Sh_roll + 2052);
    int teta3 = (int)(-651.89 * elbow + 2048);
    int teta4 = (int)(-651.89 * Wrist_pitch + 2048 - 100);
    int teta5 = (int)(-651.89 * Wrist_roll + 2048);

    my_arm_msg.shoulder_pitch = teta1;
    my_arm_msg.shoulder_roll = teta2;
    my_arm_msg.elbow = teta3;
    my_arm_msg.wrist_pitch = teta4; ////??????????????????????
    my_arm_msg.wrist_roll = teta5;

    return my_arm_msg;

}

position convertKinect2Inverse(position pos) {
    position Inv_position;
    Inv_position.x = pos.z - L3 + 0.03+0.26; // length griper
    Inv_position.y = kinect2arm_y - pos.x;
    Inv_position.z=h_arm -h_table - (2*h_obj/3)- 0.11;
    // Inv_position.z = pos.y - kinect2arm_z - 0.07;
    return Inv_position;
}


void executeCB_grip(const  athomerobot_msgs::gripGoalConstPtr &goal, grip_Server *as_grip ) {

    std::cout << "grip_Server action started... :)\n We're in CallBack now\n" << std::endl;

    // create messages that are used to published feedback/result

    if (goal->input == "Grip_Start") {
        state = 1;
    }
    // // start executing the action
    athomerobot_msgs::gripResult grip_result;

    position obj_result;
    position kinect_pos;
    position robot_pos;
    position robot_pos2;
    string command;
    ArmMsg arm_msg;
    ArmMsg arm_msg_pick;
    head_msg my_head_msg;
    int_msg gripper_msg;

    int table_height;
    while (globalRuningFlag_grip) {
        if (state == 1) {

            my_head_msg.pan = 512;
            my_head_msg.tilt = head_tilt_gl;
            head.publish(my_head_msg);
            z_msg.data = 330;
            desired_z.publish(z_msg);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
               gripper_msg.data = -1;
            gripper_right.publish(gripper_msg);
            arm_msg = arm_ready_right();
            arm_right.publish(arm_msg);
            boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

            command = "Object_Start";
            obj_result = object_function(command);
            kinect_pos = convertKinectPosition(obj_result);
            cout << "xk......: " << kinect_pos.x << endl;
            cout << "yk......: " << kinect_pos.y << endl;
            cout << "zk......: " << kinect_pos.z << endl;
            robot_pos = convertKinect2Inverse(kinect_pos);
            cout << "x......: " << robot_pos.x << endl;
            cout << "y......: " << robot_pos.y << endl;
            cout << "z......: " << robot_pos.z << endl;
            state = 2;

        } else if (state == 2) {

            check_constraints(robot_pos);
            gripper_msg.data = -1;
            gripper_right.publish(gripper_msg);
            command = "Pose_Request";
            cout << "Pose_Request" << endl;
            // obj_result = object_function(command);
            // kinect_pos = convertKinectPosition(obj_result);
            // cout << "xk......: " << kinect_pos.x << endl;
            // cout << "yk......: " << kinect_pos.y << endl;
            // cout << "zk......: " << kinect_pos.z << endl;
            // robot_pos = convertKinect2Inverse(kinect_pos);
            // cout << "x......: " << robot_pos.x << endl;
            // cout << "y......: " << robot_pos.y << endl;
            // cout << "z......: " << robot_pos.z << endl;
            robot_pos2 = fun(robot_pos);
            arm_msg = inverse_kinematic(robot_pos2);
            arm_right.publish(arm_msg);
            boost::this_thread::sleep(boost::posix_time::milliseconds(4000));

            gripper_msg.data = 10;
            gripper_right.publish(gripper_msg);
            boost::this_thread::sleep(boost::posix_time::milliseconds(6000));

            arm_msg_pick = arm_pick(arm_msg);
            arm_right.publish(arm_msg_pick);
            boost::this_thread::sleep(boost::posix_time::milliseconds(6000));

            sepanta_move("movex",-objectZ);
            sepanta_move("movey",-objecty);
            break;
        } 

        boost::this_thread::sleep(boost::posix_time::milliseconds(100));

    } /// end while

    grip_result.result = "DONE";
    cout << "DONE" << endl;
    as_grip->setSucceeded(grip_result);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));

}
int main(int argc, char **argv) {
    ros::init(argc, argv, "grip_reg_action_server");
    ros::NodeHandle nodeHandle;
    ros::NodeHandle n_Publish;
    ros::NodeHandle motor_listen;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    arm_right = n_Publish.advertise<ArmMsg>("AUTROBOTIN_arm_right", 10);
    gripper_right = n_Publish.advertise<int_msg>("AUTROBOTIN_gripper_right", 10);
    head = n_Publish.advertise<head_msg>("AUTROBOTIN_head", 10);
    desired_z = n_Publish.advertise<int_msg>("AUTROBOTIN_desirez", 10);
    grip_Server actionServer_grip(nodeHandle, "Grip_Reg_action", boost::bind(&executeCB_grip, _1, &actionServer_grip), false);
    globalServer_grip = &actionServer_grip;
    actionServer_grip.start();

    boost::thread Preempt_thread_grip(&PreemptThreadGrip);
    ros::spin();
    Preempt_thread_grip.join();
    return 0;
}
