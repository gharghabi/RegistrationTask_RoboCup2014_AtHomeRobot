
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
#include <athomerobot_msgs/sepantaAction.h>
#include <athomerobot_msgs/slamactionAction.h>
#include "KinectControl.h"
#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <math.h>
#include <sstream>
#include <boost/thread.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tbb/atomic.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <athomerobot_msgs/gripAction.h>
#include <athomerobot_msgs/objectAction.h>

#include <ros/package.h>
#include <boost/thread.hpp>
#include <athomerobot_msgs/sepantaAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>
#include <pcl/common/time.h>

//typedef actionlib::SimpleActionServer<registration::objectAction> regtogrip_Server;
typedef actionlib::SimpleActionClient<athomerobot_msgs::gripAction> object_Client;
#define R_TIME 110
using namespace std;
using namespace cv;

typedef athomerobot_msgs::arm arm_msg_right;
typedef athomerobot_msgs::arm arm_msg_left;
typedef std_msgs::Int32 int_msg;
typedef athomerobot_msgs::omnidata drive_msg;
typedef athomerobot_msgs::head head_msg;
int_msg gripper_msg;
int_msg msg_z;
tbb::atomic<int> Compass;
typedef actionlib::SimpleActionClient<athomerobot_msgs::slamactionAction> SLAMClient;
typedef actionlib::SimpleActionClient<athomerobot_msgs::sepantaAction> RClient;

typedef actionlib::SimpleActionServer<athomerobot_msgs::objectAction> object_Server;
RClient *globalR;
SLAMClient *globalSLAM;

ros::Publisher Text_pub;
ros::Publisher arm_left;
ros::Publisher arm_right;
ros::Publisher desired_z;
ros::Publisher gripper_left;
ros::Publisher gripper_right;
ros::Publisher omni_drive;
ros::Publisher odometry_x;
ros::Publisher odometry_y;
ros::Publisher n_head;
ros::Publisher headDaraje;
ros::Publisher twistPublisher;
ros::Publisher gototablePublisher;
arm_msg_left my_arm_msg_left;
arm_msg_right my_arm_msg_right;
head_msg my_head_msg;
object_Server *globalServer_object;

ifstream IntroductionFile;// ("PartOneFile.txt");
ifstream GripFile;
ifstream DanceFile;
std_msgs::String TextForSpeech;

//TODO  meghdar dehi avalie

float shoulderPitch;
float shoulderRoll;
float elbow;
float wristPitch;
float wristRoll;
float Objectz =0;

float gripR = -1;
// float shoulderL = 512;
// float elbowL = 512;
// float wristL = 512;
// float handL = 512;
// float gripL = 512;
float timePub;
float panhead = 512;
float tilthead = 1500;

float height = 1.5;
float width = 1.5;
float centerX = 1;
float centerY = 1;
float xGlobal = 1;
float yGlobal = 1;

static double first = 0;
static double last = 0;
static double totaltime = 0;
int BloobFExplodeNum = 3;
int number_ball = 2;
//regtogrip_Server *globalServer_regtogrip;
object_Client *globalClient_object;

struct position
{
    float x;
    float y;
    float z;
};
int argcGlobal;
char **argvGlobal;
position currentPosition()
{
    position loca;
    loca.x = xGlobal;
    loca.z = yGlobal;
    return loca;
}

Mat imageG;
pcl::PointCloud<pcl::PointXYZRGB> globalcloud;
bool cloudcome = false;
std::vector<cv::Rect> faces;
void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg) ;
void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) ;
int RegisterationIntroduction() ;
position DetectColor(string color);
position calcPosition (std::vector<cv::Point> contour);
bool FaceDetect(int, int);
int findMaximumFace();
position DetectFaceForBlob();
void Move(string type, float v);
void Mainlogic();
void bloob_and_flags();
void goWithSlam(string where);
void initPosition();
void gripaction_function();
void sendToSpeech(string s);
void setHand_GrapPosition();
void chatterCallback_compass(const std_msgs::Int32::ConstPtr &msg);
void DropBall();
arm_msg_right arm_move_right(float shoulder_pitch, float shoulder_roll, float elbow, float wrist_pitch, float wrist_roll);
void Omnidrive(int x, int y, int w);
void resetRobot();
void executeCB_object(const  athomerobot_msgs::objectGoalConstPtr &goal, object_Server *as_object );
void explode_bloob();
string whichColorDetected();
struct  Coordinate
{
    float x, y, z;
};

struct grip
{
    float x;
    float y;
    float z;
};

void moveToXY (float x, float y)
{
    cout<<"move x "<<x<<" move y "<<y<<endl;
    Move("movey", y);
    Move("movex", x);
}

int bloobstate = 0;
string color = "blue";
void bloob_and_flags()
{
    totaltime=0;
    int x, y, z;
    resetRobot();
    //?    goWithSlam("registration_source");

    gripaction_function();
    ROS_INFO("grip start");
    my_head_msg.pan = 512;
    my_head_msg.tilt = 1500; //0 ta 90
    n_head.publish(my_head_msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    // msg_z.data = 420 ;//beyne 2000 ta 3000 vali 2000 balast
    // desired_z.publish(msg_z);
    // boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

    Move("movey", 0.6);
    Move("movex",  0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    //+_+_+_+_+_+_+_detect face _+_+_+_+_+_+_+_+//

    int  startdegree = Compass;
    ROS_INFO("compass");

    // first = pcl::getTime();
    // boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    // last = pcl::getTime();
    // cout<<last-first<<"  ############# "<<endl;

    first = pcl::getTime();//*
    cout<<first<<" first"<<endl;
    //std::cout << i << " i " << std::endl;
    for (int i = 0; i < number_ball; i++)
    {
        position ps =  DetectFaceForBlob();
        cout << "Position ps.x=:" << ps.x << " ps.z=: " << ps.z;
       Omnidrive(0, 0, -100);
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
     
        while ((ps.x == 0 || (ps.x < 230 || ps.x > 550)) && ros::ok())
        {
              // cout<<ps.y<<" ps.y "<<endl;
              //        cout<<ps.x<<" ps.x "<<endl;
            //ROS_INFO("face detect");
            ps =  DetectFaceForBlob();
           //*
        }
        // cout<<ps.y<<" ps.y "<<endl;
        //         cout<<ps.x<<" ps.x "<<endl;

       // std::cout << i << " i " << std::endl;
        
        Omnidrive(0, 0, 0);
        sendToSpeech("welcome");
        ROS_INFO("welcome");
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
    }

    last = pcl::getTime();//*
    cout<<last-first<<" last - first "<<endl;
 
    totaltime = last - first- number_ball*5;
    cout<<last<<" last"<<endl;
    Omnidrive(0,0,100);
    cout<<totaltime<<" totalend"<<endl;
    first = pcl::getTime();
    while((pcl::getTime()-first)<totaltime-3)
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }

   // Move("turngl",startdegree);
    Move("movex",  0);
    Move("movey", -0.6);
   // Move("movex", -Objectz);
    
    gripper_msg.data = -1;
    gripper_right.publish(gripper_msg);
    resetRobot();
}

void twitteSystem(string s1,string s2)
{
    for (int j=0;j<5;j++){
    ostringstream temp;
    temp<<j;
    string url="http://www.autathome.tk/png/kinect"+temp.str()+".png";
    string path = ros::package::getPath("registration") + "/src/bash/twitter_.sh "+ s1+" "+s2+" "+url;
    system(path.c_str());
    }
}

void sendFileMain()
{
    string path = ros::package::getPath("registration") + "/src/bash/sendFiles.sh";
    system(path.c_str());
 }
void sendFileSSH()
{
    string path = ros::package::getPath("registration") + "/src/bash/sendSSH.sh";
    system(path.c_str());
}

void twitte()
{
for(int k=0;k<1;k++){
                ostringstream temp;
                temp<<k;
                sendToSpeech("get_pic");            
                std::stringstream s;
                string path = ros::package::getPath("registration") + "/src/png/kinect"+temp.str()+".png";
                s << path;
                ROS_INFO("save6i     cloud1");
                while (globalcloud.size()==0)
                {
                    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                }
                pcl::io::savePNGFile(s.str(), globalcloud);
                    boost::this_thread::sleep(boost::posix_time::seconds(3));
    
            
        }
    sendFileSSH();//ferestadan be servere aghaye khezri nyazi be vpn nadare age vpnraun koni ino bayad comment koni
   // sendFileMain();//vpn bayad vasl bashad 
    //twitteSystem("AUT@Home","");//twitteSystem("AUT@Home","#RoboCup2014");
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Reading_From_file");

    argcGlobal = argc;
    argvGlobal = argv;
    ros::NodeHandle n_arm_left;
    ros::NodeHandle n_arm_right;
    ros::NodeHandle n_omni_drive;
    ros::NodeHandle SpeechNodehandler;
    ros::NodeHandle n_;
    ros::NodeHandle desired_z_n;
    ros::NodeHandle gotodiscNHandle;
    ros::NodeHandle nodeHandle;
    //------------------
    Text_pub = SpeechNodehandler.advertise<std_msgs::String>("/AUTROBOTOUT_speak", 1);
    desired_z = desired_z_n.advertise<int_msg>("AUTROBOTIN_desirez", 10);
    gripper_left = n_.advertise<int_msg>("AUTROBOTIN_gripper_left", 10);
    gripper_right = n_.advertise<int_msg>("AUTROBOTIN_gripper_right", 10);
    omni_drive = n_omni_drive.advertise<drive_msg>("AUTROBOTIN_omnidrive", 10);
    arm_left = n_.advertise<arm_msg_left>("AUTROBOTIN_arm_left", 10);
    arm_right = n_.advertise<arm_msg_right>("AUTROBOTIN_arm_right", 10);
    n_head = n_.advertise<head_msg>("/AUTROBOTIN_head", 10);
    headDaraje = n_.advertise<head_msg>("/AUTROBOTIN_head", 10);
    twistPublisher = n_.advertise<int_msg>("AUTROBOTIN_twist", 10);
    gototablePublisher = n_.advertise<int_msg>("AUTROBOTIN_gototable", 10);
    ros::Subscriber subs = n_.subscribe("AUTROBOTOUT_compass", 10, chatterCallback_compass);
    usleep(3000000);
    /*
    Color Detection
    */
    ros::NodeHandle nodeHandleForKinect;
    image_transport::ImageTransport imageTransport(nodeHandleForKinect);
    image_transport::Subscriber imageSubscriber;
    imageSubscriber = imageTransport.subscribe( "/camera/rgb/image_color", 1, rosImageCallBack);

    ros::NodeHandle nodeHandleForPC;
    ros::Subscriber PCSubscriber = nodeHandleForPC.subscribe("/camera/depth_registered/points", 1, PointCloudCallBack);
    object_Server actionServer_object(nodeHandle, "Object_Reg_action", boost::bind(&executeCB_object, _1, &actionServer_object), false);
    globalServer_object = &actionServer_object;
    actionServer_object.start();

    // first = pcl::getTime();
    // boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    // last = pcl::getTime();
    // cout<<last-first<<endl;
    ROS_INFO("wait for server start");
    boost::thread Mainlogic_process(&Mainlogic);

    ros::spin();
    Mainlogic_process.interrupt();
    Mainlogic_process.join();
    return 0;
}
void Mainlogic()
{
    const int MAXITEMS = 5;
    string lastItem;

    /*init_robot()
    *
    * Reset State Gusture Of Robot
    */

    //init_robot();

    string inventory[MAXITEMS] = {"function5", "function1", "function4", "function2", "function3"};


    for (int i = 1; i < argcGlobal; ++i)
    {
        std::cout << argvGlobal[i] << std::endl;
        if (strcmp(argvGlobal[i], "1") == 0)
        {
            RegisterationIntroduction();

        }
        else if (strcmp(argvGlobal[i], "2") == 0)
        {
            color = "green";
            sendToSpeech("start_music");
            bloob_and_flags();
            ROS_INFO("end");
        }
        else if (strcmp(argvGlobal[i], "3") == 0)
        {
            color = "blue";
            sendToSpeech("start_music");
            bloob_and_flags();
        }
        else if (strcmp(argvGlobal[i], "4") == 0)
        {
            explode_bloob();
        }
        else if (strcmp(argvGlobal[i], "5") == 0)
        {
            DropBall();
        }
        else if (strcmp(argvGlobal[i], "6") == 0)
        {
            resetRobot();
            // Move("movey", 0.6);
            // Move("movex",  0);
            twitte();
            // Move("movey", -0.6);
            // Move("movex",  0);

        }
        std::cout << i << std::endl;
    }


    ros::shutdown();
}

position DetectColor(string color, int sizeC)
{
  //  std::cout << "detector_Color" << std::endl;
    std::vector<std::vector<cv::Point> > contours;
    position coordinate;
    coordinate.x = 0;
    coordinate.z = 0;
    RNG rng(12345);
    std::vector<cv::Vec4i> hierarchy;
    Point2f MaxRegCenter;
    Mat temimg;
    imageG.copyTo(temimg);

    if (temimg.size().width != 0)
    {
        int iLowH, iLowS, iLowV, iHighH, iHighS, iHighV;
        if (color == "red")
        {
            iLowH = 0;
            iLowS = 177;
            iLowV = 237;
            iHighH = 0;
            iHighS = 255;
            iHighV = 255;
        }
        else if (color == "blue")
        {
            //abi roshan
            // ROS_INFO("blue");
            // iLowH = 98;
            // iLowS = 50;
            // iLowV = 80;
            // iHighH = 137;
            // iHighS = 255;
            // iHighV = 255;

            //abi tire
            iLowH = 103;
            iLowS = 120;
            iLowV = 175;
            iHighH = 111;
            iHighS = 199;
            iHighV = 254;
            //??????????

            // iLowH = 98;
            // iLowS = 50;
            // iLowV = 80;
            // iHighH = 111;
            // iHighS = 255;
            // iHighV = 254;

        }
        else if (color == "yellow")
        {
            ROS_INFO("yelloqw");
            iLowH = 25;
            iLowS = 34;
            iLowV = 101;
            iHighH = 40;
            iHighS = 254;
            iHighV = 254;
        }
        else if ((color == "black"))
        {
            iLowH = 104;
            iLowS = 0;
            iLowV = 0;
            iHighH = 104;
            iHighS = 9;
            iHighV = 80;
        }
        else if ((color == "green"))
        {

        //sabze roshan
            // iLowH = 41;
            // iLowS = 52;
            // iLowV = 170;
            // iHighH = 85;
            // iHighS = 225;
            // iHighV = 224;
        //sabze tire
            iLowH = 41;
            iLowS = 41;
            iLowV = 98;
            iHighH = 102;
            iHighS = 253;
            iHighV = 224;


            // //sabze tire
            // iLowH = 41;
            // iLowS = 41;
            // iLowV = 98;
            // iHighH = 102;
            // iHighS = 253;
            // iHighV = 203;
        //sabz
            // iLowH = 41;
            // iLowS = 41;
            // iLowV = 98;
            // iHighH = 102;
            // iHighS = 253;
            // iHighV = 223;
        }

        Mat imgHSV;

        Mat imgThresholded = Mat::zeros( imageG.size(), CV_8UC3 );

        ROS_INFO("detectcolo4");
        int thresh = 100;
        int max_thresh = 255;
        cvtColor(temimg, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        ROS_INFO("slam2");



        //         cv::Vec3i pixel;
        //     Point3_<uchar>* p;
        //     int minx = 10000;
        //     int miny = 10000;
        //     int minz = 10000;
        //     int maxx = 0;
        //     int maxy = 0;
        //     int maxz = 0;
        //     for (int w = 0; w<imgHSV.size().width; ++w)
        //     {
        //         for (int h = 0; h<imgHSV.size().height;++h )
        //         {
        //             p = imgHSV.ptr<Point3_<uchar> >(h,w);
        //            // std::cout<<p->x<<" 0 "<<p->y<<" 1 "<<p->z<<" 2"<< std::endl;
        //             if ((p->x)<minx)
        //                 minx = p->x;
        //             if ((p->y)<miny)
        //                 miny = p->y;
        //             if ((p->z)<minz)
        //                 minz = p->z;
        //             if ((p->x)>maxx)
        //                 maxx = p->x;
        //             if ((p->y)>maxy)
        //                 maxy = p->y;
        //             if ((p->z)>maxz)
        //                 maxz = p->z;
        //             std::cout<<int(p->x)<<" h "<<int(p->y)<<" s "<<int(p->z)<<" v "<<std::endl;
        //         }
        //     }
        // std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<minx<<" mix "<<miny<<" miy "<<minz<<" miz "<<maxx<<" max" <<maxy<<" may "<<maxz<<" maz "<<std::endl;
        // waitKey(0);
        //     ros::shutdown();


        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
        //morphological opening (removes small objects from the foreground)


        Mat medianImg;
        medianBlur(imgThresholded, medianImg, 21);

        imshow("medianImg", medianImg);
        waitKey(10);

        Canny( medianImg, medianImg, thresh, thresh * 2, 3 );

        cv::findContours( medianImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        //  cv::drawContours(globalImage, contours, -1, Scalar::all(255), CV_FILLED);
        // ROS_INFO("570");
        // std::cout<<globalImage.size().width<<" width "<<globalImage.size().height<<" height "<<imgThresholded.size().width<<" width "<<imgThresholded.size().height<<" heith "<<std::endl;
        // Mat resultimg = globalImage & imgThresholded;
        // cvNamedWindow( "object_position", 1);
        // cv::imshow("object_position",imgThresholded);



        ////////////////////>>>>>>>>>>> accecing contours <<<<<<<<<<<<<<<<<<<<<<

        //// >>>>>>>> find centers
        /// Get the moments
        if (contours.size() != 0)
        {

            vector<Moments> mu(contours.size() );
            for ( int i = 0; i < contours.size(); i++ )
            {
                mu[i] = moments( contours[i], false );
            }

            ///  Get the mass centers:
            vector<Point2f> centers( contours.size() ); // mc -> centers
            for ( int i = 0; i < contours.size(); i++ )
            {
                centers[i] = Point2f( mu[i].m10 / mu[i].m00 , mu[i].m01 / mu[i].m00 );
            }

            /// Draw contours
            Mat drawing = Mat::zeros( imgThresholded.size(), CV_8UC3 );

            /// Calculate the area with the moments 00 and compare with the result of the OpenCV function

            const float bad_point = std::numeric_limits<float>::quiet_NaN();
            float MaxArea = -1;
            int MaxAreaIdx = -1;
            //printf("\t Info: Area and Contour Length \n");
            for ( int i = 0; i < contours.size(); i++ )
            {
                //printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
                // if (contourArea(contours[i]) > 100)
                // {
                if ( contourArea(contours[i]) > MaxArea)
                {
                    MaxArea = contourArea(contours[i]);
                    MaxAreaIdx = i;
                    // cout << "MaxArea   " << MaxArea << endl;

                    // Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255) );
                    // drawContours( imgThresholded, contours, i, color, 2, 8, hierarchy, 0, Point() );
                    // circle( imgThresholded, centers[i], 4, color, -1, 8, 0 );
                }

                //}
            }
            //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            //drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
            //circle( drawing, mc[i], 4, color, -1, 8, 0 );
                    //?
            if (MaxArea != -1)
            {
                  MaxRegCenter = centers[MaxArea];
                //?MaxRegCenter = centers[MaxArea];
            }


            if (MaxAreaIdx != -1)
            {
                if (contours[MaxAreaIdx].size() > sizeC)
                {

                    ROS_INFO("contoursize %d",sizeC);
                    // Mat drawing = Mat::zeros(imgThresholded.size(), CV_8UC3 );
                    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255) );
                    drawContours( drawing, contours, MaxAreaIdx, color, 2, 8, hierarchy, 0, Point() );
                    circle( drawing, centers[MaxAreaIdx], 4, color, -1, 8, 0 );
                    coordinate = calcPosition(contours[MaxAreaIdx]);
                    std::cout<<coordinate.x<<" coordinatex "<<coordinate.y<<" coordinatey "<<coordinate.z<<" coordinate.z "<<endl;
                }
            }
            /// Show in a window
            // cout << "fshow_countours" << endl;

            if (drawing.size().width != 0)
            {
                imshow( "Contours2", drawing );
                waitKey(10);
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            cout << "fshow_countours2" << endl;
        }
    }
    return coordinate;

}
int counterimage = 0;
void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg)
{

    cv_bridge::CvImagePtr imagePointer;
    try
    {
        imagePointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    ++counterimage;
    if (counterimage > 7)
    {
        imageG = imagePointer->image;
 
        // imshow("windowerror", imageG);
        // waitKey(3);  
          }
    //DetectColor("yellow");
    //DetectFaceForBlob();
}
void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cloudcome = true;
    pcl::fromROSMsg(*msg, globalcloud);


}
position calcPosition (std::vector<cv::Point> contours)
{
    position World_Pose;
    int cnt_z = 0;
    int cnt_y = 0;
    int cnt_x = 0;
    pcl::PointXYZ p1;

    if (cloudcome != false)
    {
        for ( int b = 0; b < contours.size();  b++ )
        {
            p1.x = globalcloud.at(contours[b].x, contours[b].y).x;
            p1.y = globalcloud.at(contours[b].x, contours[b].y).y;
            p1.z = globalcloud.at(contours[b].x, contours[b].y).z;

            if ( !isnan(p1.x))
            {
                World_Pose.x += p1.x;
                cnt_x++;
            }
            if ( !isnan(p1.y))
            {
                World_Pose.y += p1.y;
                cnt_y++;
            }
            if ( !isnan(p1.z) && (p1.z > 0))
            {
                World_Pose.z += p1.z;
                cnt_z++;
            }
        }
        if ((cnt_x != 0) && (cnt_y != 0) && (cnt_z != 0))
        {
            World_Pose.x = World_Pose.x / cnt_x;
            World_Pose.y = World_Pose.y / cnt_y;
            World_Pose.z = World_Pose.z / cnt_z;
        }
        // printf(" [%f, %f] ", World_Pose.x, World_Pose.z );
    }
    return World_Pose;
}
position DetectFaceForBlob()
{
    Mat temp;
    imageG.copyTo(temp);
    position coordinate;
    coordinate.x =  0;
    coordinate.y = 0;
    coordinate.z = 0;
    int ind = -1;
    //ROS_INFO("chi");
    if ( FaceDetect(80, 80))
    {
        ROS_INFO("face1");
        ind = findMaximumFace();
        std::cout << ind << std::endl;
        if (ind != -1)
        {
            //coordinate = ToWorldPose(faces[ind]);
            ROS_INFO("World_Pose2");
            coordinate.x = faces[ind].x;
            coordinate.y = faces[ind].y;
            Point center( faces[ind].x + faces[ind].width * 0.5, faces[ind].y + faces[ind].height * 0.5 );
            ellipse( temp, center, Size( faces[ind].width * 0.5, faces[ind].height * 0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0);
            imshow("salam", temp);
            waitKey(20);
        }
    }
    return coordinate;
}

bool FaceDetect(int a, int b)
{
    String face_cascade_name = ros::package::getPath("registration") + "/haarcascade_frontalface_default.xml";
    cv::CascadeClassifier face_cascade;
    face_cascade = cv::CascadeClassifier(face_cascade_name);
    Mat gray;
    Mat imgageglobal;
    imgageglobal = imageG;
    if (imgageglobal.size().width != 0)
    {
        cvtColor(imgageglobal, gray, CV_BGR2GRAY);
        if ( !face_cascade.load( face_cascade_name ) )
        {
            printf("--(!)Error loading\n");
        };
        face_cascade.detectMultiScale( gray, faces, 1.2, 12, 0 | CV_HAAR_SCALE_IMAGE, Size(a, b) ); //khub ba yek ghalat gray, faces, 1.2, 10, 0|CV_HAAR_SCALE_IMAGE, Size(50, 50) )
        // for ( size_t i = 0; i < faces.size(); i++ )
        // {
        //     Point center( faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5 );
        //     ellipse( camImage, center, Size( faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0);
        // }
        // imshow("salam", camImage);
        // waitKey(20);
    }
    if (faces.size() != 0)
    {
        return true;
    }
    else
    {
        return false;
    }

}
int findMaximumFace()
{
    if (faces.empty())
    {
        return -1;
    }
    int maximumIndex = 0;
    float maximum;
    maximum = faces[0].width * faces[0].height ;
    float temp;
    for (int i = 1; i < faces.size(); i++)
    {
        if ((temp = faces[i].width * faces[i].height) > maximum)
        {
            maximumIndex = i;
            maximum = temp;
        }
    }
    ROS_INFO("max index %d", maximumIndex);
    return maximumIndex;
}

void DropBall()
{

}
int RegisterationIntroduction()
{

   resetRobot();

    string path = ros::package::getPath("registration") + "/src/files/motors_info6.txt";
    IntroductionFile.open(path.c_str(), ios_base::in);
    float timeprev = 0;
    int counter = 0;
    string line;
    sendToSpeech("start_introduce");
    cout << "start speech" << endl;
    if (IntroductionFile.is_open())
    {

        for (std::string line; std::getline(IntroductionFile, line); )   //read stream line by line
        {
            if (line != "next")
            {
                std::istringstream in(line);      //make a stream for the line itself
                in >> timePub >> shoulderPitch >> shoulderRoll >> elbow >> wristRoll >> wristPitch  >> tilthead;
                if (shoulderPitch != 0)
                {
                    //ROS_INFO("start");
                    usleep((timePub - timeprev) * 1000000);
                    my_arm_msg_right = arm_move_right(2700, shoulderRoll, elbow, wristPitch, wristRoll); // bala bordane do dast va eshare be khod
                    arm_right.publish(my_arm_msg_right);
                    my_head_msg.pan = tilthead;
                    my_head_msg.tilt = 1500; //0 ta 90
                    n_head.publish(my_head_msg);
                    timeprev = timePub;
                }
            }
            else
            {
                cout << counter << "counter" << endl;
                if (counter == 0)   //+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-first 21 seconds+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
                {
                    timeprev = 0;
                    gripper_msg.data = 1;
                    gripper_right.publish(gripper_msg);
                    // msg_z.data = 2220 ;//beyne 2000 ta 3000 vali 2000 balast
                    // desired_z.publish(msg_z);

                    usleep(4000000);
                    sendToSpeech("start_state_1");
                    cout << "start speech" << endl;
                }
                timeprev = 0;
                ++counter;
            }
        }
        IntroductionFile.close();
    }
    else cout << "Unable to open file";
    return 0;

}

void sendToSpeech(string s)
{

    //My Name Is Sepanta
    // Sare Kinect Ro Boland Kone Va Roo Be Davar Ha Bashe.
    TextForSpeech.data = s;
    Text_pub.publish(TextForSpeech);
    cout << "State One Start" << endl << endl;
    usleep(1000000);
}
void setHand_GrapPosition()
{
    //TODO Set Hand Position
    arm_msg_right my_arm_msg_right;
        shoulderPitch = 3300;
        shoulderRoll = 2048;
        elbow = 800;
        wristPitch = 1900;
        wristRoll =  2048;
    my_arm_msg_right = arm_move_right(shoulderPitch, shoulderRoll, elbow, wristPitch, wristRoll); // bala bordane do dast va eshare be khod
    arm_right.publish(my_arm_msg_right);
}
void resetRobot()
{
    // msg_z.data = 400 ;//beyne 2000 ta 3000 vali 2000 balast
    // desired_z.publish(msg_z);
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    my_head_msg.pan = 512;
    my_head_msg.tilt = 1500; //0 ta 90
    n_head.publish(my_head_msg);
    gripper_msg.data = 1;
    gripper_right.publish(gripper_msg);
    shoulderPitch = 3000;
    shoulderRoll = 2048;
    elbow = 2048;
    wristPitch = 2048;
    wristRoll =  2048;
    my_arm_msg_right = arm_move_right(shoulderPitch, shoulderRoll, elbow, wristPitch, wristRoll); // bala bordane do dast va eshare be khod
    arm_right.publish(my_arm_msg_right);
    boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
}

bool checkBorderX(float x)
{
    float currentX = x + xGlobal;
    if (currentX > height)
    {
        return false;
    }
    else
    {
        return true;
    }
}
void changeGlobalX(float x)
{
    xGlobal = xGlobal + x;
}
void changeGlobalY(float y)
{
    yGlobal = yGlobal + y;
}

bool checkBordery(float y)
{
    float currentY = y + yGlobal;
    if (currentY > width)
    {
        return false;
    }
    else
    {
        return true;
    }
}
void initPosition()
{
    yGlobal = 100;
    xGlobal = 100;
}

void Move(string type, float v)
{

    ros::Rate loop_rate(20);
    RClient RAction("sepanta_action", true);
    globalR = &RAction;
    ROS_INFO("Waiting for What Did You Say action server to start.");
    /****** CAUTION!! :
     *
     * will wait for infinite time till server starts, it should be started before!!
     *
     */

    RAction.waitForServer();
    ROS_INFO("What Did You Say Action server started, sending goal.");
    athomerobot_msgs::sepantaGoal RGoal;


    //nkhCheck to see if every client needs it's own goal
    //int RInput = DEFALT_GOAL_VALUE;
    if (type == "movex")
    {
        RGoal.type = "movex";
            RGoal.value = (v * 100) ;
    
    }
    else if (type == "movey")
    {
        RGoal.type = "movey";
        RGoal.value = (v * 100);
    }
    else if (type == "tunrgl")
    {
        RGoal.type = "tunrgl";
        RGoal.value = -(v * 100);
    }
    RAction.sendGoal(RGoal);
    //wait for the action to return
    ROS_INFO("Waiting for What Did You Say to be dONE!");
    bool finished_before_timeout = RAction.waitForResult(ros::Duration(R_TIME));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = RAction.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("What Did You Say Action finished\n");
        }
        else
        {
            ROS_INFO("Finished What Did You Say with current State: %s\n",
                     RAction.getState().toString().c_str());
        }

    }
    else
    {
        ROS_INFO("What Did You Say Action did not finish before the time out.");

        RAction.cancelGoal();
        RAction.stopTrackingGoal();
    }
}

void goWithSlam(string where)
{
    //edwin's code :D
    ROS_INFO("Going %s with slam...", where.c_str());
    athomerobot_msgs::slamactionGoal interfacegoal;
    interfacegoal.x = 0;
    interfacegoal.y = 0;
    interfacegoal.yaw = 0;
    interfacegoal.ID = where;

    //globalSLAM->sendGoal(interfacegoal);
    ROS_INFO("goal sent to slam... waiting for reach there.");

    globalSLAM->sendGoal(interfacegoal);
    bool finished_before_timeout = globalSLAM->waitForResult(ros::Duration(R_TIME));
    actionlib::SimpleClientGoalState state = globalSLAM->getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Slam finished\n");
    }
    else
    {
        ROS_INFO("Finished Slam with current State: %s\n",
                 globalSLAM->getState().toString().c_str());
    }
}

arm_msg_right arm_move_right(float shoulder_pitch, float shoulder_roll, float elbow, float wrist_pitch, float wrist_roll)
{
    arm_msg_right my_arm_msg;
    my_arm_msg.shoulder_pitch = shoulder_pitch;
    my_arm_msg.shoulder_roll = shoulder_roll;
    my_arm_msg.elbow = elbow;
    my_arm_msg.wrist_pitch = wrist_pitch;
    my_arm_msg.wrist_roll = wrist_roll;
    return my_arm_msg;
}

arm_msg_left arm_move_left(float shoulder_pitch, float shoulder_roll, float elbow, float wrist_pitch, float wrist_roll)
{
    arm_msg_right my_arm_msg;
    my_arm_msg.shoulder_pitch = shoulder_pitch;
    my_arm_msg.shoulder_roll = shoulder_roll;
    my_arm_msg.elbow = elbow;
    my_arm_msg.wrist_pitch = wrist_pitch;
    my_arm_msg.wrist_roll = wrist_roll;
    return my_arm_msg;
}
void Omnidrive(int x, int y, int w)
{
    athomerobot_msgs    ::omnidata msg;
    msg.d0 = x;
    msg.d1 = y;
    msg.d2 = w;
    omni_drive.publish(msg);
}
void executeCB_object(const  athomerobot_msgs::objectGoalConstPtr &goal, object_Server *as_object )
{

    std::cout << "grip_Server action started... :)\n We're in CallBack now\n" << std::endl;
    // create messages that are used to published feedback/result
    ROS_INFO("object start ");
    if (  (goal->input == "Object_Start") || (goal->input == "Pose_Request") )
    {
        athomerobot_msgs::objectResult object_result;
        std::cout << goal->input << " srrr" << std::endl;
        position placeobject;
        placeobject.x = 0;
        placeobject.y = 0;
        placeobject.z = 0;
        placeobject = DetectColor(color, 20);
        ROS_INFO("bloob1");
        std::cout << placeobject.x << " object_result.x " << placeobject.y << " object_result.y " << placeobject.z << " object_result.z " << std::endl;

        while (((placeobject.z == 0) || (abs(placeobject.x) > 1) || (abs(placeobject.z) > 3)) && ros::ok())
        {
            cout << "Color Not Found: " << endl;
            placeobject = DetectColor(color, 20);
            std::cout << placeobject.x << " object_result.x " << placeobject.y << " object_result.y " << placeobject.z << " object_result.z " << std::endl;
            ROS_INFO("bloob2");
            sleep(2);
        }


        object_result.x = placeobject.x;
        object_result.y = placeobject.y;
        object_result.z = placeobject.z;
        Objectz = placeobject.z+0.08;
        cout << "DONE" << endl;
        as_object->setSucceeded(object_result);
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    // // start executing the action
}

void gripaction_function()
{

    object_Client ac_object("Grip_Reg_action", true);
    globalClient_object = &ac_object;
    ac_object.waitForServer();

    ROS_INFO("wait for server in func");
    athomerobot_msgs::gripGoal goal_grip;
    goal_grip.input = "Grip_Start";

    ac_object.sendGoal(goal_grip);
    //wait for the action to return

    bool finished_before_timeout = ac_object.waitForResult();//(ros::Duration(1));
    actionlib::SimpleClientGoalState state_ac_object = ac_object.getState();
    ROS_INFO("finished");
    if (state_ac_object == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Action_obj finished\n");
        athomerobot_msgs::gripResultConstPtr result_grip;
        result_grip = ac_object.getResult();
        // ROS_INFO(result_grip.toString().c_str());
    }
    else
    {
        ROS_INFO("Finished current State: %s\n", ac_object.getState().toString().c_str());
    }
}
void chatterCallback_compass(const std_msgs::Int32::ConstPtr &msg)
{
    Compass = msg->data;
}
void explode_bloob()
{
    for (int i = 0 ; i< BloobFExplodeNum; ++i)
    {
        //+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_reset robot+_+_+_+_+_+_+___+_
        // msg_z.data = 420 ;//beyne 2000 ta 3000 vali 2000 balast
        // desired_z.publish(msg_z);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        my_head_msg.pan = 512;
        my_head_msg.tilt = 1400; //0 ta 90
        n_head.publish(my_head_msg);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        gripper_msg.data = 1;
        gripper_right.publish(gripper_msg);
        shoulderPitch = 3100;
        shoulderRoll = 2048;
        elbow = 2048;
        wristPitch = 2048;
        wristRoll =  2048;
        my_arm_msg_right = arm_move_right(shoulderPitch, shoulderRoll, elbow, wristPitch, wristRoll); // bala bordane do dast va eshare be khod
        arm_right.publish(my_arm_msg_right);
        boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
        //+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_reset robot+_+_+_+_+_+_+___+_

        Omnidrive(0, 0, -100);
        first = pcl::getTime();
        while ((pcl::getTime() - first) < 14)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            ROS_INFO("umadam");
        }
        Omnidrive(0, 0, 0);
        ROS_INFO("wait for detect color");
        sendToSpeech("wait_for_detect_color");
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));


        string color = whichColorDetected();
        sendToSpeech(color);//speech tells i found the color i will go to explode the bloob

         ROS_INFO("color detected: [%s]",color.c_str());
        std::cout << "color" << color << " ))))))))))))))))" << std::endl;

        // msg_z.data = 390 ;//beyne 2000 ta 3000 vali 2000 balast
        // desired_z.publish(msg_z);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        //setHand_GrapPosition();
        my_head_msg.pan = 512;
        my_head_msg.tilt = 1800; //0 ta 90
        n_head.publish(my_head_msg);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        Omnidrive(0, 0, 100);
        first = pcl::getTime();
        while ((pcl::getTime() - first) < 14)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }
        Omnidrive(0, 0, 0);

        setHand_GrapPosition();

        position placeobject;
        placeobject.x = 0;
        placeobject.y = 0;
        placeobject.z = 0;
       
        placeobject = DetectColor(color, 20);

        std::cout << placeobject.x << " object_result.x " << placeobject.y << " object_result.y " << placeobject.z << " object_result.z " << std::endl;

        while (((placeobject.z == 0) || (abs(placeobject.x) > 2) || (abs(placeobject.z) > 4)) && ros::ok())
        {
            placeobject = DetectColor(color, 20);
            std::cout << placeobject.x << " object_result.x " << placeobject.y << " object_result.y " << placeobject.z << " object_result.z " << std::endl;
            sleep(2);
        }
        float x= 0.3 - placeobject.x;//0.19
        sendToSpeech("explode_bloob");
        float odom_value;
        string direction;

        if (x  > 0.02 ) { //???????????
            cout << "move left :" << (x - 0.01) * 100 << endl;
           
            odom_value = ((x-0.01 ));//odom_value = ((x-0.02 ));
            direction = "movey";
            //?Move(direction, odom_value);
            moveToXY(0, odom_value);
        }
        else if (x  < -0.02 ) { ///??????????
            //move right
            cout << "move right :" << (x + 0.01) * 100 << endl;
            
            odom_value = ((x ));//odom_value = ((x + 0.01));
            direction = "movey";
            moveToXY(0, odom_value);
        }
        if (placeobject.z-0.3 <0)
            placeobject.z=0;
        else
        placeobject.z=placeobject.z -0.28;
        moveToXY(placeobject.z,0);
        cout << "move forward :" << placeobject.z << endl;
            
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
         moveToXY(-placeobject.z,0);
         moveToXY(0,-odom_value);
         resetRobot();
    }
}
string whichColorDetected()
{
    position placeobject;
    placeobject.x = 0;
    placeobject.y = 0;
    placeobject.z = 0;

    while ((placeobject.z == 0) && (placeobject.x == 0) && (placeobject.y == 0) && (ros::ok()))
    {
        color = "yellow";
        placeobject = DetectColor(color, 80);
        if ((placeobject.z != 0) && (placeobject.x != 0) && (placeobject.y != 0) && (placeobject.z < 4))
        {
            ROS_INFO("yellow");
            return color;
        }
        color = "blue";
        placeobject = DetectColor(color, 80);
        if ((placeobject.z != 0) && (placeobject.x != 0) && (placeobject.y != 0) && (placeobject.z < 4))
        {
            ROS_INFO("blue");
            return color;
        }
        color = "green";
        placeobject = DetectColor(color, 80);
        if ((placeobject.z != 0) && (placeobject.x != 0) && (placeobject.y != 0) && (placeobject.z < 4))
        {
            ROS_INFO("green");
            return color;
        }
        placeobject.x = 0;
        placeobject.y = 0;
        placeobject.z = 0;
    }

}