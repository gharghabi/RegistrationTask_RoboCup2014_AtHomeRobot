class BallDetection{
private:
    int i;
public:
    void mainBallDetection()
    {
        CvCapture* camera =cvCreateCameraCapture(-1); // Use the default camera

        IplImage* frame = 0;
        CvMemStorage* storage = cvCreateMemStorage(0); //needed for Hough circles

        // capturing some extra frames seems to help stability
        frame = cvQueryFrame(camera);
        frame = cvQueryFrame(camera);
        frame = cvQueryFrame(camera);

        // with default driver, PSEye is 640 x 480
        CvSize size = cvSize(640,480);
        IplImage *  hsv_frame    = cvCreateImage(size, IPL_DEPTH_8U, 3);
        IplImage*  thresholded    = cvCreateImage(size, IPL_DEPTH_8U, 1);
        IplImage*  thresholded2    = cvCreateImage(size, IPL_DEPTH_8U, 1);

        CvScalar hsv_min = cvScalar(0, 50, 170, 0);
        CvScalar hsv_max = cvScalar(10, 180, 256, 0);
        CvScalar hsv_min2 = cvScalar(170, 50, 170, 0);
        CvScalar hsv_max2 = cvScalar(256, 180, 256, 0);

        //do {
            frame = cvQueryFrame(camera);
            if (frame != NULL) {
                printf("got frame\n\r");
                // color detection using HSV
                cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
                // to handle color wrap-around, two halves are detected and combined
                cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
                cvInRangeS(hsv_frame, hsv_min2, hsv_max2, thresholded2);
                cvOr(thresholded, thresholded2, thresholded);

                cvSaveImage("thresholded.jpg",thresholded);

                // hough detector works better with some smoothing of the image
                cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 9, 9 );
                CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height/4, 100, 40, 20, 200);

                        for (int i = 0; i < circles->total; i++)
                        {
                            float* p = (float*)cvGetSeqElem( circles, i );
                            printf("Ball! x=%f y=%f r=%f\n\r",p[0],p[1],p[2] );
                                cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),
                                                 3, CV_RGB(0,255,0), -1, 8, 0 );
                                cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),
                                                 cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0 );
                         }

                cvSaveImage("frame.jpg", frame);
            } else {
                printf("Null frame\n\r");
            }
      //} while (true);
      cvReleaseCapture(&camera);

    }
};


class BallTrack{
private:
    int i;
public:

 void mainBallTrack()
 {
    VideoCapture cap(0); //capture the video from webcam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
     //  return -1;
    }

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  int iLowH = 170;
 int iHighH = 179;

  int iLowS = 150;
 int iHighS = 255;

  int iLowV = 60;
 int iHighV = 255;

  //Create trackbars in "Control" window
 createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 179);

  createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

  createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);

  int iLastX = -1;
 int iLastY = -1;

  //Capture a temporary image from the camera
 Mat imgTmp;
 cap.read(imgTmp);

  //Create a black image with the size as the camera output
 Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;


    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

    Mat imgHSV;

   cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

  Mat imgThresholded;

   inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

  //morphological opening (removes small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

   //morphological closing (removes small holes from the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

   //Calculate the moments of the thresholded image
  Moments oMoments = moments(imgThresholded);

   double dM01 = oMoments.m01;
   double dM10 = oMoments.m10;
   double dArea = oMoments.m00;

   // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
  if (dArea > 10000)
  {
   //calculate the position of the ball
   int posX = dM10 / dArea;
   int posY = dM01 / dArea;

   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
   {
    //Draw a red line from the previous point to the current point
    line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
   }

    iLastX = posX;
   iLastY = posY;
  }

   imshow("Thresholded Image", imgThresholded); //show the thresholded image

   imgOriginal = imgOriginal + imgLines;
  imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break;
       }
    }
  }
};


class DetectBall{
private:
    int H_MIN;
    int H_MAX;
    int S_MIN;
    int S_MAX;
    int V_MIN;
    int V_MAX;
    //default capture width and height
     int FRAME_WIDTH;
     int FRAME_HEIGHT;
    //max number of objects to be detected in frame
     int MAX_NUM_OBJECTS;
    //minimum and maximum object area
     int MIN_OBJECT_AREA;
     int MAX_OBJECT_AREA;
    //names that will appear at the top of each window
     string windowName;
     string windowName1;
     string windowName2;
     string windowName3;
     string trackbarWindowName;
public:
    DetectBall(){
        H_MIN = 0;
        H_MAX = 256;
        S_MIN = 0;
        S_MAX = 256;
        V_MIN = 0;
        V_MAX = 256;
        //default capture width and height
        FRAME_WIDTH = 640;
        FRAME_HEIGHT = 480;
        //max number of objects to be detected in frame
        MAX_NUM_OBJECTS=50;
        //minimum and maximum object area
        MIN_OBJECT_AREA = 20*20;
        MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
        //names that will appear at the top of each window
        windowName = "Original Image";
        windowName1 = "HSV Image";
        windowName2 = "Thresholded Image";
        windowName3 = "After Morphological Operations";
        trackbarWindowName = "Trackbars";
    }

    void on_trackbar( int, void* )
    {//This function gets called whenever a
        // trackbar position is changed





    }
    string intToString(int number){


        std::stringstream ss;
        ss << number;
        return ss.str();
    }
    void createTrackbars(){
        //create window for trackbars


        namedWindow(trackbarWindowName,0);
        //create memory to store trackbar name on window
        char TrackbarName[50];
        sprintf( TrackbarName, "H_MIN", H_MIN);
        sprintf( TrackbarName, "H_MAX", H_MAX);
        sprintf( TrackbarName, "S_MIN", S_MIN);
        sprintf( TrackbarName, "S_MAX", S_MAX);
        sprintf( TrackbarName, "V_MIN", V_MIN);
        sprintf( TrackbarName, "V_MAX", V_MAX);
        //create trackbars and insert them into window
        //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
        //the max value the trackbar can move (eg. H_HIGH),
        //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
        //                                  ---->    ---->     ---->
        //        createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
        //        createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
        //        createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
        //        createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
        //        createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
        //        createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );


    }
    void drawObject(int x, int y,Mat &frame){

        //use some of the openCV drawing functions to draw crosshairs
        //on your tracked image!

        //UPDATE:JUNE 18TH, 2013
        //added 'if' and 'else' statements to prevent
        //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

        circle(frame,Point(x,y),20,Scalar(0,255,0),2);
        if(y-25>0)
        line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
        else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
        if(y+25<FRAME_HEIGHT)
        line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
        else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
        if(x-25>0)
        line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
        else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
        if(x+25<FRAME_WIDTH)
        line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
        else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

        putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

    }
    void morphOps(Mat &thresh){

        //create structuring element that will be used to "dilate" and "erode" image.
        //the element chosen here is a 3px by 3px rectangle

        Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
        //dilate with larger element so make sure object is nicely visible
        Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

        erode(thresh,thresh,erodeElement);
        erode(thresh,thresh,erodeElement);


        dilate(thresh,thresh,dilateElement);
        dilate(thresh,thresh,dilateElement);



    }
    void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

        Mat temp;
        threshold.copyTo(temp);
        //these two vectors needed for output of findContours
        vector< vector<Point> > contours;
        vector<Vec4i> hierarchy;
        //find contours of filtered image using openCV findContours function
        findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
        //use moments method to find our filtered object
        double refArea = 0;
        bool objectFound = false;
        if (hierarchy.size() > 0) {
            int numObjects = hierarchy.size();
            //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
            if(numObjects<MAX_NUM_OBJECTS){
                for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                    Moments moment = moments((cv::Mat)contours[index]);
                    double area = moment.m00;

                    //if the area is less than 20 px by 20px then it is probably just noise
                    //if the area is the same as the 3/2 of the image size, probably just a bad filter
                    //we only want the object with the largest area so we safe a reference area each
                    //iteration and compare it to the area in the next iteration.
                    if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
                        x = moment.m10/area;
                        y = moment.m01/area;
                        objectFound = true;
                        refArea = area;
                    }else objectFound = false;

MAXITEMS
                }
                //let user know you found an object
                if(objectFound ==true){
                    putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
                    //draw object location on screen
                    drawObject(x,y,cameraFeed);}

            }else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
        }
    }
    void mainDetector()
    {
        //some boolean variables for different functionality within this
        //program
        bool trackObjects = false;
        bool useMorphOps = false;
        //Matrix to store each frame of the webcam feed
        Mat cameraFeed;
        //matrix storage for HSV image
        Mat HSV;
        //matrix storage for binary threshold image
        Mat threshold;
        //x and y values for the location of the object
        int x=0, y=0;
        //create slider bars for HSV filtering
        createTrackbars();
        //video capture object to acquire webcam feed
        VideoCapture capture;
        //open capture object at location zero (default location for webcam)
        capture.open(0);
        //set height and width of capture frame
        capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
        //start an infinite loop where webcam feed is copied to cameraFeed matrix
        //all of our operations will be performed within this loop
        while(1){
            //store image to matrix
            capture.read(cameraFeed);
            //convert frame from BGR to HSV colorspace
            cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
            //filter HSV image between values and store filtered image to
            //threshold matrix
            inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
            //perform morphological operations on thresholded image to eliminate noise
            //and emphasize the filtered object(s)
            if(useMorphOps)
            morphOps(threshold);
            //pass in thresholded frame to our object tracking function
            //this function will return the x and y coordinates of the
            //filtered object
            if(trackObjects)
                trackFilteredObject(x,y,threshold,cameraFeed);

            //show frames
            imshow(windowName2,threshold);
            imshow(windowName,cameraFeed);
            imshow(windowName1,HSV);


            //delay 30ms so that screen can refresh.
            //image will not appear without this waitKey() command
            waitKey(30);
        }
    }
};

