/*
----------------------------------------------------------------------------------------------------------------------------
                                                     Hand Gesture Detection

											Done By
												Mohafiz Raz.M.A
 												Febin Jose
												C.Sanjay Arvind
----------------------------------------------------------------------------------------------------------------------------
*/

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>


// Create a string that contains the exact cascade name
// Contains the trained classifer for detecting hand
const char *cascade_name="hand.xml";

//The function detects the hand from input frame and draws a rectangle around the detected portion of the frame
void detect_and_draw( IplImage* img )
{

    // Create memory for calculations
    static CvMemStorage* storage = 0;

    // Create a new Haar classifier
    static CvHaarClassifierCascade* cascade = 0;

    // Sets the scale with which the rectangle is drawn with
    int scale = 1;

    // Create two points to represent the hand locations
    CvPoint pt1, pt2;

    // Looping variable
    int i; 

    // Load the HaarClassifierCascade
    cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name, 0, 0, 0 );
    
    // Check whether the cascade has loaded successfully. Else report and error and quit
    if( !cascade )
    {
        fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
        return;
    }
    
    // Allocate the memory storage
    storage = cvCreateMemStorage(0);

    // Create a new named window with title: result
    cvNamedWindow( "result", 1 );

    // Clear the memory storage which was used before
    cvClearMemStorage( storage );

    // Find whether the cascade is loaded, to find the hands. If yes, then:
    if( cascade )
    {

        // There can be more than one hand in an image. So create a growable sequence of hands.
        // Detect the objects and store them in the sequence
        CvSeq* hands = cvHaarDetectObjects( img, cascade, storage,
                                            1.1, 2, CV_HAAR_DO_CANNY_PRUNING,
                                            cvSize(40, 40) );

        // Loop the number of hands found.
        for( i = 0; i < (hands ? hands->total : 0); i++ )
        {
           // Create a new rectangle for drawing the hand
            CvRect* r = (CvRect*)cvGetSeqElem( hands, i );

            // Find the dimensions of the hand,and scale it if necessary
            pt1.x = r->x*scale;
            pt2.x = (r->x+r->width)*scale;
            pt1.y = r->y*scale;
            pt2.y = (r->y+r->height)*scale;

            // Draw the rectangle in the input image
            cvRectangle( img, pt1, pt2, CV_RGB(230,20,232), 3, 8, 0 );
        }
    }

    // Show the image in the window named "result"
    cvShowImage( "result", img );

   
}


// A Simple Camera Capture Framework
int main()
{

  // Gets the input video stream from camera
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  
  // Checks if the input stream is obtained
  if( !capture ) 
  {
    fprintf( stderr, "ERROR: capture is NULL \n" );
    getchar();
    return -1;
  }

  // Show the image captured from the camera in the window and repeat
  while( 1 )
  {

    // Get one frame
    IplImage* frame = cvQueryFrame( capture );
    
    // Cecks if a frame is obtained
    if( !frame )
    {
      fprintf( stderr, "ERROR: frame is null...\n" );
      getchar();
      break;
    }

    // Flips the frame into mirror image 
    cvFlip(frame,frame,1);

    // Call the function to detect and draw the hand positions
    detect_and_draw(frame);

    //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
    //remove higher bits using AND operator
    if( (cvWaitKey(10) & 255) == 27 ) 
      break;
  }

  // Release the capture device housekeeping
  cvReleaseCapture( &capture );

  return 0;
}
