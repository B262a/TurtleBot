#include <iostream>
//OpenCV specific libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//ROS specific libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace cv;
using namespace std;

geometry_msgs::Twist move(int moveType){

  geometry_msgs::Twist velocity;

  if (moveType == 3){
    velocity.linear.x= -0.1;
    velocity.linear.y= 0.0;
    velocity.linear.z= 0.0;

    velocity.angular.x= 0.0;
    velocity.angular.y= 0.0;
    velocity.angular.z= -0.1;

  }else if (moveType == 4){
    velocity.linear.x= 0.1;
    velocity.linear.y= 0.0;
    velocity.linear.z= 0.0;

    velocity.angular.x= 0.0;
    velocity.angular.y= 0.0;
    velocity.angular.z= 0.5;
  }else if (moveType == 0){
    velocity.linear.x= 0.0;
    velocity.linear.y= 0.0;
    velocity.linear.z= 0.0;

    velocity.angular.x= 0.0;
    velocity.angular.y= 0.0;
    velocity.angular.z= 0.0;
  }
  else{
    velocity.linear.x= 0.1;
    velocity.linear.y= 0.0;
    velocity.linear.z= 0.0;

    velocity.angular.x= 0.0;
    velocity.angular.y= 0.0;
    velocity.angular.z= -0.1;    
    }
    
return velocity;
}


int main(int argc, char* argv[])
{
    //Initiatialize the main ROS connection
    ros::init(argc, argv, "Publisher");

    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);

    ros::Rate loop_rate(10);

    //Initialize the variable "cap" holding the input from the thermal camera
    VideoCapture cap;

    cap.open ("http://root:axis@10.42.0.69/axis-cgi/mjpg/video.cgi?resolution=640x480&req_fps=30&.mjpg");

    if (!cap.isOpened())//If connection is unsuccessful, close the program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //Get the frame width the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //Get the frame height of the video

    //Prints the frame size at x, y coordinates
    //cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    //Create two empty windows named "ThermalLiveFeed" and "ThresholdLiveFeed"
    namedWindow("ThermalLiveFeed",CV_WINDOW_AUTOSIZE); 
    namedWindow("ThresholdLiveFeed",CV_WINDOW_AUTOSIZE);


    while (1)
    {
        //Create an OpenCV specific array
        Mat frame;

        //Define the minimum and maximum thresholding values
        double thresh = 76;
        double maxVal = 255;

        //Boolean variable which reports whether it succeded or not to read a frame from the video
        bool bSuccess = cap.read(frame); 

        if (!bSuccess) //if unsuccessful, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

        //Create two more OpenCV arrays
        Mat grey;
        Mat dst;

        //Create the cvtColor variable which converts the frame from the BGB color format to Grey and assigns it to the  variable "grey"
        cv::cvtColor(frame, grey, CV_BGR2GRAY);

        //Displays the raw thermal input in the empty "ThermalLiveFeed window"
        imshow("ThermalLiveFeed", frame);

        /*The threshold() commands converts the grey image to a binary value image named "dst
        by using the thresh and maxVal variables and the THRESH_BINARY method*/

        threshold(grey, dst, thresh, maxVal, THRESH_BINARY);

        //Displays the raw thermal input in the empty "ThresholdLiveFeed window"
        imshow("ThresholdLiveFeed", dst);
        
        //Create the contours vector, holding the contour points of all detected shapes within the threshold
        std::vector<std::vector<cv::Point> > contours;

        /*Create an empty imageclone array which holds a duplicate of the binary image dst 
        and find the shape contours within it, using the finContours() command*/
        Mat imageClone;
        imageClone = dst.clone();
        findContours(imageClone, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

        //Create an empty array with the same same as the binary image (dst) with a black background
        Mat drawImg(dst.size(), CV_8UC3, Scalar(0,0,0));

        //Starts a loop for each detected shape
        for(size_t idx = 0; idx < contours.size(); idx++)
        {   //Defines the area of the detected shape
            int area = contourArea(contours[idx]);
            //cout << idx << " area:" << area << endl; - OPtionally displays the area of the contour

            //The detection will be very innacurate if the area is below 500
            if(area > 500)
            {
                //Fill the drawImg array with contour values, using the drawContours() method
                drawContours(drawImg, contours, idx, Scalar(0,0,255));

                //Detect the contour center, print it's coordinates and draw a green circle at that position
                Moments mu = moments(contours[idx], false);
                Point2f mc = Point2f(mu.m10/mu.m00, mu.m01/mu.m00);
                //cout << "Coordinates: " << mc.x << ", " << mc.y << endl;
                circle(drawImg, mc, 5,Scalar(0,255,0));

                //Create a new empty windows and display the contour on it
                namedWindow("findContours", WINDOW_AUTOSIZE);

                imshow("findContours", drawImg);

                /*The image window has a 640x480 size. If the center of the shape(the green circle)
                  strays too far out of the windows centre, in this case, leaving the central 440x280 coordinates
                  a command will be sent to move the turtlebot towards the direction of the circle*/ 
                if(100 > mc.x)
                {
                    cout << "Target is hitting the border on the left x-axis: " << mc.x << endl;

                    pub.publish(move(1)); // Left x-axis is "drive right" angular and linear
                }

                else if (mc.x > 540)
                {
                    cout << "Target is hitting the border on the right x-axis: " << mc.x << endl;

                    pub.publish(move(2)); // Right x-axis is "drive left" angular and linear
                }

                else if ( 100 > mc.y)
                {
                    cout << "Target is hitting the border on the upper y-axis: " << mc.y << endl;

                    pub.publish(move(3)); // Upper y-axis is "drive backwards" MinusLinear
                }

                else if (mc.y > 380)
                {
                    cout << "Target is hitting the border on the lower y-axis: " << mc.y << endl;

                    pub.publish(move(4)); // Lower y-axis is "drive forwards" Linear
                }
		        else 
		        {
		            pub.publish(move(0));
		        }
            }
        }

        //Wait for 'ESC' key press for 30ms. If 'ESC' key is pressed, exit the loop
        if (waitKey(30) == 27)
        {
            cout << "ESC key is pressed by user, exiting" << endl;
            break;
        }
      ros::spinOnce();

      loop_rate.sleep();
    }

  return 0;
}
