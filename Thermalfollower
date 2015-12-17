//Including all the required C++, OpenCV and ROS libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

using namespace cv;
using namespace std;

//Define an integer variable to count how many times the while loop has been completed
int counter;

//Function which takes input from the loop in the main function
void publish(int moveType){

// Initiatialize the publisher node
ros::NodeHandle n; 

//Publish data to the following topic
ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); 

// The rate at which the publisher sends messages to the topic above(30Hz)
ros::Rate loop_rate(30); 

//Define the type of the topic which is being published, assigned to the variable input_vel
geometry_msgs::Twist input_vel; 

/*If the counter equals 90, it means that the while loop ran 3 times, equivalent to 3 seconds.
It is used as means to give the turtlebot time to execute a command, before passing the next one*/
if(counter == 90){
    
    //Depending on the input received from the main function, publish to the turtlebot the direction it should be moving
    if (moveType == 1){
        
        //Linear velocity measure in meters/second
        input_vel.linear.x=0.5;
        input_vel.linear.y=0.0;
        input_vel.linear.z=0.0;

        //Angular velocity measured in radians/second
        input_vel.angular.x=0.0;
        input_vel.angular.y=0.0;
        input_vel.angular.z=0.0;

        //Publish the velocities to the turtlebot movement topic
        cmd_vel.publish(input_vel);
        counter = 0;
        return;
    }

    if (moveType == 2){
        input_vel.linear.x=1.0;
        input_vel.linear.y=0.0;
        input_vel.linear.z=0.0;

        input_vel.angular.x=0.0;
        input_vel.angular.y=0.0;
        input_vel.angular.z=0.0;

        cmd_vel.publish(input_vel);
        counter = 0;
        return;
    
    }

    if (moveType == 3){
        input_vel.linear.x=-1.0;
        input_vel.linear.y=0.0;
        input_vel.linear.z=0.0;

        input_vel.angular.x=0.0;
        input_vel.angular.y=0.0;
        input_vel.angular.z=-1.0;

        cmd_vel.publish(input_vel);
        counter = 0;
        return;
    }

    if (moveType == 4){
        input_vel.linear.x=1.0;
        input_vel.linear.y=0.0;
        input_vel.linear.z=0.0;

        input_vel.angular.x=0.0;
        input_vel.angular.y=0.0;
        input_vel.angular.z=1.0;

        cmd_vel.publish(input_vel);
        counter = 0;
        return;
    }
}

return;
}




int main(int argc, char* argv[])
{
    //Initiatialize the main ROS connection
    ros::init(argc, argv, "Publisher"); 

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
    cout << "Frame size : " << dWidth << " x " << dHeight << endl;

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
                cout << "Coordinates: " << mc.x << ", " << mc.y << endl;
                circle(drawImg, mc, 5,Scalar(0,255,0));

                //Create a new empty windows and display the contour on it
                namedWindow("findContours", WINDOW_AUTOSIZE);

                imshow("findContours", drawImg);

                /*The image window has a 640x480 size. If the center of the shape(the green circle)
                  strays too far out of the windows centre, in this case, leaving the central 440x280 coordinates
                  a command will be sent to move the turtlebot towards the direction of the circle*/ 
                if(100 > mc.x)
                {
                    cout << "He is hitting the border on the left x-axis: " << mc.x << endl;

                    publish(1); // Left x-axis is "drive right" angular and linear
                }

                if (mc.x > 540)
                {
                    cout << "He is hitting the border on the right x-axis: " << mc.x << endl;

                    publish(2); // Right x-axis is "drive left" angular and linear
                }

                if ( 100 > mc.y)
                {
                    cout << "He is hitting the border on the upper y-axis: " << mc.y << endl;

                    publish(3); // Upper y-axis is "drive backwards" MinusLinear
                }

                if (mc.y > 380)
                {
                    cout << "He is hitting the border on the lower y-axis: " << mc.y << endl;

                    publish(4); // Lower y-axis is "drive forwards" Linear
                }
            }
        }

        //Wait for 'ESC' key press for 30ms. If 'ESC' key is pressed, exit the loop
        if (waitKey(30) == 27)
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }

      //Update the value of the counter variable such as to know how many times the 
      counter++;
    }
    
return 0;
}
