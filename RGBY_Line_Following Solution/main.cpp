// Include files for required libraries
#include <stdio.h>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

Pi2c car(0x04); // Configure the I2C interface to the Car as a global variable
using namespace std;
using namespace cv;

void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV
}

void send_error(int16_t error);

int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices

    // Create a GUI window
    namedWindow("Coloured Line");

    while(1)    // Main loop to perform image processing
    {
        Mat frame,colouredline;
        int16_t error, cumulative_error=0, prev_error, PID; //variables for PID
        double Kp=0.4, Ki=0.4, Kd=0.3; //PID constants

        while(frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        flip(frame,frame,0); //flip frame around x-axis
        flip(frame,frame,1); //flip frame around y-axis
        frame = frame(Range(140,240),Range(20,300)); //crop frame

        Mat frameHSV;
        //Convert frame to HSV
        cvtColor(frame,frameHSV,COLOR_BGR2HSV);
        // Gaussian Blur
        GaussianBlur(frameHSV,frameHSV,Size(11,11),0,0);

        int colours_hue[5]={0,20,35,78,120};
        int min_val,max_val,Maximum=-999;
        Mat HSV_compare = frameHSV.clone();
        for(int i=0;i<4;i++){
            inRange(HSV_compare,Scalar(colours_hue[i],50,40),Scalar((colours_hue[i+1]-1),255,255),frameHSV);
            if(countNonZero(frameHSV)>Maximum){
                min_val = colours_hue[i];
                max_val = colours_hue[i+1]-1;
            }
        }
        // Detect Coloured Line
        inRange(HSV_compare,Scalar(min_val,50,40),Scalar(max_val,255,255),colouredline);

        //Morphology operation
        Mat maskMorph = getStructuringElement(MORPH_ELLIPSE, Size(10, 10));
        dilate(colouredline,colouredline,maskMorph);

        // Get a pointer to the start of the column
        // get pixel NB: (y,x) / (row,col)
        //width is 280 pixels/columns, height is 100 pixels/rows
        int weighted_pixels[colouredline.cols];
        int unweightedpixels[colouredline.cols];
        //Initialise all array locations to 0.
        for(int i=0;i<colouredline.cols;i++){
            weighted_pixels[i]=0;
            unweightedpixels[i]=0;
        }
        int weight=-140;
        for(int i=0;i<colouredline.cols;i++){
            for(int j=0;j<colouredline.rows;j++){
                int pixels=static_cast<int>(colouredline.at<unsigned char>(j,i));
                weighted_pixels[i]+=(pixels/255);
                unweightedpixels[i]+=(pixels/255);
            }
            weighted_pixels[i]=weighted_pixels[i]*weight;
            weight++;
        }
        int sum_weight=0;
        int sum=0;
        int weighted_average;
        for(int i=0;i<colouredline.cols;i++){
            sum_weight+=weighted_pixels[i];
            sum+=unweightedpixels[i];
        }
        weighted_average=sum_weight/sum;
        error=weighted_average;

        //PID Code for Arduino
        //when going left, error positive. when going right, negative

        PID = (Kp*error)+(Ki*cumulative_error)+(Kd*(error-prev_error));
        cout << "PID:  " << PID<< endl;
        car.i2cWriteArduinoInt(PID);
        prev_error = error;
        cumulative_error+=error;

        imshow("Coloured Line", colouredline);
        int key = waitKey(1);   // Wait 1ms for a keypress (required to update windows)

        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;
	}
	closeCV();  // Disable the camera and close any windows
	return 0;
}





