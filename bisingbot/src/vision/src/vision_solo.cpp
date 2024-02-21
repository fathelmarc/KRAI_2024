#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32MultiArray.h>

using namespace std;
using namespace cv;
using namespace ros;

int targetx = 44;
int targety = 408;

geometry_msgs::Quaternion koordinat;
void setCaptureFPS(cv::VideoCapture& cap, double fps) {
    cap.set(cv::CAP_PROP_FPS, fps);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "mata1");
    ros::NodeHandle nh;
    ros::Publisher cam = nh.advertise<geometry_msgs::Quaternion>("chat_vis",1000);

    // Initialize ROS
    Ptr<BackgroundSubtractor>pBackSub;
    pBackSub = createBackgroundSubtractorKNN();
    
    VideoCapture cap(2); 

    double desiredFPS = 30.0;
    setCaptureFPS(cap, desiredFPS);

    Mat img;
    Mat imgHSV, mask, imgColor;
    int hmin = 0, smin = 0, vmin = 0;
    int hmax = 179, smax = 255, vmax = 255;

    namedWindow("Trackbars", (640, 200)); // Create Window
    createTrackbar("Hue Min", "Trackbars", &hmin, 179);
    createTrackbar("Hue Max", "Trackbars", &hmax, 179);
    createTrackbar("Sat Min", "Trackbars", &smin, 255);
    createTrackbar("Sat Max", "Trackbars", &smax, 255);
    createTrackbar("Val Min", "Trackbars", &vmin, 255);
    createTrackbar("Val Max", "Trackbars", &vmax, 255);

    double fps = cap.get(CAP_PROP_FPS);
    cout << "Frames per second camera : " << fps << endl;

     // Number of frames to capture
    int num_frames = 1;

    // Start and end times
    clock_t start;
    clock_t end;

    double ms, fpsLive;

    int keyboard;
   
    // Check if the camera opened successfully
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera.");
        return -1;
    }

    while (ros::ok()) {
        int max;

        Mat frame, fgMask;
        start = clock();
        cap.read(frame); 

        pBackSub->apply(frame, fgMask);

        resize(frame,frame,Size (640,480));
        resize(fgMask,fgMask,Size (640,480));

        erode(fgMask, fgMask, (5, 5));
        dilate(fgMask, fgMask, (5, 5));

        Mat imghsv;
        cvtColor(frame, imghsv, COLOR_BGR2HSV);
        
        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);

        int objectCenter;

        Mat maskobject1;
        inRange(imghsv, lower, upper, maskobject1);

        long sum = 0;
        int N = 1;

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                sum += 1;
            }
        }

        if (frame.empty()) {
            ROS_WARN("Empty frame received from camera.");
            continue;
        }
        end = clock();

        vector<vector<Point>> contoursobject1;
        findContours(maskobject1, contoursobject1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            
        double maxArea = 0;
        int maxAreaIdx = -1;

        for (size_t i = 0; i < contoursobject1.size(); ++i) {
            double area = contourArea(contoursobject1[i]);
            if (area > maxArea) {
                maxArea = area;
                maxAreaIdx = static_cast<int>(i);
            }
        }

        if (maxAreaIdx != -1 && maxArea > 100) {
            //Rectangle Object 1
                const auto& contour = contoursobject1[maxAreaIdx];

            // Your existing code for processing the contour goes here
            Moments oMoments = moments(contour);
            double dM01 = oMoments.m01;
            double dM10 = oMoments.m10;
            double dArea = oMoments.m00;

            Rect boundingBox = boundingRect(contour);
            rectangle(frame, boundingBox, Scalar(255, 0, 0), 2);

            // Circle Object2
            Point circleCenter(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
            int circleRadius = (boundingBox.width + boundingBox.height) / 4;
            circle(frame, circleCenter, circleRadius, Scalar(0, 0, 255), 2);

            // Poin Object1
            Point objectCenter(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
            Point frameCenter(frame.cols / 2, frame.rows);

            //Rectangle background object 1
            Size textSize = getTextSize("Object1", FONT_HERSHEY_SIMPLEX, 0.5, 2, nullptr);
            rectangle(frame, Point(boundingBox.x, boundingBox.y - textSize.height - 10), Point(boundingBox.x + textSize.width, boundingBox.y), Scalar(255, 0, 0), FILLED);

            // //garis dan keterangan
            // putText(frame, "Object1", Point(boundingBox.x, boundingBox.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);
            // line(frame, frameCenter, objectCenter, Scalar(0, 255, 0), 2);
            // circle(frame, Point(objectCenter), 5, Scalar(100, 255, 100), FILLED);
            //line(frame,Point(320,100),Point(220,50),Scalar(0,255,100),2);

            //Koordinat Object 1
            string coordinateText2 = "(" + to_string(objectCenter.x) + ", " + to_string(objectCenter.y) + ")";
            putText(frame, coordinateText2, Point(objectCenter.x - 20, objectCenter.y - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(250, 50, 255), 2);
            double seconds = (double(end) - double(start)) / double(CLOCKS_PER_SEC);
            fpsLive = double(num_frames) / double(seconds);
            putText(frame, "FPS: " + to_string(int(fpsLive)), { 50, 50 }, FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2);

            koordinat.x = targetx;
            koordinat.y = targety;
            koordinat.z = objectCenter.x;
            koordinat.w = objectCenter.y;
            cam.publish(koordinat);
        }   
        spinOnce();
        keyboard = waitKey(1);
        if (keyboard == 'q' or keyboard == 27) {
            break;
        }
        imshow("frame", frame);
        imshow("obj1", maskobject1);

    }

    return 0;
}
