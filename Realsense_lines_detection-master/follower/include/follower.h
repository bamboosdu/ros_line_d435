#ifndef follower_h
#define follower_h

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <vector>
#include <signal.h>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <librealsense2/rs.hpp>

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h> // uses the "Trigger.srv" message defined in ROS


using namespace std;
using namespace cv;
using namespace ros;


// void SigintHandler(int sig);


class FindLine
{
public:
    FindLine(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::ServiceServer minimal_service_;
    ros::Publisher  minimal_publisher_;
    
    double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    double val_to_remember_; // member variables will retain their values even as callbacks come and go
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();
    
    void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber
    //prototype for callback for example service
    bool serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

public:
    Mat loadFrame(const sensor_msgs::ImageConstPtr& msg);
    void imageCallbackRGB(const sensor_msgs::ImageConstPtr& msg);
}; // note: a class definition requires a semicolon at the end of the definition

#endif
