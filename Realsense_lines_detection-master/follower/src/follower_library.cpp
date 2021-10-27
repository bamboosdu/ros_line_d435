#include "../include/follower.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>

//在终端中使用ctrl+c可以强制终止运行的程序，但有时需要在终止时作一些处理（如ros::shotdown,free等），可使用信号函数作退出处理
void SigintHandler(int sig)
{
  ROS_INFO_STREAM("Follower Node Is Shutting Down");
  shutdown();
}


//example_ros_class.cpp:
FindLine::FindLine(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of FindLine");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    //initializeServices();
    
    //initialize variables here, as needed
    val_to_remember_=0.0; 
    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void FindLine::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    minimal_subscriber_ = nh_.subscribe("/usb_cam/image_raw", 1, &FindLine::imageCallbackRGB, this);  
    // add more subscribers here, as needed
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void FindLine::initializeServices()
{
    ROS_INFO("Initializing Services");
    minimal_service_ = nh_.advertiseService("example_minimal_service",
                                                   &FindLine::serviceCallback,
                                                   this);  
    // add more services here, as needed
}

//member helper function to set up publishers;
void FindLine::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    minimal_publisher_ = nh_.advertise<std_msgs::Float32>("line_info", 100, true); 
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to minimal_publisher_ (which is a member method)
void FindLine::subscriberCallback(const std_msgs::Float32& message_holder) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    val_from_subscriber_ = message_holder.data; // copy the received data into member variable, so ALL member funcs of ExampleRosClass can access it
    ROS_INFO("myCallback activated: received value %f",val_from_subscriber_);
    std_msgs::Float32 output_msg;
    val_to_remember_ += val_from_subscriber_; //can use a member variable to store values between calls; add incoming value each callback
    output_msg.data= val_to_remember_;
    // demo use of publisher--since publisher object is a member function
    minimal_publisher_.publish(output_msg); //output the square of the received value; 
}

//member function implementation for a service callback function
bool FindLine::serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
    ROS_INFO("service callback activated");
    response.success = true; // boring, but valid response info
    response.message = "here is a response string";
    return true;
}



Mat FindLine::loadFrame(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr tempResult = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat result = tempResult->image.clone();
    return result;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}
void FindLine::imageCallbackRGB(const sensor_msgs::ImageConstPtr& msg)
{
  // Load full color frame
  Mat srcFrame = loadFrame(msg);
  Mat croppedFrame;
  resize(srcFrame, croppedFrame, srcFrame.size()/3*2);
  Mat t_matHSV = Mat::zeros(croppedFrame.size(), CV_8UC3);
  cvtColor(croppedFrame, t_matHSV, COLOR_BGR2HSV);

  // Cropping full color image
  Mat t_matFindCountor = Mat::zeros(croppedFrame.size(), CV_8UC1);
  for(int row=0 ; row<croppedFrame.rows ; ++row)
  {

    for(int col=0 ; col<croppedFrame.cols ; ++col)
    {
      Scalar s_hsv = t_matHSV.at<Vec3b>(row, col);//获取像素点为（j, i）点的HSV的值
      if( (((s_hsv.val[0]>0) && (s_hsv.val[0]<8)) || ((s_hsv.val[0]>160) && (s_hsv.val[0]<180))) &&
             s_hsv.val[1]>150 && s_hsv.val[2]>100 && s_hsv.val[2]<240)
      {
        t_matFindCountor.at<uchar>(row, col) = 255;
      } 

    }
  }
  // Apply Gaussian blur
  GaussianBlur(croppedFrame, croppedFrame, Size(9, 9), 2, 2);
  GaussianBlur(t_matFindCountor, t_matFindCountor, Size(9, 9), 2, 2);

  vector<vector<Point>> t_vecContours;
  findContours(t_matFindCountor, t_vecContours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
  //find width+height max contour
  int t_iMaxIndex = -1;
  int t_iMaxWaddH = 0;
  for(int index=0 ; index<t_vecContours.size() ; ++index)
  {
    Rect t_rectNow = boundingRect(Mat(t_vecContours[index]));
    // if( (3 > t_rectNow.width/t_rectNow.height && 3 > t_rectNow.height/t_rectNow.width) ||
    if(200 > t_rectNow.width + t_rectNow.height)
    {
      rectangle(croppedFrame, 
                t_rectNow, 
                cv::Scalar(255, 0, 0), 2);
      putText(croppedFrame, to_string(t_rectNow.width)+"_"+to_string(t_rectNow.height), 
              Point(t_rectNow.x+t_rectNow.width, t_rectNow.y+20), 
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2, 8, 0);
      continue;
    }

    if(t_iMaxWaddH < t_rectNow.width+t_rectNow.height)
    {
      t_iMaxIndex = index;
      t_iMaxWaddH = t_rectNow.width+t_rectNow.height;
    }
  }
  
  if(-1 == t_iMaxIndex)
  {
    putText(croppedFrame, "no rect", 
            Point(20, croppedFrame.rows-50), 
            cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 250), 2, 8, 0);
    imshow("result", croppedFrame);
    waitKey(5);// Load full color frame
    return;
  }

  //fitLine
  cv::Vec4f line;
  cv::fitLine(t_vecContours[t_iMaxIndex],
              line,
              DIST_HUBER,
              0,
              0.01,
              0.01);

  double cos_theta = line[0];
  double sin_theta = line[1];
  double x0 = line[2], y0 = line[3];

  double phi = atan2(sin_theta, cos_theta) + CV_PI / 2.0;
  double rho = y0 * cos_theta - x0 * sin_theta;

  // std::cout << "phi = " << phi / CV_PI * 180 << std::endl;
  // std::cout << "rho = " << rho << std::endl;

  double k = sin_theta / cos_theta;

  double b = y0 - k * x0;

  double x = 0;
  double y = k * x + b;
  
  
  std_msgs::Float32 output_msg;
  if (phi < CV_PI/4. || phi > 3.*CV_PI/4.)// ~vertical line
  {
    cv::Point pt1(rho/cos(phi), 0);
    cv::Point pt2((rho - croppedFrame.rows * sin(phi))/cos(phi), croppedFrame.rows);
    cv::line( croppedFrame, pt1, pt2, cv::Scalar(0, 255, 0), 2);
    circle(croppedFrame, pt1, 8, Scalar(0,0,0),-1);
    circle(croppedFrame, pt2, 8, Scalar(0,255,0),-1);
    
    cout << float(pt2.x) << "\t/\t" << float(croppedFrame.cols);
    output_msg.data = float(pt2.x) / float(croppedFrame.cols);
  }
  else
  {
    cv::Point pt1(0, rho/sin(phi));
    cv::Point pt2(croppedFrame.cols, (rho - croppedFrame.cols * cos(phi))/sin(phi));
    cv::line( croppedFrame, pt1, pt2, cv::Scalar(0, 255, 0), 2);
    circle(croppedFrame, pt1, 8, Scalar(0,0,0),-1);
    circle(croppedFrame, pt2, 8, Scalar(0,255,0),-1);
    
    cout << float(pt2.x) << "\t/\t" << float(croppedFrame.cols);
    output_msg.data = float(pt2.x) / float(croppedFrame.cols);
  }

  Rect t_rectMax = boundingRect(Mat(t_vecContours[t_iMaxIndex]));
  rectangle(croppedFrame, 
            t_rectMax, 
            cv::Scalar(0, 255, 0), 2);
  double t_dAngle = phi / CV_PI * 180;
  if(90 > t_dAngle)
  {
    t_dAngle = 90 - t_dAngle;
    putText(croppedFrame, "phi:"+to_string(t_dAngle), 
            Point(croppedFrame.cols-500, croppedFrame.rows-50), 
            cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 250), 2, 8, 0);
    circle(croppedFrame, Point(t_rectMax.x, t_rectMax.y+t_rectMax.height), 16, Scalar(0,255,0),-1);
  }
  else
  {
    t_dAngle = t_dAngle - 90;
    putText(croppedFrame, "phi:"+to_string(t_dAngle), 
            Point(20, croppedFrame.rows-50), 
            cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 250), 2, 8, 0);
    circle(croppedFrame, Point(t_rectMax.x+t_rectMax.width, t_rectMax.y+t_rectMax.height), 16, Scalar(0,255,0),-1);
  }
  static int t_iindex = 0;

  minimal_publisher_.publish(output_msg); //output the square of the received value; 

  imwrite("./result/"+to_string(t_iindex++)+".png", croppedFrame);
  imshow("result", croppedFrame);
  waitKey(5);// Load full color frame
}