#include "../include/follower.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>

NodeHandle n;
ros::Publisher follower_pub = n.advertise<std_msgs::String>("find_line", 1000);

//在终端中使用ctrl+c可以强制终止运行的程序，但有时需要在终止时作一些处理（如ros::shotdown,free等），可使用信号函数作退出处理
void SigintHandler(int sig)
{
  ROS_INFO_STREAM("Follower Node Is Shutting Down");
  shutdown();
}

Mat loadFrame(const sensor_msgs::ImageConstPtr& msg)
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

Mat loadDepthFrame(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr tempResult = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    Mat result = tempResult->image.clone();
    return result;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

Mat createGreyscale(Mat input_image)
{
  Mat result;
  cvtColor(input_image, result, CV_BGR2GRAY);
  return result;
}

Mat createColor(Mat input_image)
{
  Mat result;
  cvtColor(input_image, result, COLOR_GRAY2BGR);
  return result;
}

Mat edgeDetection(Mat input_frame, int lowThreshold, int highThreshold, int kernel_size)
{
  Mat result;
  Canny(input_frame, result, lowThreshold, highThreshold, kernel_size);
  return result;
}

Mat gaussianBlur(Mat input_frame, int size, int point)
{
  Mat result;
  GaussianBlur(input_frame, result, Size(size, size), point, point);
  return result;
}

vector<Vec4i> probLineTransform(Mat input_image, int threshold, int min_line, int max_gap)
{
  vector<Vec4i> result;
  HoughLinesP(input_image, result, 1, CV_PI/180, threshold, min_line, max_gap);
  return result;
}

int longestLineIndex(vector<Vec4i> input_vector)
{
  int result;
  for(size_t i = 0; i < input_vector.size(); i++)
  {
    Vec4i l = input_vector[i];
    int length1 = sqrt(((l[3]-l[1])^2) + ((l[2]-l[0])^2));
    int length2;
    if(i == 0 || length1 >= length2)
    {
      result = i;
      length2 = length1;
    }
  }
  return result;
}

void drawingLines(vector<Vec4i> lines, Mat image, int r, int g, int b)
{
  for(size_t i = 0; i < lines.size(); i++)
  {
    Vec4i l = lines[i];
    line(image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(b,g,r), 2, LINE_AA);
  }
}

void imageCallbackRGB(const sensor_msgs::ImageConstPtr& msg)
{
  // Load full infrared frame
  // Mat frameRGB = loadFrame(msg);
  
  //resize
  // Mat t_matResize;
  // resize(frameRGB, t_matResize, Size(1280, 720));
  // // Mat t_matHSV = Mat::zeros(t_matResize.size(), CV_8UC3);
  // // cvtColor(t_matResize, t_matHSV, COLOR_BGR2HSV);
  // // cout << "1" << endl;

  //shaixuan red
  // Mat croppedFrame = frameRGB;
  // Mat croppedFrame = frameRGB;
  // for(int row=0 ; row<t_matResize.rows ; ++row)
  // {

  //   for(int col=0 ; col<t_matResize.cols ; ++col)
  //   {
  //     // Scalar s_hsv = t_matHSV.at<Vec3b>(row, col);//获取像素点为（j, i）点的HSV的值
  //     // if( ((s_hsv.val[0]>0) && (s_hsv.val[0]<8)) || 
  //     //     ((s_hsv.val[0]>120) && (s_hsv.val[0]<180)) )
  //     if(100 > t_matResize.at<Vec3b>(row, col)[0] &&
  //        100 > t_matResize.at<Vec3b>(row, col)[1] &&
  //        160 < t_matResize.at<Vec3b>(row, col)[2])
  //     {
  //       croppedFrame.at<Vec3b>(row, col)[0] = t_matResize.at<Vec3b>(row, col)[0];
  //       croppedFrame.at<Vec3b>(row, col)[1] = t_matResize.at<Vec3b>(row, col)[1];
  //       croppedFrame.at<Vec3b>(row, col)[2] = t_matResize.at<Vec3b>(row, col)[2];
  //     }

  //   }

  // }

  // // imshow("IR Image", t_matResize);
  // // imshow("IR Probabilistic Line Transform", croppedFrame);

  // Mat t_matGray = Mat::zeros(croppedFrame.size(), CV_8UC1);
  // Mat croppedFrame_gray = createGreyscale(croppedFrame);
  // imshow("gray", croppedFrame_gray);

  // Mat t_matCany;
  // Canny(t_matGray, t_matCany, 25, 100, 3);
  // Mat t_matCanyBGR;
  // cvtColor(t_matCany, t_matCanyBGR, COLOR_GRAY2BGR);
  // // ROS_INFO("3");

  // imshow("gray", t_matCany);
  // imshow("color", t_matCanyBGR);



  // waitKey(20);
  // return;

  // Load full color frame
  Mat srcFrame = loadFrame(msg);
  Mat croppedFrame;
  resize(srcFrame, croppedFrame, srcFrame.size()/3*2);
  Mat t_matHSV = Mat::zeros(croppedFrame.size(), CV_8UC3);
  cvtColor(croppedFrame, t_matHSV, COLOR_BGR2HSV);

  // Defining crop area
  // Rect myROI(0, 0, 1920, 1080);/

  // Cropping full color image
  Mat t_matFindCountor = Mat::zeros(croppedFrame.size(), CV_8UC1);
  for(int row=0 ; row<croppedFrame.rows ; ++row)
  {

    for(int col=0 ; col<croppedFrame.cols ; ++col)
    {
      Scalar s_hsv = t_matHSV.at<Vec3b>(row, col);//获取像素点为（j, i）点的HSV的值
      if( (((s_hsv.val[0]>0) && (s_hsv.val[0]<8)) || ((s_hsv.val[0]>160) && (s_hsv.val[0]<180))) &&
             s_hsv.val[1]>150 && s_hsv.val[2]>100 && s_hsv.val[2]<240)
      // if(100 > croppedFrame.at<Vec3b>(row, col)[0] &&
      //    100 > croppedFrame.at<Vec3b>(row, col)[1] &&
      //    160 < croppedFrame.at<Vec3b>(row, col)[2])
      {
        t_matFindCountor.at<uchar>(row, col) = 255;
      }
      // else 
      // {
      //   croppedFrame.at<Vec3b>(row, col)[0] = 0;
      //   croppedFrame.at<Vec3b>(row, col)[1] = 0;
      //   croppedFrame.at<Vec3b>(row, col)[2] = 0;
      // }

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
    
  
  if (phi < CV_PI/4. || phi > 3.*CV_PI/4.)// ~vertical line
  {
      cv::Point pt1(rho/cos(phi), 0);
      cv::Point pt2((rho - croppedFrame.rows * sin(phi))/cos(phi), croppedFrame.rows);
      cv::line( croppedFrame, pt1, pt2, cv::Scalar(0, 255, 0), 2);
      circle(croppedFrame, pt1, 8, Scalar(0,0,0),-1);
      circle(croppedFrame, pt2, 8, Scalar(0,255,0),-1);
  }
  else
  {
      cv::Point pt1(0, rho/sin(phi));
      cv::Point pt2(croppedFrame.cols, (rho - croppedFrame.cols * cos(phi))/sin(phi));
      cv::line( croppedFrame, pt1, pt2, cv::Scalar(0, 255, 0), 2);
      circle(croppedFrame, pt1, 8, Scalar(0,0,0),-1);
      circle(croppedFrame, pt2, 8, Scalar(0,255,0),-1);
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
  imwrite("./result/"+to_string(t_iindex++)+".png", croppedFrame);
  imshow("result", croppedFrame);
  waitKey(5);// Load full color frame



  // std_msgs::String msg;

  // std::stringstream ss;
  // ss << "Test sending topic" << count;
  // // ss << "position ratio" << pt2.x;
  // msg.data = ss.str();

  // track_info_pub.publish(msg);

  // ROS_INFO("%s", msg.data.c_str());



  
  // Mat frameRGB = loadFrame(msg);
  // Mat t_matHSV = Mat::zeros(frameRGB.size(), CV_8UC3);
  // cvtColor(frameRGB, t_matHSV, COLOR_BGR2HSV);

  // // Defining crop area
  // // Rect myROI(0, 0, 1920, 1080);/

  // // Cropping full color image
  // Mat croppedFrame = frameRGB;
  // Mat
  // for(int row=0 ; row<croppedFrame.rows ; ++row)
  // {

  //   for(int col=0 ; col<croppedFrame.cols ; ++col)
  //   {
  //     Scalar s_hsv = t_matHSV.at<Vec3b>(row, col);//获取像素点为（j, i）点的HSV的值
  //     if( (((s_hsv.val[0]>0) && (s_hsv.val[0]<8)) || ((s_hsv.val[0]>160) && (s_hsv.val[0]<180))) &&
  //            s_hsv.val[1]>80 && s_hsv.val[2]>50 && s_hsv.val[2]<220)
  //     // if(100 > croppedFrame.at<Vec3b>(row, col)[0] &&
  //     //    100 > croppedFrame.at<Vec3b>(row, col)[1] &&
  //     //    160 < croppedFrame.at<Vec3b>(row, col)[2])
  //     {
  //       continue;
  //     }
  //     else 
  //     {
  //       croppedFrame.at<Vec3b>(row, col)[0] = 0;
  //       croppedFrame.at<Vec3b>(row, col)[1] = 0;
  //       croppedFrame.at<Vec3b>(row, col)[2] = 0;
  //     }

  //   }
  // }
  // // Apply Gaussian blur
  // GaussianBlur(croppedFrame, croppedFrame, Size(9, 9), 2, 2);

  // // Converting cropped image to greyscale
  // Mat croppedFrame_gray, edges, color_canny_image;
  // cvtColor(croppedFrame, croppedFrame_gray, COLOR_BGR2GRAY);
  // Canny(croppedFrame_gray, edges, 25, 100, 3);
  // vector<Vec4i> lines;
  
  // /*lines:是一个vector<Vec4i>,Vec4i是一个包含4个int数据类型的结构体，[x1,y1,x2,y2],可以表示一个线段。
  // rho:就是一个半径的分辨率。
  // theta:角度分辨率。
  // threshold:判断直线点数的阈值。
  // minLineLength：线段长度阈值。
  // minLineGap:线段上最近两点之间的阈值。*/
  // HoughLinesP(edges, lines, 1, CV_PI/180, 200, 200, 10);
  
  // cvtColor(edges, color_canny_image, COLOR_GRAY2BGR);
  // for(size_t i = 0; i < lines.size(); i++)
  // {
  //     Vec4i l = lines[i];
  //     line(color_canny_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(20, 50, 230), 2, LINE_AA);
  // }

  // // // Edge detection
  // // Mat edges = edgeDetection(croppedFrame_gray, 25, 100, 3);

  // // // Converting greyscale image to color for drawing
  // // Mat pltRGB = createColor(edges);

  // // // Probabilistic Line Transform
  // // vector<Vec4i> lines = probLineTransform(edges, 50, 50, 10);

  // // Drawing detected lines
  // // drawingLines(lines, pltRGB, 255, 255, 0);

  // Mat t_matSrcReaize, t_matResultResize;
  // resize(frameRGB, t_matSrcReaize, frameRGB.size()/2);
  // resize(color_canny_image, t_matResultResize, color_canny_image.size()/2);

  // Mat t_matAll(Size(t_matSrcReaize.cols*2+30,t_matSrcReaize.rows), CV_8UC3);
  // t_matSrcReaize.copyTo(t_matAll(Rect(0, 0, t_matSrcReaize.cols, t_matSrcReaize.rows)));
  // t_matResultResize.copyTo(t_matAll(Rect(t_matSrcReaize.cols+30, 0, t_matResultResize.cols, t_matResultResize.rows)));
  
  // putText(t_matAll, to_string(lines.size()), 
  //         Point(t_matAll.cols-100, 20), 
  //         cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 20), 1, 8, 0);
  
  // imshow("result",t_matAll);
        

  // waitKey(30);
}

void imageCallbackIR(const sensor_msgs::ImageConstPtr& msg)
{
  // Load full color frame
  Mat frameIR = loadFrame(msg);

  // Defining crop area
  Rect myROI(0, 0, 1920, 1080);

  // Cropping full color image
  Mat croppedFrame = frameIR(myROI);

  // Apply Gaussian blur
  Mat croppedFrameBlurred = gaussianBlur(croppedFrame, 9, 2);

  // Edge detection
  Mat edges = edgeDetection(croppedFrameBlurred, 50, 200, 3);

  // Converting greyscale image to color for drawing
  Mat pltIR = createColor(edges);

  // Probabilistic Line Transform
  vector<Vec4i> lines = probLineTransform(edges, 50, 30, 10);

  // Drawing detected lines
  drawingLines(lines, pltIR, 0, 0, 255);

  imshow("IR Image", frameIR);
  imshow("IR Probabilistic Line Transform", pltIR);

  waitKey(30);
}
