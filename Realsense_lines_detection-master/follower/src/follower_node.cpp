#include "../include/follower.h"
#include "follower_library.cpp"

int main(int argc, char** argv)
{
  init(argc, argv, "follower", init_options::NoSigintHandler);
  
  //NodeHandles节点类
  //创建时候，如果内部节点没有开始，ros::NodeHandle会开始节点，ros::NodeHandle实例销毁，节点就会关闭。
  NodeHandle n;
  ROS_INFO_STREAM("Demo Node Is Up"); //shu chu info
  FindLine exampleRosClass(&n);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
  
  //会订阅/camera/color/image_raw 同时将数据送入回调函数
  //Subscriber imageSubRGB = n.subscribe("/usb_cam/image_raw", 1, &exampleRosClass.imageCallbackRGB);
  // Subscriber imageSubIR = n.subscribe("/camera/infra1/image_rect_raw", 1, imageCallbackIR);

  //在终端中使用ctrl+c可以强制终止运行的程序，但有时需要在终止时作一些处理（如ros::shotdown,free等），可使用信号函数作退出处理
  signal(SIGINT, SigintHandler);
  Rate rate(30); // 30Hz

  while (ok())
  {

    rate.sleep();
    spinOnce();
  }
  return 0;
}

