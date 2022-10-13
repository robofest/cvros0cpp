// http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    cv_bridge::CvImagePtr imp = cv_bridge::toCvCopy(msg, "bgr8"); // mutable
    cv::Mat src;
    cv::Mat dst;
    cv::flip(imp->image, imp->image, 1); // 1: flip y axis
    cv::resize(imp->image, imp->image, cv::Size(), 0.5, 0.5);
    cv::imshow("view", imp->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:%s", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opencv0a_node");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("cam_pub/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}