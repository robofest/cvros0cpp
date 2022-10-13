#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;

public:
  ImageConverter() : it_{nh_}
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cam_pub/image_raw", 1, &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_p;
    try
    {
      cv_p = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    cv::resize(cv_p->image, cv_p->image, cv::Size(cv_p->image.cols*0.5, cv_p->image.rows*0.5), 0, 0, cv::INTER_LINEAR);
    if (cv_p->image.rows > 60 && cv_p->image.cols > 60)
      cv::circle(cv_p->image, cv::Point(cv_p->image.cols/2, cv_p->image.rows/2), cv_p->image.rows/2, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_p->image);
    cv::waitKey(3);

    // Output modified video stream
    // image_pub_.publish(cv_p->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw1Circle");
  ImageConverter ic{};
  ros::spin();
  return 0;
}
