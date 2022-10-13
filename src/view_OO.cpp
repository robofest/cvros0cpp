#include <ros/ros.h>
// Includes for working with images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

class OpenCV0
{
  public:
    OpenCV0() : it{nh}
    {
        image_sub = it.subscribe("/cam_pub/image_raw", 1, &OpenCV0::imageCb, this);
    }
    ~OpenCV0()
    {
        cv::destroyWindow("Color Image");
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

  private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
};


/**
 * Called once every time a image is published on the topic this node
 * is subscribed to. The image is passed to the function as a ImageConstPtr
 */
void OpenCV0::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    //Convert to cv image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), 0.5, 0.5);
    cv::imshow("Color Image", cv_ptr->image); // Show the color image
    cv::waitKey(20); // sleeps for 20 milisec to update GUI window
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "opencv0");
    OpenCV0 sd{};
    ros::spin();
    return 0;
}