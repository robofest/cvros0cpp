#include <ros/ros.h>
// Includes for working with images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#define CVWIN_PREVIEW "Threshold Preview"

class OpenCV0
{
  public:
    OpenCV0();
    ~OpenCV0();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

  private:
    float whiteAmount(const cv::Mat& image);
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pub_;
    int thresh_;
};

OpenCV0::OpenCV0() : it_{nh_}
{
    image_sub_ = it_.subscribe("/cam_pub/image_raw", 1, &OpenCV0::imageCb, this); // cam_pub pubs 
    thresh_ = 140;
}

OpenCV0::~OpenCV0()
{
    cv::destroyWindow(CVWIN_PREVIEW);
}

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

    // Convert the source to grayscale
    cv::Mat image_gray;
    cv::Mat gray2;
    cv::cvtColor(cv_ptr->image, image_gray, CV_BGR2GRAY);
    cv::resize(image_gray, gray2, cv::Size(image_gray.cols*0.5, image_gray.rows*0.5), 0, 0, cv::INTER_LINEAR);
    //cv::resize(image_gray, gray2, cv::Size(300,200), cv::INTER_LINEAR);
    cv::imshow("Gray Image", gray2); // Show the grayscale image
    
    constexpr int max_BINARY_value = 255;
    constexpr int thresh_type = 0; // Normal
    cv::Mat image_thresh; // for B/W image
    cv::threshold(gray2, image_thresh, thresh_, max_BINARY_value, thresh_type);

    float white_amount = whiteAmount(image_thresh); // get the ratio of white pixels
    ROS_INFO_STREAM("white/all pixel ratio: " << white_amount);
  
    // Show B/W image
    cv::putText(image_thresh, std::to_string(int(white_amount*100))+"%", 
	    cv::Point(30,30), // Coordinates
            cv::FONT_HERSHEY_SIMPLEX, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(200,200,200), // BGR Color --- Bright gray
            2); // Line Thickness (Optional)
    cv::imshow(CVWIN_PREVIEW, image_thresh);
    cv::waitKey(20); // sleeps for 20 milisec to update GUI window
}

/**
 * Count White Pixels: Return the ratio of white pixels
 * full white: 1.0         full black: 0.0
 *
 * https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html?highlight=threshold
 */
float OpenCV0::whiteAmount(const cv::Mat& image)
{
    int white_cnt = cv::countNonZero(image);
    return (float)white_cnt/(image.cols*image.rows);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "opencv1");
    OpenCV0 sd{};
    ros::spin();
    return 0;
}
