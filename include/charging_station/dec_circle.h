#ifndef DEC_CIRCLE_H
#define DEC_CIRCLE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <map>

namespace charging_station {

class ImageRectDetector {
public:
    ImageRectDetector();

    // 声明 getChargingStationPose 函数
    geometry_msgs::Pose getChargingStationPose();

    // 声明 getChargingStationPoseStatic 函数
    static geometry_msgs::Pose getChargingStationPoseStatic();

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat getDepthImage(const ros::Time& timestamp);
    void processImage(cv::Mat& image);
    void drawCross(cv::Mat& img, cv::Point center, cv::Scalar color = cv::Scalar(255, 0, 0), int thickness = 1, int size = 10);
    cv::Mat thresholdImage(const cv::Mat& image, double threshold_value = 127, double max_value = 255, int method = cv::THRESH_BINARY);
    double calculatePerimeter(const std::vector<cv::Point>& cnt);
    bool isRectangleLike(const std::vector<cv::Point>& cnt, const std::vector<cv::Point>& approx, double min_aspect_ratio = 3.7, double max_aspect_ratio = 7.7, double min_fill_ratio = 0.5);
    std::vector<std::tuple<cv::Point, int, int>> filterRegions(const cv::Mat& image, double min_area, double max_area);
    cv::Vec3f calculateRectangleNormal(const cv::Mat& depth_image, const std::vector<cv::Point>& rectangle_corners);
    cv::Vec3f imageToCameraCoordinates(const cv::Mat& depth_image, cv::Point center, double fx, double fy, double cx, double cy);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Publisher center_pub_;
    ros::Publisher normal_pub_;
    cv_bridge::CvImagePtr cv_ptr_;
    std::map<ros::Time, cv::Mat> depth_image_buffer_;
    double fx_, fy_, cx_, cy_;
    geometry_msgs::Pose charging_station_pose_; // 添加 charging_station_pose_ 成员变量
};

} // namespace charging_station

#endif // DEC_CIRCLE_H