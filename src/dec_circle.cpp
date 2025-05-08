#include "charging_station/dec_circle.h"
#include <tf/transform_datatypes.h>

namespace charging_station {

// 图像矩形检测器类构造函数
ImageRectDetector::ImageRectDetector()
    : it_(nh_) {
    // image_sub_ = it_.subscribe("/xv_sdk/xv_dev/fisheye_cameras/left/image", 1, &ImageRectDetector::imageCallback, this);
    // image_sub_ = it_.subscribe("camera/image", 1, &ImageRectDetector::imageCallback, this);

    // depth_sub_ = it_.subscribe("/xv_sdk/xv_dev/fisheye_cameras/left/depth", 1, &ImageRectDetector::depthCallback, this);
    // center_pub_ = nh_.advertise<geometry_msgs::Point>("/detected_rectangle_center", 10);
    // normal_pub_ = nh_.advertise<geometry_msgs::Vector3>("/detected_rectangle_normal", 10);
    // fx_ = 500;
    // fy_ = 500;
    // cx_ = 320;
    // cy_ = 240;
}

// 图像回调函数
void ImageRectDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_INFO("Received image with dimensions: %d x %d", cv_ptr_->image.cols, cv_ptr_->image.rows);

    cv::Mat processed_image = cv_ptr_->image.clone();
    processImage(processed_image);
}

void ImageRectDetector::depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        ROS_INFO("Received depth image with dimensions: %d x %d", depth_image.cols, depth_image.rows);
        depth_image_buffer_[msg->header.stamp] = depth_image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

// 获取深度图像
cv::Mat ImageRectDetector::getDepthImage(const ros::Time& timestamp) {
    auto closest_timestamp = std::min_element(depth_image_buffer_.begin(), depth_image_buffer_.end(),
        [&](const std::pair<ros::Time, cv::Mat>& a, const std::pair<ros::Time, cv::Mat>& b) {
            return (a.first - timestamp).toSec() < (b.first - timestamp).toSec();
        });

    if (closest_timestamp == depth_image_buffer_.end()) {
        ROS_WARN("No depth image found for the given timestamp.");
        return cv::Mat();
    }

    if (std::abs((closest_timestamp->first - timestamp).toSec()) > 0.05) {
        ROS_WARN("No suitable depth image found within 0.05 seconds.");
        return cv::Mat();
    }

    cv::Mat depth_image = closest_timestamp->second;
    depth_image_buffer_.erase(closest_timestamp);
    return depth_image;
}

// 处理图像
void ImageRectDetector::processImage(cv::Mat& image) {
    ROS_INFO("Processing image with dimensions: %d x %d", image.cols, image.rows);

    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    ROS_INFO("Gray image dimensions: %d x %d", gray_image.cols, gray_image.rows);

    cv::Mat thresh = thresholdImage(gray_image, 110, 255, cv::THRESH_BINARY_INV);
    ROS_INFO("Threshold image dimensions: %d x %d", thresh.cols, thresh.rows);

    auto rectangles_to_draw = filterRegions(thresh, 50, 2000);
    ROS_INFO("Detected %zu rectangles", rectangles_to_draw.size());

    for (const auto& [center, width, height] : rectangles_to_draw) {
        cv::rectangle(image, cv::Point(center.x - width / 2, center.y - height / 2),
                      cv::Point(center.x + width / 2, center.y + height / 2), cv::Scalar(0, 255, 0), 2);
        drawCross(image, center, cv::Scalar(255, 0, 0), 2, 10);
    }

    if (rectangles_to_draw.empty()) {
        ROS_INFO("No rectangles detected.");
    } else if (rectangles_to_draw.size() == 1) {
        auto [center, width, height] = rectangles_to_draw[0];
        std::vector<cv::Point> rectangle_corners = {
            cv::Point(center.x - width / 2, center.y - height / 2),
            cv::Point(center.x + width / 2, center.y - height / 2),
            cv::Point(center.x + width / 2, center.y + height / 2),
            cv::Point(center.x - width / 2, center.y + height / 2)
        };

        cv::Mat depth_image = getDepthImage(cv_ptr_->header.stamp);
        if (depth_image.empty()) {
            ROS_WARN("Depth image is empty. Skipping depth-related operations.");
        } else {
            cv::Vec3f normal = calculateRectangleNormal(depth_image, rectangle_corners);
            geometry_msgs::Vector3 normal_msg;
            normal_msg.x = normal[0];
            normal_msg.y = normal[1];
            normal_msg.z = normal[2];
            normal_pub_.publish(normal_msg);

            cv::Vec3f center_3d = imageToCameraCoordinates(depth_image, center, fx_, fy_, cx_, cy_);
            geometry_msgs::Point center_3d_msg;
            center_3d_msg.x = center_3d[0];
            center_3d_msg.y = center_3d[1];
            center_3d_msg.z = center_3d[2];
            center_pub_.publish(center_3d_msg);

            charging_station_pose_.position.x = center_3d[0];
            charging_station_pose_.position.y = center_3d[1];
            charging_station_pose_.position.z = center_3d[2];

            tf::Quaternion q;
            q.setRPY(0, 0, atan2(normal[1], normal[0]));
            charging_station_pose_.orientation.x = q.x();
            charging_station_pose_.orientation.y = q.y();
            charging_station_pose_.orientation.z = q.z();
            charging_station_pose_.orientation.w = q.w();
        }
    } else {
        ROS_INFO("Multiple rectangles detected. Waiting for single rectangle detection.");
    }

    ROS_INFO("Displaying processed image.");
    cv::imshow("Processed Image", image);
    cv::waitKey(1);
}

// 绘制十字
void ImageRectDetector::drawCross(cv::Mat& img, cv::Point center, cv::Scalar color, int thickness, int size) {
    cv::line(img, cv::Point(center.x - size, center.y), cv::Point(center.x + size, center.y), color, thickness);
    cv::line(img, cv::Point(center.x, center.y - size), cv::Point(center.x, center.y + size), color, thickness);
    cv::putText(img, "rect", cv::Point(center.x - 5, center.y + 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
}

// 图像阈值处理
cv::Mat ImageRectDetector::thresholdImage(const cv::Mat& image, double threshold_value, double max_value, int method) {
    cv::Mat thresh;
    cv::threshold(image, thresh, threshold_value, max_value, method);
    if (thresh.empty()) {
        ROS_ERROR("Threshold image is empty.");
    }
    return thresh;
}

// 计算轮廓周长
double ImageRectDetector::calculatePerimeter(const std::vector<cv::Point>& cnt) {
    return cv::arcLength(cnt, true);
}

// 判断是否为矩形
bool ImageRectDetector::isRectangleLike(const std::vector<cv::Point>& cnt, const std::vector<cv::Point>& approx, double min_aspect_ratio, double max_aspect_ratio, double min_fill_ratio) {
    if (approx.size() != 4) {
        return false;
    }

    cv::Rect bounding_rect = cv::boundingRect(approx);
    double aspect_ratio = static_cast<double>(bounding_rect.width) / bounding_rect.height;
    if (!(min_aspect_ratio <= aspect_ratio && aspect_ratio <= max_aspect_ratio)) {
        return false;
    }

    double area = cv::contourArea(cnt);
    double rect_area = bounding_rect.width * bounding_rect.height;
    double fill_ratio = area / rect_area;

    if (fill_ratio < min_fill_ratio) {
        return false;
    }

    return true;
}

// 过滤区域
std::vector<std::tuple<cv::Point, int, int>> ImageRectDetector::filterRegions(const cv::Mat& image, double min_area, double max_area) {
    std::vector<std::tuple<cv::Point, int, int>> rectangles_info;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        double perimeter = calculatePerimeter(cnt);

        if (area >= min_area && area <= max_area) {
            std::vector<cv::Point> approx;
            cv::approxPolyDP(cnt, approx, 0.04 * perimeter, true);
            if (isRectangleLike(cnt, approx)) {
                cv::Rect bounding_rect = cv::boundingRect(approx);
                cv::Point center(bounding_rect.x + bounding_rect.width / 2, bounding_rect.y + bounding_rect.height / 2);
                rectangles_info.emplace_back(center, bounding_rect.width, bounding_rect.height);
            }
        }
    }

    if (rectangles_info.empty()) {
        ROS_INFO("No rectangles detected.");
    }

    return rectangles_info;
}

// 计算矩形法线
cv::Vec3f ImageRectDetector::calculateRectangleNormal(const cv::Mat& depth_image, const std::vector<cv::Point>& rectangle_corners) {
    if (depth_image.empty()) {
        ROS_ERROR("Depth image is empty. Cannot calculate rectangle normal.");
        return cv::Vec3f(0, 0, 0);
    }

    std::vector<cv::Point3f> points_3d;
    for (const auto& corner : rectangle_corners) {
        if (corner.x < 0 || corner.x >= depth_image.cols || corner.y < 0 || corner.y >= depth_image.rows) {
            ROS_ERROR("Corner point out of depth image bounds: (%d, %d)", corner.x, corner.y);
            continue;
        }
        float depth = depth_image.at<float>(corner.y, corner.x);
        float Z = depth;
        float X = (corner.x - cx_) * Z / fx_;
        float Y = (corner.y - cy_) * Z / fy_;
        points_3d.emplace_back(X, Y, Z);
    }

    cv::Mat points_mat(points_3d);
    cv::Mat centroid = cv::Mat::zeros(3, 1, CV_32F);
    for (int i = 0; i < points_3d.size(); ++i) {
        centroid += points_mat.row(i).t();
    }
    centroid /= points_3d.size();

    cv::Mat covariance_matrix = cv::Mat::zeros(3, 3, CV_32F);
    for (int i = 0; i < points_3d.size(); ++i) {
        cv::Mat diff = points_mat.row(i).t() - centroid;
        covariance_matrix += diff * diff.t();
    }
    covariance_matrix /= points_3d.size();

    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(covariance_matrix, eigenvalues, eigenvectors);

    cv::Vec3f normal;
    cv::normalize(eigenvectors.row(2).t(), normal);
    return normal;
}

cv::Vec3f ImageRectDetector::imageToCameraCoordinates(const cv::Mat& depth_image, cv::Point image_point, double fx, double fy, double cx, double cy) {
    if (depth_image.empty()) {
        ROS_ERROR("Depth image is empty. Cannot convert image point to camera coordinates.");
        return cv::Vec3f(0, 0, 0);
    }

    if (image_point.x < 0 || image_point.x >= depth_image.cols || image_point.y < 0 || image_point.y >= depth_image.rows) {
        ROS_ERROR("Image point out of depth image bounds: (%d, %d)", image_point.x, image_point.y);
        return cv::Vec3f(0, 0, 0);
    }
    float depth = depth_image.at<float>(image_point.y, image_point.x);
    float Z = depth;
    float X = (image_point.x - cx) * Z / fx;
    float Y = (image_point.y - cy) * Z / fy;
    return cv::Vec3f(X, Y, Z);
}

// 获取充电站位姿
geometry_msgs::Pose ImageRectDetector::getChargingStationPose() {
    return charging_station_pose_;
}

// 获取充电站静态位姿
geometry_msgs::Pose ImageRectDetector::getChargingStationPoseStatic() {
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;

    tf::Quaternion q;
    q.setRPY(0, 0, 0.0);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

} // namespace charging_station