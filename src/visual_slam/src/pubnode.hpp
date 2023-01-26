#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("main"), count_(0)
    {
      pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_topic_2", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
    void SetPoints(std::vector<cv::Mat> points) {
      created_points_ = points;
    }
  private:
    void timer_callback()
    {
    //   auto message = std_msgs::msg::String();
    //   message.data = "Hello, world! " + std::to_string(count_++);
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //   publisher_->publish(message);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->width = created_points_.size();
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);
        for(int i = 0; i < created_points_.size(); i++) {
            cv::Mat_<float> point_32;
            created_points_[i].convertTo(point_32, CV_32F);
            cloud->points[i].x = point_32.at<float>(0);
            cloud->points[i].y = point_32.at<float>(1);
            cloud->points[i].z = point_32.at<float>(2);
        }
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        pointcloud_publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    size_t count_;
    std::vector<cv::Mat> created_points_;
};


