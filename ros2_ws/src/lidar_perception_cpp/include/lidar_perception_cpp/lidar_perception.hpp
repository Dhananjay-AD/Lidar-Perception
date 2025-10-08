#ifndef LIDAR_PERCEPTION_HPP_
#define LIDAR_PERCEPTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <memory>
#include "rclcpp/qos.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <cmath>


class LidarPerceptionCpp : public rclcpp::Node
{
    public:
        LidarPerceptionCpp(); //constructor
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;  

    private:
        void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        //output message from callback/published message
        sensor_msgs::msg::PointCloud2::SharedPtr output_msg;

};


#endif // LIDAR_PERCEPTION_HPP_