#include "lidar_perception_cpp/lidar_perception.hpp"

LidarPerceptionCpp::LidarPerceptionCpp():Node("lidar_perception_node")
{   
    rclcpp::QoS qos(rclcpp::KeepLast(10));

    // Optionally set reliability
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed_points",qos);
    sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/pandar_points",qos,std::bind(&LidarPerceptionCpp::topic_callback,this,std::placeholders::_1));

}


void LidarPerceptionCpp::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    RCLCPP_INFO(this->get_logger(), "Original cloud size: %zu", cloud->points.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    for(const auto &point : cloud->points){
        float distance = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
        if (distance <= 20){
            cloud_filtered->points.push_back(point);
        }
        
    }
    cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;  // unorganized cloud
        cloud_filtered->is_dense = true;
        // Create ROS2 message
        sensor_msgs::msg::PointCloud2 output_msg;

        // Convert PCL cloud to ROS2 PointCloud2
        pcl::toROSMsg(*cloud_filtered, output_msg);

        // Keep the same header (frame_id and timestamp)
        output_msg.header = msg->header;
        pub->publish(output_msg);

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<LidarPerceptionCpp>());
    rclcpp::shutdown();
    return 0;
}