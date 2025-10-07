#include "lidar_perception_cpp/lidar_perception.hpp"

LidarPerceptionCpp::LidarPerceptionCpp():Node("lidar_perception_node")
{   
    rclcpp::QoS qos(rclcpp::KeepLast(10));

    // Optionally set reliability
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/topic1",qos);
    sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/pandar_points",qos,std::bind(&LidarPerceptionCpp::topic_callback,this,std::placeholders::_1));

}


void LidarPerceptionCpp::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr /*msg*/){
    RCLCPP_INFO(this->get_logger(),"hi");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<LidarPerceptionCpp>());
    rclcpp::shutdown();
    return 0;
}