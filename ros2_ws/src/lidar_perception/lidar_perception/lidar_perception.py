import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import time
import ros2_numpy as rnp

class LidarPerception(Node):
    def __init__(self):
        super().__init__('lidar_perception')
        self.latest_msg = None
        self.increment = 0
        self.total_points = None
        qos = QoSProfile(depth = 10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(PointCloud2,'/processed_points',qos)
        self.sub = self.create_subscription(PointCloud2,'/pandar_points',self.callback,qos)
        self.timer = self.create_timer(0.5,self.timer_callback)



    def callback(self, msg):
        self.latest_msg = msg
        self.increment += 1
        self.total_points = msg.height*msg.width
        self.get_logger().info(f"No. of points are {self.total_points}")
        

    def timer_callback(self):
        if self.latest_msg == None:
            self.get_logger().info("Waiting for pcl msg")
            return
        msg = self.latest_msg
        t1 = time.time()
        points = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        t2 = time.time()
        self.get_logger().info(f"shape of points array is {points.shape}")
        Distance = np.sqrt(points[:,0]**2 + points[:,1]**2 + points[:,2]**2)
        points = points[Distance < 20]
        points = list(map(list,points))

        
        t3 = time.time()
        self.processed_pcl = point_cloud2.create_cloud(msg.header,msg.fields,points)
        t4 = time.time()
        self.pub.publish(self.processed_pcl)
        t5 = time.time()
        self.get_logger().info(f"""time to read data from pcl is {t2 -t1} 
        time to filter pcl data is {t3-t2} 
        time to make new pcl msg from filtered data is {t4-t3} 
        time to publish is {t5-t4}""")
    
def main(args = None):
    rclpy.init(args = args)
    node = LidarPerception()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

