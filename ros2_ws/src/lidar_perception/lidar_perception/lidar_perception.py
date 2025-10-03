import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import time

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



    def callback(self, msg):
        self.latest_msg = msg
        self.increment += 1
        self.total_points = msg.height*msg.width
        self.get_logger().info(f"No. of points are {self.total_points}")
        t1 = time.time()
        points = point_cloud2.read_points(msg, field_names=("x","y","z","intensity","return_type","channel","azimuth","elevation","distance","time_stamp"), skip_nans = True)
        t2 = time.time()
        for p in points:
            x = p[0]
            y = p[1]
            z = p[2]
            if np.sqrt(x*x + y*y + z*z) < 20:
                list.append(p)

        
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

