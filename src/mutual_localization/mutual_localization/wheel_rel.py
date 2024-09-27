import rclpy
from rclpy.node import Node
import sys, os
from sensor_msgs.msg import PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovariance, Point, Quaternion, Twist
from scipy.spatial.transform import Rotation as R

class WheelRel(Node):

    def __init__(self):
        super().__init__('wheel_rel')
        self.subs = []

        self.declare_parameter('robot1', '0') # The first robot
        self.declare_parameter('robot2', '1') # The second robot

        robot1 = 'robot' + self.get_parameter('robot1').get_parameter_value()._string_value
        robot2 = 'robot' + self.get_parameter('robot2').get_parameter_value()._string_value
        odom_topic = '_'.join(['wheel_rel_odom', str(robot1), str(robot2)])
        self.odom_parent = '/'.join([robot1, 'odom'])
        self.odom_child = '/'.join([robot2, 'odom'])

        self.subs.append(Subscriber(self,  Odometry, '/'.join([robot1, 'odom'])))
        self.subs.append(Subscriber(self,  Odometry, '/'.join([robot2, 'odom'])))
    

        self.ts = ApproximateTimeSynchronizer(self.subs, queue_size=5, slop=0.5)
        self.ts.registerCallback(self.ts_callback)

        self.publisher_ = self.create_publisher(Odometry, odom_topic, 10)
        self.odometry = Odometry()
        print("Wheel Rel "+ str(robot1) + str(robot2) + " setup!")


    def ts_callback(self, odom0, odom1):
        self.odometry.header.stamp = self.get_clock().now().to_msg()
        self.odometry.header.frame_id = self.odom_parent
        self.odometry.child_frame_id = self.odom_child

        twist_with_covariance = TwistWithCovariance()
        twist = Twist()

        twist.linear.x = odom0.twist.twist.linear.x - odom1.twist.twist.linear.x
        twist.linear.y = odom0.twist.twist.linear.y - odom1.twist.twist.linear.y
        twist.linear.z = odom0.twist.twist.linear.z - odom1.twist.twist.linear.z
        twist.angular.z = odom0.twist.twist.angular.z - odom1.twist.twist.angular.z

        twist_with_covariance.covariance = odom0.twist.covariance + odom1.twist.covariance
        twist_with_covariance.twist = twist

        self.odometry.twist = twist_with_covariance
        self.publisher_.publish(self.odometry)
       

    
def main(args=None):
    rclpy.init(args=args)

    wheel_rel = WheelRel()

    rclpy.spin(wheel_rel)
    
    wheel_rel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()