import rclpy
from rclpy.node import Node
import sys, os
from sensor_msgs.msg import PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
import ros2_numpy
sys.path.insert(0, '/home/wbc/Projects/lunar_MOAM/src/pointlk_reg/pointlk_reg/PointNetLK')
import ptlk
import numpy as np
import torch
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Point, Quaternion, Pose
from scipy.spatial.transform import Rotation as R

class PointLK_reg(Node):

    def __init__(self):
        super().__init__('pointlk_reg')
        self.subs = []

        self.sample_num = 1000 # sample points from original
        self.scan_range = 11 # scan_range

        self.declare_parameter('robot1', '0') # The first robot
        self.declare_parameter('robot2', '1') # The second robot
        self.declare_parameter('pretrained_path', '/home/wbc/results/ex1_pointlk_0915_model_best.pth') 

        robot1 = 'robot' + self.get_parameter('robot1').get_parameter_value()._string_value
        robot2 = 'robot' + self.get_parameter('robot2').get_parameter_value()._string_value
        odom_topic = '_'.join(['pointlk_reg', str(robot1), str(robot2)])
        self.odom_parent = '/'.join([robot1, 'odom'])
        self.odom_child = '/'.join([robot2, 'odom'])
        pretrained_path = self.get_parameter('pretrained_path').get_parameter_value()._string_value
        print(pretrained_path)

        self.subs.append(Subscriber(self,  PointCloud2, "/"+str(robot1)+"/mid360_PointCloud2"))
        self.subs.append(Subscriber(self,  PointCloud2, "/"+str(robot2)+"/mid360_PointCloud2"))

        self.ts = ApproximateTimeSynchronizer(self.subs, queue_size=5, slop=0.1)
        self.ts.registerCallback(self.ts_callback)
        self.g = None

        self.publisher_ = self.create_publisher(Odometry, odom_topic, 10)
        self.timer = self.create_timer(0.5, self.publish_odometry)

        # Registration
        if not torch.cuda.is_available():
            self.device = 'cpu'
        else:
            self.device = 'cuda'
            print("Use GPU to registration")
        
        self.action = Action()
        self.model = self.action.create_model()
        # Load Pretrained model
        assert os.path.isfile(pretrained_path)
        self.model.load_state_dict(torch.load(pretrained_path, map_location='cpu'))
        self.model.to(self.device)


    def ts_callback(self, p0, p1):
        nd0 = self.data_dispose(p0)
        nd1 = self.data_dispose(p1)

        print(nd0.shape)
        data0 = torch.from_numpy(nd0)
        data1 = torch.from_numpy(nd1)
        self.g = self.action.do_estimate(data0, data1, self.model, self.device)
        print(self.g)
        self.get_logger().info('Registration finished')

    def data_dispose(self, p):
        nd = ros2_numpy.numpify(p) # [n,3]
        print(nd.shape)
        nd = nd.view((np.float32, len(nd.dtype.names)))
        nd = nd[~np.all(nd == 0, axis=1)]
        print(nd.shape)
        norms = np.linalg.norm(nd, axis=1)
        indices = np.where(norms <= self.scan_range)[0]
        nd = nd[indices]
        print(nd.shape)
        nd = nd[nd[:, 2] > 0]
        print(nd.shape)
        indices = np.random.choice(nd.shape[0], self.sample_num, replace=True)
        nd = nd[indices]
        nd = np.expand_dims(nd, axis=0)

        return nd
    
    def publish_odometry(self):
        # Create the Odometry message
        if self.g == None:
            return
        odometry = Odometry()
        
        # Populate with example data
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = self.odom_parent
        odometry.child_frame_id = self.odom_child

        translation = self.g[0, :3, 3]
        rotation_matrix = self.g[0, :3, :3]
        # Convert to numpy arrays for easier manipulation
        trans = translation.cpu().detach()
        rot = rotation_matrix.cpu().detach()
        translation_np = trans.numpy()
        rotation_matrix_np = rot.numpy()
        print(translation_np[0])
        print(translation_np[1])
        print(translation_np[2])

        # Convert rotation matrix to quaternion
        r = R.from_matrix(rotation_matrix_np)
        quaternion = r.as_quat()

        # Create ROS2 message types
        pose_with_covariance = PoseWithCovariance()
        pose = Pose()
        pose.position = Point(x=float(translation_np[0]), y=float(translation_np[1]), z=float(translation_np[2]))
        pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        pose_with_covariance.pose = pose

        variances = [0.1, 0.1, 0.1, 0.01, 0.01, 0.01]  # Example variances
        covariance = [0.0] * 36  # Initialize a 6x6 matrix

        for i in range(6):
            covariance[i * 6 + i] = variances[i]  # Set the diagonal elements

        pose_with_covariance.covariance = covariance

        odometry.pose = pose_with_covariance
        
        # Publish the message
        self.publisher_.publish(odometry)
        self.get_logger().info('Published odometry')

class Action:
    def __init__(self):
        # PointNet
        self.dim_k = 1024 # Feature dim
        self.sym_fn = ptlk.pointnet.symfn_max

        # LK
        self.delta = 1.0e-2
        self.max_iter = 50
        self.xtol = 1.0e-7
        self.p0_zero_mean = True
        self.p1_zero_mean = True

    def create_model(self):
        ptnet = self.create_pointnet_features()
        print("Model Created...")
        return self.create_from_pointnet_features(ptnet)

    def create_pointnet_features(self):
        ptnet = ptlk.pointnet.PointNet_features(self.dim_k, use_tnet=False, sym_fn=self.sym_fn)
        return ptnet

    def create_from_pointnet_features(self, ptnet):
        return ptlk.pointlk.PointLK(ptnet, self.delta)


    def do_estimate(self, p0, p1, model, device):
        p0 = p0.to(device) # template (1, N, 3)
        p1 = p1.to(device) # source (1, M, 3)
        r = ptlk.pointlk.PointLK.do_forward(model, p0, p1, self.max_iter, self.xtol,\
                                            self.p0_zero_mean, self.p1_zero_mean)
        #r = model(p0, p1, self.max_iter)
        est_g = model.g # (1, 4, 4)

        return est_g
    
def main(args=None):
    rclpy.init(args=args)

    pointlk_reg = PointLK_reg()

    rclpy.spin(pointlk_reg)
    
    pointlk_reg.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()