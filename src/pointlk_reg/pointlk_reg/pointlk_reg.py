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


class PointLK_reg(Node):

    def __init__(self):
        super().__init__('pointlk_reg')
        self.subs = []
        for i in range(2):
            sub = Subscriber(self,  PointCloud2, "/robot"+str(i+1)+"/mid360_PointCloud2")
            self.subs.append(sub)
        self.ts = ApproximateTimeSynchronizer(self.subs, queue_size=5, slop=0.1)
        self.ts.registerCallback(self.ts_callback)

        # Registration
        if not torch.cuda.is_available():
            self.device = 'cpu'
        else:
            self.device = 'cuda'
            print("Use GPU to registration")
        
        self.action = Action()
        self.model = self.action.create_model()
        # Load Pretrained model
        pretrained_path = '/home/wbc/results/ex1_pointlk_0915_model_best.pth'
        assert os.path.isfile(pretrained_path)
        self.model.load_state_dict(torch.load(pretrained_path, map_location='cpu'))
        self.model.to(self.device)


    def ts_callback(self, p0, p1):
        nd0 = self.data_dispose(p0)
        nd1 = self.data_dispose(p1)

        print(nd0.shape)
        data0 = torch.from_numpy(nd0)
        data1 = torch.from_numpy(nd1)
        g = self.action.do_estimate(data0, data1, self.model, self.device)
        print(g)
        self.get_logger().info('Registration finished')

    def data_dispose(self, p):
        nd = ros2_numpy.numpify(p)
        nd = nd.view((np.float32, len(nd.dtype.names)))
        nd = nd[~np.all(nd == 0, axis=1)]
        nd = np.expand_dims(nd, axis=0)

        return nd

class Action:
    def __init__(self):
        # PointNet
        self.dim_k = 1024 # Feature dim
        self.sym_fn = ptlk.pointnet.symfn_max

        # LK
        self.delta = 1.0e-2
        self.max_iter = 20
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