import sys
import threading

from gazebo_msgs.srv import GetEntityState
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class ModelStatePlot(Node):

    def __init__(self):
        super().__init__('mode_state_plot')
        # Multi threads
        self._lock = threading.Lock()
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.serv_cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.sub_cbg= rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # Requests
        self.req = GetEntityState.Request()
        # Parameters
        self.declare_parameter('robot_num', '3') # The first robot
        self.robot_num = self.get_parameter('robot_num').get_parameter_value()._string_value
        self.robot_num = int(self.robot_num)
        # Robot information
        self.robot_name = [('robot'+ str(i)) for i in range(self.robot_num)]

        # States from gazebo
        self.model_states = None # k * robot_num * 3 (x, y, yaw)
        self.new_states = None # robot_num * 3 (x, y, yaw)
        self.t = [] # Message Time
        self.relative_odom = None # Relative odom
        self.model_state_futures = [None for i in range(self.robot_num)]
        self.model_state_clients = [self.create_client(GetEntityState, '/gazebo/get_entity_state', callback_group=self.serv_cbg)  for i in range(self.robot_num)]

        # Publish relative odom
        self.rel_pubs = []
        for i in range(self.robot_num - 1):
            for j in range(i + 1, self.robot_num):
                topic_name = f'/true_odom_{self.robot_name[i]}_{self.robot_name[j]}'
                pub = self.create_publisher(Point, topic_name, 10)
                self.rel_pubs.append(pub)

        # Wait for client
        for i in range(self.robot_num):
            while not self.model_state_clients[i].wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            print("Service Ready")
            self.get_logger().info('service ready')
        # Timer
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.cbg)

        # Plot 
        self.fig, self.ax = plt.subplots()
        self.lines = []
        for i in range(self.robot_num):
            line, = self.ax.plot([], [], label=f'Robot {i}')
            self.lines.append(line)

    def send_request(self):
        self.req.reference_frame = 'world'
        for i in range(self.robot_num):
            self.req.name = self.robot_name[i]
            self.model_state_futures[i] = self.model_state_clients[i].call_async(self.req)
        return
    
    def publish_points(self):
        # Publish each point from the numpy array
        for i in range(self.relative_odom.shape[0]):
            point = Point()
            point.x = float(self.relative_odom[i, 0])
            point.y = float(self.relative_odom[i, 1])
            point.z = float(self.relative_odom[i, 2])
            
            # Publish to the corresponding topic
            self.rel_pubs[i].publish(point)
            # self.get_logger().info(f'Publishing {point} to point_topic_{i}')
    
    def timer_callback(self):  
        with self._lock:
            for i in range(self.robot_num):
                while not self.model_state_futures[i].done():
                    pass
                model_state = self.model_state_futures[i].result()
                if self.new_states is None:
                    self.new_states = np.zeros((self.robot_num, 3)) 
                self._decode(model_state, self.new_states[i])
                self.t.append(model_state.header.stamp.sec + model_state.header.stamp.nanosec * 1e-9)
            if self.model_states is not None:
                self.model_states = np.concatenate((self.model_states, np.expand_dims(self.new_states, axis=0)), axis=0)
            else:
                self.model_states = np.expand_dims(self.new_states, axis=0)
            self._compute_relative_odom()
            self.publish_points()
            self.send_request()


    # Extract information from msg
    def _decode(self, model_state, target_state):
        target_state[0] = model_state.state.pose.position.x
        target_state[1] = model_state.state.pose.position.y
        target_state[2] = model_state.state.pose.position.z # Need to change

    def _compute_relative_odom(self):
        """
        Compute the relative odometry between all robot pairs.
        
        Updates:
            self.relative_odom: [n * (n - 1) / 2, 3] relative odometry
        """
        pair_count = self.robot_num * (self.robot_num - 1) // 2
        self.relative_odom = np.zeros((pair_count, 3))
        
        pair_idx = 0
        print(f"Compute callback")
        for i in range(self.robot_num - 1):
            for j in range(i + 1, self.robot_num):
                self.relative_odom[pair_idx] = self.new_states[j] - self.new_states[i]
                pair_idx += 1



    def plt_func(self, _):  
        """Function for for adding data to axis.

        Args:
            _ : Dummy variable that is required for matplotlib animation.
        
        Returns:
            Axes object for matplotlib
        """
        # lock thread
        with self._lock:
            if self.model_states is not None:
                for i, line in enumerate(self.lines):
                    x = self.model_states[:, i, 0]
                    y = self.model_states[:, i, 1]
                    line.set_data(x, y)
                all_x = self.model_states[:, :, 0]
                all_y = self.model_states[:, :, 1]
                self.ax.set_xlim(np.min(all_x), np.max(all_x))
                self.ax.set_ylim(np.min(all_y), np.max(all_y))
                if not hasattr(self, "_legend_added"):
                    self.ax.legend()
                    self._legend_added = True
                
            return self.ax
        
    def _plt(self):
        """Function for initializing and showing matplotlib animation."""
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=1000)
        plt.show()


def main():
    rclpy.init()
    print("Init")
    model_plot_node = ModelStatePlot()
    model_plot_node.send_request()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(model_plot_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    model_plot_node._plt()



if __name__ == '__main__':
    main()
