import sys
import threading

from gazebo_msgs.srv import GetEntityState
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np

class ModelStatePlot(Node):

    def __init__(self):
        super().__init__('mode_state_plot')
        # Multi threads
        self._lock = threading.Lock()
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        # Requests
        self.req = GetEntityState.Request()
        # Parameters
        self.declare_parameter('robot_num', '3') # The first robot
        self.robot_num = self.get_parameter('robot_num').get_parameter_value()._string_value
        self.robot_num = int(self.robot_num)
        # Robot information
        self.robot_name = [('robot'+ str(i)) for i in range(self.robot_num)]
        self.model_states = None # k * robot_num * 3 (x, y, yaw)
        self.new_state = np.zeros((self.robot_num, 3)) # robot_num * 3 (x, y, yaw)
        self.t = [] # Message Time
        self.model_state_futures = [None for i in range(self.robot_num)]
        self.model_state_clients = [self.create_client(GetEntityState, '/gazebo/get_entity_state')  for i in range(self.robot_num)]
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

    def send_request(self):
        self.req.reference_frame = 'world'
        for i in range(self.robot_num):
            self.req.name = self.robot_name[i]
            self.model_state_futures = self.model_state_clients[i].call_async(self.req)
        return
    
    def timer_callback(self):  
        with self._lock:
            for i in range(self.robot_num):
                if self.model_state_futures[i].done():
                    print("Position get")
                    model_state = self.model_state_future.result()
                    self._decode(model_state, self.new_state[i])
                    self.t.append(self.model_state.header.stamp.sec + self.model_state.header.stamp.nanosec * 1e-9)
            if self.model_states:
                self.model_states = np.concatenate((self.model_states, np.expand_dims(self.new_state, axis=0)), axis=0)
            else:
                self.model_states = np.expand_dims(self.new_state, axis=0)
            self.send_request()

    # Extract information from msg
    def _decode(self, model_state, target_state):
        target_state[0] = model_state.state.pose.position.x
        target_state[1] = model_state.state.pose.position.y
        target_state[2] = model_state.state.pose.position.z # Need to change


    def plt_func(self, _):  
        """Function for for adding data to axis.

        Args:
            _ : Dummy variable that is required for matplotlib animation.
        
        Returns:
            Axes object for matplotlib
        """
        # lock thread
        with self._lock:
            if self.model_states:
                x = self.model_states[:, 0, 0]
                y = self.model_states[:, 0, 1]
                self.ax.plot(x, y)

            return self.ax
        
    def _plt(self):
        """Function for initializing and showing matplotlib animation."""
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=1000)
        plt.show()


def main():
    rclpy.init()
    print("Init")
    model_plot_node = ModelStatePlot()
    print("2")
    # model_plot_node.send_request()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(model_plot_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    model_plot_node._plt()



if __name__ == '__main__':
    main()
