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
        # Model state client
        self.cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetEntityState.Request()
        self.robot_name = 'robot1'
        self.model_state = None
        self.model_state_future = None
        # Timer
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.cbg)
        # Plot 
        self.fig, self.ax = plt.subplots()
        self.t = [0]
        self.x = [0]

        

    def send_request(self, robot_name):
        self.req.name = robot_name
        self.req.reference_frame = 'world'
        return self.cli.call_async(self.req)
    
    def timer_callback(self):  
        with self._lock:
            if self.model_state_future.done():
                self.model_state = self.model_state_future.result()
                self.t.append(self.model_state.header.stamp.sec)
                self.x.append(self.model_state.state.pose.position.x)
            self.model_state_future = self.send_request(self.robot_name)

    def plt_func(self, _):
        """Function for for adding data to axis.

        Args:
            _ : Dummy variable that is required for matplotlib animation.
        
        Returns:
            Axes object for matplotlib
        """
        # lock thread
        with self._lock:
            t = np.array(self.t)
            x = np.array(self.x)
            self.ax.plot(t, x)

            return self.ax
        
    def _plt(self):
        """Function for initializing and showing matplotlib animation."""
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=1000)
        plt.show()


def main():
    rclpy.init()
    model_plot_node = ModelStatePlot()
    model_plot_node.model_state_future = model_plot_node.send_request(model_plot_node.robot_name)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(model_plot_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    model_plot_node._plt()



if __name__ == '__main__':
    main()
