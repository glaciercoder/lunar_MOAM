import sys
import threading

from gazebo_msgs.srv import GetEntityState
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np

class RelativeOdomPlot(Node):

    def __init__(self):
        super().__init__('relative_odom_plot')

        # Multi-threading lock
        self._lock = threading.Lock()
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.serv_cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.sub_cbg= rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # Parameters
        self.declare_parameter('robot_num', '3')
        self.robot_num = int(self.get_parameter('robot_num').value)

        # Robot names
        self.robot_name = [f'robot{i}' for i in range(self.robot_num)]

        # Gazebo service clients
        self.req = GetEntityState.Request()
        self.req.reference_frame = 'world'
        self.gazebo_clients = [self.create_client(GetEntityState, '/gazebo/get_entity_state', callback_group=self.serv_cbg) for _ in range(self.robot_num)]

        # Ensure services are ready
        for client in self.gazebo_clients:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for service...')

        # Variables to store states
        # current_states: [robot_num, 3] (x, y, z) - positions of robots
        self.current_states = np.zeros((self.robot_num, 3))

        # relative_odom: [n * (n - 1) / 2, 3] (dx, dy, dz) - relative positions between robot pairs
        self.relative_odom = None

        # Timer for requesting states
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Plot setup
        self.fig, self.ax = plt.subplots()
        self.lines = []
        pair_count = self.robot_num * (self.robot_num - 1) // 2
        for i in range(pair_count):
            line, = self.ax.plot([], [], label=f'Relative Odom Pair {i}')
            self.lines.append(line)

    def timer_callback(self):
        """Callback for the timer to get robot states and compute relative odometry."""
        with self._lock:
            # Request states for all robots
            for i, client in enumerate(self.gazebo_clients):
                self.req.name = self.robot_name[i]
                future = client.call_async(self.req)
                rclpy.spin_until_future_complete(self, future)
                result = future.result()

                self._decode(result, self.current_states[i])

            # Compute relative odometry
            self._compute_relative_odom()

    def _decode(self, model_state, target_state):
        """
        Decode the pose from model_state into target_state.
        
        Args:
            model_state: Gazebo state result
            target_state: [3] (x, y, z) to store the decoded pose
        """
        target_state[0] = model_state.state.pose.position.x
        target_state[1] = model_state.state.pose.position.y
        target_state[2] = model_state.state.pose.position.z

    def _compute_relative_odom(self):
        """
        Compute the relative odometry between all robot pairs.
        
        Updates:
            self.relative_odom: [n * (n - 1) / 2, 3] relative odometry
        """
        pair_count = self.robot_num * (self.robot_num - 1) // 2
        self.relative_odom = np.zeros((pair_count, 3))
        
        pair_idx = 0
        for i in range(self.robot_num):
            for j in range(i + 1, self.robot_num):
                self.relative_odom[pair_idx] = self.current_states[j] - self.current_states[i]
                pair_idx += 1

    def plot_update(self, _):
        """Update the plot with the relative odometry data."""
        with self._lock:
            if self.relative_odom is not None:
                for i, line in enumerate(self.lines):
                    x = self.relative_odom[:i, 0]
                    y = self.relative_odom[:i, 1]
                    line.set_data(x, y)
                all_x = self.relative_odom[:, 0]
                all_y = self.relative_odom[:, 1]
                self.ax.set_xlim(np.min(all_x) - 1, np.max(all_x) + 1)
                self.ax.set_ylim(np.min(all_y) - 1, np.max(all_y) + 1)
                if not hasattr(self, '_legend_added'):
                    self.ax.legend()
                    self._legend_added = True
        return self.ax

    def _plt(self):
        """Function to show matplotlib animation."""
        self.ani = anim.FuncAnimation(self.fig, self.plot_update, interval=1000)
        plt.show()

def main():
    rclpy.init()
    relative_odom_plot_node = RelativeOdomPlot()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(relative_odom_plot_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    relative_odom_plot_node._plt()

if __name__ == '__main__':
    main()
