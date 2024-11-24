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

class RelativeOdomPlot(Node):

    def __init__(self):
        super().__init__('relative_odom_plot')

        # Multi-threading lock
        self._lock = threading.Lock()
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.sub_cbg= rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.tm_cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # Parameters
        self.declare_parameter('robot_num', '3')
        self.robot_num = int(self.get_parameter('robot_num').value)

        # Robot names
        self.robot_name = [f'robot{i}' for i in range(self.robot_num)]

        # Create a list to hold subscribers
        self.true_rel_subs = []

        # Create a subscriber for true relative position
        pair_count = self.robot_num * (self.robot_num - 1) // 2
        for i in range(self.robot_num - 1):
            for j in range(i + 1, self.robot_num):
                topic_name = f'/true_odom_{self.robot_name[i]}_{self.robot_name[j]}'
                sub = self.create_subscription(
                Point, 
                topic_name, 
                lambda msg, i=i, j=j: self.point_callback(msg, i, j), 
                10,
                callback_group=self.sub_cbg
            )
                self.true_rel_subs.append(sub)

        # Fused odometry subscriptions
        self.fused_subs = {}
        sub_idx = 0
        for i in range(self.robot_num - 1):
            for j in range(i + 1, self.robot_num):
                print(f"i : {i},j: {j}, idx:{sub_idx}")
                topic_name = f'/pointlk_reg_{self.robot_name[i]}_{self.robot_name[j]}'
                self.fused_subs[topic_name] = self.create_subscription(
                    Odometry,
                    topic_name,
                    lambda msg: self.fused_callback(msg, i, j),
                    10,
                    callback_group=self.sub_cbg
                )
                

        # Variables to store states
        # relative_odom: [n * (n - 1) / 2, 3] (dx, dy, dz) - relative positions between robot pairs
        self.fused_odom = None
        self.relative_odom = None
        self.relative_odom_history = [[[], []] for _ in range(pair_count)]
        self.fused_odom_history = [[[], []] for _ in range(pair_count)]

        # Plot setup
        self.fig, self.ax = plt.subplots()
        self.lines = []
        self.fused_lines = []
        pair_count = self.robot_num * (self.robot_num - 1) // 2
        for i in range(pair_count):
            line, = self.ax.plot([], [], label=f'Relative Odom Pair {i}')
            self.lines.append(line)
            line, = self.ax.plot([], [], linestyle='--', label=f'Fused Odom Pair {i}')
            self.fused_lines.append(line)
            
    def fused_callback(self, msg, i, j):   
        pair_count = self.robot_num * (self.robot_num - 1) // 2
        self.fused_odom = np.zeros((pair_count, 3))       
        idx = (i * self.robot_num - (i * (i + 1)) // 2 + j - i - 1)
        # Update self.fused_odom with the new odometry data
        self.fused_odom_history[idx][0].append(msg.pose.pose.position.x)
        self.fused_odom_history[idx][1].append(msg.pose.pose.position.y)

    
    def point_callback(self, msg, i, j):
        # Store the received data in the numpy array
        pair_count = self.robot_num * (self.robot_num - 1) // 2
        self.relative_odom = np.zeros((pair_count, 3), dtype=np.float64)
        index = (i * self.robot_num - (i * (i + 1)) // 2 + j - i - 1)
        self.relative_odom[index, 0] = msg.x
        self.relative_odom[index, 1] = msg.y
        self.relative_odom[index, 2] = msg.z
        
        # # Log the received message
        # self.get_logger().info(f'Received {msg} on point_topic_{index}')
        # self.get_logger().info(f'Updated array:\n{self.relative_odom}')

    def plot_update(self, _):
        """Update the plot with the historical data for both relative and fused odometry."""
        with self._lock:
            if self.relative_odom is not None:
                for i, (rel_line, fused_line) in enumerate(zip(self.lines, self.fused_lines)):
                    # Append the latest data for relative odometry
                    rel_x, rel_y = self.relative_odom_history[i]
                    rel_x.append(self.relative_odom[i, 0])
                    rel_y.append(self.relative_odom[i, 1])
                    rel_line.set_data(rel_x, rel_y)
                    
                    
                    # Append the latest data for fused odometry if available
                    if self.fused_odom_history is not None:
                        fused_line.set_data(self.fused_odom_history[i][0], self.fused_odom_history[i][1])
                       
                
                # Adjust plot limits dynamically
                # all_x = [x for pair in self.relative_odom_history for x in pair[0]] + \
                #         [x for pair in self.fused_odom_history for x in pair[0]]
                # all_y = [y for pair in self.relative_odom_history for y in pair[1]] + \
                #         [y for pair in self.fused_odom_history for y in pair[1]]
                
                self.ax.set_xlim( - 1,  + 1)
                self.ax.set_ylim( - 1,  + 1)
                
                if not hasattr(self, '_legend_added'):
                    self.ax.legend()
                    self._legend_added = True

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
