import rclpy
from rclpy.node import Node
from my_msgs.msg import State
import numpy as np

class OptimalIntersectionPublisher(Node):

    def __init__(self):
        super().__init__('optimal_intersection_publisher')
        self.publisher_ = self.create_publisher(State, 'optimal_intersection', 10)
        self.declare_parameter("predicted_target_states", [])
        self.declare_parameter("interceptor_state", [])
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.publish_optimal_intersection)

    def publish_optimal_intersection(self):
        predicted_target_states = self.get_parameter("predicted_target_states").value
        interceptor_state = self.get_parameter("interceptor_state").value

        optimal_state = self.find_optimal_intersection(predicted_target_states, interceptor_state)
        
        msg = State()
        msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.ax, msg.ay, msg.az = optimal_state
        
        self.publisher_.publish(msg)

    # Your find_optimal_intersection method goes here ...

def main(args=None):
    rclpy.init(args=args)
    optimal_intersection_publisher = OptimalIntersectionPublisher()
    rclpy.spin(optimal_intersection_publisher)
    optimal_intersection_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
