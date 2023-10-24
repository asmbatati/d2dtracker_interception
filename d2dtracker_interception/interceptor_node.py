import rclpy
from rclpy.node import Node
from interceptor_node.msg import InterceptorInput, Trajectory, Point

class InterceptorNode(Node):

    def __init__(self):
        super().__init__('interceptor_node')
        self.publisher_ = self.create_publisher(Trajectory, 'interceptor_trajectory', 10)
        self.subscription = self.create_subscription(
            InterceptorInput,
            'interceptor_input',
            self.listener_callback,
            10)
        self.subscription  

    def listener_callback(self, msg):
        traj_data = interceptor_trajectory(
            msg.xtgt, msg.ytgt, msg.ztgt, msg.xint, msg.yint, msg.vint,
            msg.vmax, msg.amax, msg.φt, msg.dt, msg.mpc_horizon_length
        )
        
        traj_msg = Trajectory()
        for data in traj_data:
            point = Point()
            point.x, point.y, point.z = data
            traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = InterceptorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
