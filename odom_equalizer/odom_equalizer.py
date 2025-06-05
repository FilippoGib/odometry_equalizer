import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import gz.transport13   as ign_transport
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from nav_msgs.msg import Odometry
import sys


class OdomEqualizer(Node):
    """
    This node subscribes to /sim/Odometry (ROS2) and forwards the pose
    to Ignition Transport via the /world/sim/set_pose service.
    """

    def __init__(self):
        super().__init__('odom_equalizer')

        # QoS: reliable, (optionally) transient-local if latching is needed
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE  # or TRANSIENT_LOCAL if desired
        )

        # Subscribe to ROS2 Odometry topic
        self.subscriber = self.create_subscription(
            Odometry,
            '/sim/Odometry',
            self._odom_callback,
            qos_profile
        )

        # Ignition Transport client (named differently from the ROS2 node)
        self.ign_node = ign_transport.Node()

        # Keep the latest Odometry message; we'll push it to Ignition at 10 Hz
        self._latest_odom = None

        # Timer to call Ignition service at up to 10 Hz
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self._timer_send_pose)

        self.get_logger().info('OdomEqualizer node has been started')

    def _odom_callback(self, msg: Odometry):
        """
        Receiver callback for ROS2 Odometry. Store the latest message for the timer.
        """
        # Log that it received a new Odometry message
        self.get_logger().info(f'Received Odometry: {msg.pose.pose.position.x}, '
                                f'{msg.pose.pose.position.y}, {msg.pose.pose.position.z}')
        self._latest_odom = msg
        

    def _timer_send_pose(self):
        """
        Timer callback (10 Hz). If we've received at least one Odometry,
        convert it to an Ignition Pose and send the service request.
        """
        # log the timer event        
        if self._latest_odom is None:
            self.get_logger().info('No Odometry data received yet, skipping Ignition pose update')
            return
        
        self.get_logger().info('Timer triggered to send pose to Ignition')

        odom = self._latest_odom

        # Build the Ignition Pose message
        pose_msg = Pose()
        pose_msg.name = 'vehicle_blue'
        pose_msg.position.x = odom.pose.pose.position.x
        pose_msg.position.y = odom.pose.pose.position.y
        pose_msg.position.z = odom.pose.pose.position.z
        pose_msg.orientation.x = odom.pose.pose.orientation.x
        pose_msg.orientation.y = odom.pose.pose.orientation.y
        pose_msg.orientation.z = odom.pose.pose.orientation.z
        pose_msg.orientation.w = odom.pose.pose.orientation.w

        # Send the service request (blocking up to 2000 ms)

        try:
            outcome, body  = self.ign_node.request(
                service='/world/sim/set_pose',  # make sure the world name matches your simulator
                request=pose_msg,
                request_type=Pose,
                response_type=Boolean,
                timeout=2000  # in milliseconds
            )
        except Exception as e:
            self.get_logger().error(f"Ignition request threw exception: {e}")
            return

        # If the request couldn't even be delivered
        if outcome is False:
            self.get_logger().error('Failed to dispatch set_pose request to Ignition')
            # self.get_logger().error(f'The body of the response was: {body}')
            return
    
        # If we reach here, the pose was successfully set
        elif outcome is True:
            self.get_logger().info(
                f'The requeste returned outcome: {outcome}'
                f'Successfully set pose at '
                f'(x={pose_msg.position.x:.2f}, y={pose_msg.position.y:.2f}, z={pose_msg.position.z:.2f})'
            )

    def destroy_node(self):
        """
        Override destroy_node to cleanly close the Ignition client as well.
        """
        super().destroy_node()
        try:
            self.ign_node.close()
            self.get_logger().info('Closed Ignition Transport node')
        except AttributeError:
            # If `close` is not available, ignore
            pass


def main(args=None):
    rclpy.init(args=sys.argv)
    node = OdomEqualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
