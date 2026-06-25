from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

from .ugv01_http_v2 import UGV01Http


class UGV01HttpDriver(Node):
    def __init__(self) -> None:
        super().__init__('ugv01_http_driver')

        # Parameters
        # robot ip address from wiki
        self.declare_parameter('robot_ip', '192.168.4.1')
        # wheel_base = 187.39 - 44 = 143.39 mm (from product specifications)
        self.declare_parameter('wheel_base', 0.14339)	# meters
        self.declare_parameter('cmd_rate', 10.0)        # Hz
        self.declare_parameter('cmd_timeout', 3.0)  	# seconds

        ip_addr : str = (
            self.get_parameter('robot_ip').get_parameter_value().string_value
        )
        self.wheel_base : float = (
            self.get_parameter('wheel_base').get_parameter_value().double_value
		)
        self.cmd_rate : float = (
            self.get_parameter('cmd_rate').get_parameter_value().double_value
		)
        self.cmd_timeout : float = (
            self.get_parameter('cmd_timeout').get_parameter_value().double_value
		)

        self.hw_interface : UGV01Http = UGV01Http(ip_addr)

        # Start last command as zero (no movement)
        self.last_cmd : Twist = Twist()
        # Initialize last_cmd_time to "now" so the robot doesn't stop immediately at startup
        self.last_cmd_time : Time = self.get_clock().now()

        # Subscribe to /cmd_vel
        # qos = 10 -> incoming queue size
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Timer to send commands at fixed rate
        self.timer = self.create_timer(1.0 / self.cmd_rate, self.timer_callback)

        self.get_logger().info(f"UGV01 HTTP driver started, IP={ip_addr}")


    def cmd_vel_callback(self, msg : Twist) -> None:
        """
        Called whenever a new /cmd_vel message is received.
        Just stores the latest command and updates the timestamp;
        actual sending is done in timer_callback.
        """
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        

    def timer_callback(self) -> None:
        """
        Called periodically at cmd_rate Hz.
        Converts the latest /cmd_vel into left/right speeds and sends them via HTTP.
        If more than cmd_timeout seconds have passed since the last command was received,
        stops the robot.
        """
        now : Time = self.get_clock().now()
        dt_since_cmd : float = (now - self.last_cmd_time).nanoseconds * 1e-9
        
        if dt_since_cmd > self.cmd_timeout:
            # No recent command: stop the robot
            v_l = 0.0
            v_r = 0.0
        else:
            v : float = self.last_cmd.linear.x
            w : float = self.last_cmd.angular.z
            L : float = self.wheel_base

            # Differential drive kinematics
            v_l : float = v - 0.5 * L * w
            v_r : float = v + 0.5 * L * w

            # Clamp speeds to the 0.5 m/s limit
            max_speed : float = 0.5
            v_l = max(-max_speed, min(max_speed, v_l))
            v_r = max(-max_speed, min(max_speed, v_r))

        ok, info = self.hw_interface.set_wheel_speeds(v_l, v_r)
        
        if not ok and info is not None:
            self.get_logger().warn(
                f"HTTP error (the command may have reached the robot and been executed anyway): {info}"
            )
        
        #self.last_cmd = Twist()


def main(args : list[str] | None = None) -> None:
    """
    Initializes ROS, creates the node, spins (handles callbacks), then shuts down.
    """
    rclpy.init(args=args)
    node = UGV01HttpDriver()
    
    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("Ctrl+C received, shutting down http driver")
        
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
