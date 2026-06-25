from __future__ import annotations

import json
import queue
import threading
import itertools
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, Float64, String
from std_srvs.srv import Trigger

from .http_connection_v2 import UGV01Http

try:
    from custom_interfaces.srv import SendGenericJson
except ImportError:  # pragma: no cover
    SendGenericJson = None


@dataclass
class RobotJob:
    category: str
    payload: Any = None
    reply_queue: Optional[queue.Queue] = None


class UGV01HttpDriver(Node):
    def __init__(self) -> None:
        super().__init__('ugv01_http_driver')

        # Parameters
        # robot ip address from wiki
        self.declare_parameter('robot_ip', '192.168.4.1')
        # wheel_base = 187.39 - 44 = 143.39 mm (from product specifications)
        self.declare_parameter('wheel_base', 0.14339)	# meters
        self.declare_parameter('cmd_vel_rate', 10.0)	# Hz
        self.declare_parameter('cmd_vel_timeout', 3.0)	# seconds
        self.declare_parameter('imu_rate', 1.0)			# Hz
        self.declare_parameter('generic_json_service_timeout', 1.0)	 # seconds

        self.ip_addr: str = (
            self.get_parameter('robot_ip').get_parameter_value().string_value
        )
        self.wheel_base: float = (
            self.get_parameter('wheel_base').get_parameter_value().double_value
        )
        self.cmd_vel_rate: float = (
            self.get_parameter('cmd_vel_rate').get_parameter_value().double_value
        )
        self.cmd_vel_timeout: float = (
            self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        )
        self.imu_rate: float = (
            self.get_parameter('imu_rate').get_parameter_value().double_value
        )
        self.generic_json_service_timeout: float = (
            self.get_parameter('generic_json_service_timeout').get_parameter_value().double_value
        )

        # HTTP connection to the robot
        self.hw_interface: UGV01Http = UGV01Http(self.ip_addr)

        # Start last velocity command as zero (no movement)
        self.last_cmd_vel: Twist = Twist()
        # Initialize last_cmd_vel_time to "now" so the robot doesn't stop immediately at startup
        self.last_cmd_vel_time: Time = self.get_clock().now()

        # Subscriber to /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            msg_type=Twist, topic='cmd_vel', callback=self.cmd_vel_callback, qos_profile=10
        )

        # Publishers
        self.voltage_pub = self.create_publisher(
            msg_type=Float32, topic='voltage', qos_profile=10
        )
        self.orientation_pub = self.create_publisher(
            msg_type=Vector3, topic='orientation', qos_profile=10 
        )
        self.magn_pub = self.create_publisher(
            msg_type=Vector3, topic='magn', qos_profile=10
        )
        self.ip_pub = self.create_publisher(
            msg_type = String, topic='ip', qos_profile=10
        )
        self.mac_pub = self.create_publisher(
            msg_type = String, topic='mac', qos_profile=10
        )
        self.rssi_pub = self.create_publisher(
            msg_type=Float32, topic='rssi', qos_profile=10
        )
        self.temp_pub = self.create_publisher(
            msg_type=Float32, topic='temp', qos_profile=10
        )
        self.acce_pub = self.create_publisher(
            msg_type=Vector3, topic='acce', qos_profile=10
        )
        self.gyro_pub = self.create_publisher(
            msg_type=Vector3, topic='gyro', qos_profile=10
        )
        self.shunt_mV_pub = self.create_publisher(
            msg_type=Float64, topic='shunt_mV', qos_profile=10
        )
        self.load_V_pub = self.create_publisher(
            msg_type=Float64, topic='load_V', qos_profile=10
        )
        self.bus_V_pub = self.create_publisher(
            msg_type=Float64, topic='bus_V', qos_profile=10
        )
        self.current_mA_pub = self.create_publisher(
            msg_type=Float64, topic='current_mA', qos_profile=10
        )
        self.power_mW_pub = self.create_publisher(
            msg_type=Float64, topic='power_mW', qos_profile=10
        )

        # Priority queue support
        self.job_queue: queue.PriorityQueue[tuple[int, int, RobotJob]] = queue.PriorityQueue()
        self.job_counter = itertools.count()
        self.stop_worker = threading.Event()
        self.queue_state_lock = threading.Lock()
        self.cmd_vel_job_pending = False
        self.imu_job_pending = False
        
        if SendGenericJson is not None:
            self.generic_json_srv = self.create_service(
                SendGenericJson,
                'send_generic_json',
                self.send_generic_json_callback,
            )
            self.get_logger().info('Service /send_generic_json created')
        else:
            self.generic_json_srv = self.create_service(
                Trigger,
                'send_generic_json_unavailable',
                self.send_generic_json_unavailable_callback,
            )
            self.get_logger().warn(
                'Custom service type SendGenericJson not found. '
                'Created /send_generic_json_unavailable as a placeholder instead.'
            )

        # Worker thread: takes jobs from the priority queue and sends them to robot
        self.worker_thread = threading.Thread(target=self.http_worker, daemon=True)
        self.worker_thread.start()

        # Timers to send velocity commands and process sensor readings at fixed rates
        self.cmd_vel_timer = self.create_timer(
            timer_period_sec=(1.0 / self.cmd_vel_rate),
            callback=self.cmd_vel_timer_callback
        )
        self.imu_timer = self.create_timer(
            timer_period_sec=(1.0 / self.imu_rate),
            callback=self.imu_timer_callback
        )

        self.get_logger().info(f"UGV01 HTTP driver started, IP={self.ip_addr}")
        
    
    # ------------- #
    # ROS callbacks #
    # ------------- #

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Called whenever a new /cmd_vel message is received.
        Stores the latest velocity command and updates the timestamp.
        """
        
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()
        

    def cmd_vel_timer_callback(self) -> None:
        """
        Called periodically at cmd_vel_rate Hz to add a velocity job to the queue.
        Only adds it if there isn't one there already.
        """
        
        with self.queue_state_lock:
            if self.cmd_vel_job_pending:
                return
            self.cmd_vel_job_pending = True

        self.enqueue_job(priority=0, category="cmd_vel")
        
            
    def imu_timer_callback(self) -> None:
        """
        Called periodically at imu_rate Hz to add a read imu job to the queue.
        Only adds it if there isn't one there already.
        """
        
        with self.queue_state_lock:
            if self.imu_job_pending:
                return
            self.imu_job_pending = True

        self.enqueue_job(priority=2, category="read_imu")


    def send_generic_json_callback(self, request, response):
        raw_json: str = request.json_command.strip()

        if not raw_json:
            response.success = False
            response.response_json = ''
            response.response_text = 'Empty JSON command'
            return response

        try:
            payload = json.loads(raw_json)

        except json.JSONDecodeError as e:
            response.success = False
            response.response_json = ''
            response.response_text = f'Invalid JSON command: {e}'
            return response

        if not isinstance(payload, dict):
            response.success = False
            response.response_json = ''
            response.response_text = 'JSON command must decode to an object/dict'
            return response

        reply_queue: queue.Queue = queue.Queue(maxsize=1)
        self.enqueue_job(
            priority=1,
            category='generic_json',
            payload=payload,
            reply_queue=reply_queue,
        )

        try:
            ok, info = reply_queue.get(timeout=self.generic_json_service_timeout)
        
        except queue.Empty:
            response.success = False
            response.response_json = ''
            response.response_text = (
                f'Timed out waiting for worker result after '
                f'{self.generic_json_service_timeout:.3f} s'
            )
            return response

        response.success = ok
        
        if isinstance(info, dict):
            response.response_json = json.dumps(info)
            response.response_text = ''
        else:
            response.response_json = ''
            response.response_text = '' if info is None else str(info)

        return response


    def send_generic_json_unavailable_callback(self, request, response):
        response.success = False
        response.message = (
            'Custom service type SendGenericJson is not available. '
            'Create it and rebuild the workspace.'
        )
        return response
    

    # ------------ #
    # Queue helper #
    # ------------ #

    def enqueue_job(self, priority: int, category: str, payload: Any = None, reply_queue: Optional[queue.Queue] = None) -> None:
        count = next(self.job_counter)
        job = RobotJob(category=category, payload=payload, reply_queue=reply_queue)
        self.job_queue.put((priority, count, job))
        print(f"a new job has been added to the queue! total jobs: {self.job_queue.qsize()}")
    

    # ----------------------------------------- #
    # Differential drive kinematics calculation #
    # ----------------------------------------- #

    def compute_wheel_speeds(self) -> tuple[float, float]:
        now: Time = self.get_clock().now()
        dt_since_cmd: float = (now - self.last_cmd_vel_time).nanoseconds * 1e-9

        # if more than cmd_vel_timeout seconds have passed since the last
        # command was issued, send a command to stop the robot
        if dt_since_cmd > self.cmd_vel_timeout:
            return 0.0, 0.0

        v: float = self.last_cmd_vel.linear.x
        w: float = self.last_cmd_vel.angular.z
        L: float = self.wheel_base

        v_l: float = v - 0.5 * L * w
        v_r: float = v + 0.5 * L * w

        # Clamp speeds to the 1 m/s limit
        max_speed: float = 1.0
        v_l = max(-max_speed, min(max_speed, v_l))
        v_r = max(-max_speed, min(max_speed, v_r))

        return v_l, v_r
    

    # ------------- #
    # Worker thread #
    # ------------- #

    def http_worker(self) -> None:
        while not self.stop_worker.is_set():
            try:
                _, _, job = self.job_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                # Send wheel speeds to the robot
                if job.category == "cmd_vel":
                    # Calculates the speed according to the latest available information
                    left, right = self.compute_wheel_speeds()
                    ok, info = self.hw_interface.set_wheel_speeds(left, right)

                    if ok:
                        self.get_logger().debug(
                            f"SET_WHEEL_SPEEDS command executed successfully! {info}"
                        )
                    elif not ok and info is not None:
                        self.get_logger().warn(
                            f"HTTP error (the velocity command may have reached the robot and been executed anyway): {info}"
                        )

                # Get IMU info
                elif job.category == "read_imu":
                    ok, info = self.hw_interface.get_imu_info()

                    # IMU info was obtained successfully
                    if ok:
                        self.get_logger().debug(
                            f"GET_IMU command executed successfully! {info}"
                        )

                        parsed = self.parse_info(info)

                        if isinstance(parsed, dict):
                            self.publish_imu(parsed)

                        else:
                            self.get_logger().warn(
                                f"{job.category} returned non-dict: {parsed}"
                            )
                    
                    # IMU info was not obtained successfully
                    elif not ok and info is not None:
                        self.get_logger().warn(
                            f"{job.category} failed: {info}"
                        )

                # Send generic json command
                elif job.category == 'generic_json':
                    ok, info = self.hw_interface.send_generic_json(job.payload)

                    if ok:
                        self.get_logger().debug(
                            f"SEND_GENERIC_JSON command executed successfully! {info}"
                        )

                    if job.reply_queue is not None:
                        job.reply_queue.put((ok, info))
            
                else:
                    self.get_logger().warn(
                        f"Unknown job kind: {job.category}"
                    )

            except Exception as e:
                self.get_logger().error(
                    f"Worker error in job {job.category}: {e}"
                )

                if job.reply_queue is not None:
                    job.reply_queue.put((False, str(e)))

            finally:
                with self.queue_state_lock:
                    if job.category == 'cmd_vel':
                        self.cmd_vel_job_pending = False
                    elif job.category == 'read_imu':
                        self.imu_job_pending = False
            
                self.job_queue.task_done()


    # ---------------------- #
    # Parsing and publishing #
    # ---------------------- #

    def parse_info(self, info: Any) -> Any:
        if isinstance(info, dict):
            return info

        if isinstance(info, str):
            try:
                return json.loads(info)
            except json.JSONDecodeError:
                return info

        return info

    def publish_imu(self, data: dict[str, Any]) -> None:
        try:
            # Publish temperature
            temp_msg = Float32()
            temp_msg.data = float(data['temp'])

            self.temp_pub.publish(temp_msg)
            self.get_logger().info(
                f"Publishing temperature: {temp_msg.data}"
            )

            # Publish roll pitch yaw (x=roll, y=pitch, z=yaw)
            ori_msg = Vector3()
            ori_msg.x = float(data['roll'])
            ori_msg.y = float(data['pitch'])
            ori_msg.z = float(data['yaw'])

            self.orientation_pub.publish(ori_msg)
            self.get_logger().info(
                f"Publishing roll, pitch, yaw: {ori_msg.x, ori_msg.y, ori_msg.z}"
            )

            # Publish acceleration
            acce_msg = Vector3()
            acce_msg.x = float(data['acce_X'])
            acce_msg.y = float(data['acce_Y'])
            acce_msg.z = float(data['acce_Z'])

            self.acce_pub.publish(acce_msg)
            self.get_logger().info(
                f"Publishing acceleration: {acce_msg.x, acce_msg.y, acce_msg.z}"
            )

            # Publish gyro
            gyro_msg = Vector3()
            gyro_msg.x = float(data['gyro_X'])
            gyro_msg.y = float(data['gyro_Y'])
            gyro_msg.z = float(data['gyro_Z'])

            self.gyro_pub.publish(gyro_msg)
            self.get_logger().info(
                f"Publishing gyro: {gyro_msg.x, gyro_msg.y, gyro_msg.z}"
            )

            # Publish magn
            magn_msg = Vector3()
            magn_msg.x = float(data['magn_X'])
            magn_msg.y = float(data['magn_Y'])
            magn_msg.z = float(data['magn_Z'])

            self.magn_pub.publish(magn_msg)
            self.get_logger().info(
                f"Publishing magn: {magn_msg.x, magn_msg.y, magn_msg.z}"
            )
        
        except KeyError as e:
            self.get_logger().warn(f'Missing IMU field in response: {e}')

        except (TypeError, ValueError) as e:
            self.get_logger().warn(f'Invalid IMU data in response: {e}')


    # -------- #
    # Shutdown #
    # -------- #

    def destroy_node(self):
        self.stop_worker.set()
        
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """
    Initializes ROS, creates the node, spins (handles callbacks), then shuts down.
    """
    
    rclpy.init(args=args)
    node = UGV01HttpDriver()
    
    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nCtrl+C received, shutting down http driver")
        
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
