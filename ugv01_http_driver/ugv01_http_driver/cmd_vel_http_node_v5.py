from __future__ import annotations
import threading
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, Float64, String

from .http_connection_v1 import UGV01Http


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

		self.ip_addr : str = (
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

		self.hw_interface : UGV01Http = UGV01Http(self.ip_addr)

		# Start last command as zero (no movement)
		self.last_cmd : Twist = Twist()
		# Initialize last_cmd_time to "now" so the robot doesn't stop immediately at startup
		self.last_cmd_time : Time = self.get_clock().now()

		# Timer to send velocity commands to robot at fixed rate
		self.timer = self.create_timer(
			timer_period_sec=(1.0 / self.cmd_rate), callback=self.timer_callback
		)

		# Create subscriber to /cmd_vel
		# qos = 10 -> incoming queue size
		self.cmd_vel_sub = self.create_subscription(
			msg_type=Twist, topic='cmd_vel', callback=self.cmd_vel_callback, qos_profile=10
		)

		# Create publisher to /voltage
		self.voltage_pub = self.create_publisher(
			msg_type=Float32, topic='voltage', qos_profile=10
		)

		# Create publisher to /orientation
		self.orientation_pub = self.create_publisher(
			msg_type=Vector3, topic='orientation', qos_profile=10 
		)

		# Create publisher to /magn
		self.magn_pub = self.create_publisher(
			msg_type=Vector3, topic='magn', qos_profile=10
		)

		# Create publisher to /ip
		self.ip_pub = self.create_publisher(
			msg_type = String, topic='ip', qos_profile=10
		)

		# Create publisher to /mac
		self.mac_pub = self.create_publisher(
			msg_type = String, topic='mac', qos_profile=10
		)

		# Create publisher to /rssi
		self.rssi_pub = self.create_publisher(
			msg_type=Float32, topic='rssi', qos_profile=10
		)

		# Create publisher to /temp
		self.temp_pub = self.create_publisher(
			msg_type=Float32, topic='temp', qos_profile=10
		)

		# Create publisher to /acce
		self.acce_pub = self.create_publisher(
			msg_type=Vector3, topic='acce', qos_profile=10
		)

		# Create publisher to /gyro
		self.gyro_pub = self.create_publisher(
			msg_type=Vector3, topic='gyro', qos_profile=10
		)

		# Create publisher to /shunt_mV
		self.shunt_mV_pub = self.create_publisher(
			msg_type=Float64, topic='shunt_mV', qos_profile=10
		)

		# Create publisher to /load_V
		self.load_V_pub = self.create_publisher(
			msg_type=Float64, topic='load_V', qos_profile=10
		)

		# Create publisher to /bus_V
		self.bus_V_pub = self.create_publisher(
			msg_type=Float64, topic='bus_V', qos_profile=10
		)
		
		# Create publisher to /current_mA
		self.current_mA_pub = self.create_publisher(
			msg_type=Float64, topic='current_mA', qos_profile=10
		)

		# Create publisher to /power_mW
		self.power_mW_pub = self.create_publisher(
			msg_type=Float64, topic='power_mW', qos_profile=10
		)

		self.get_logger().info(f"UGV01 HTTP driver started, IP={self.ip_addr}")
		
		self.terminal_thread = threading.Thread(target=self.terminal_loop, daemon=True)
		self.terminal_thread.start()
		

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

			# Clamp speeds to the 1 m/s limit
			max_speed : float = 1.0
			v_l = max(-max_speed, min(max_speed, v_l))
			v_r = max(-max_speed, min(max_speed, v_r))


		ok, info = self.hw_interface.set_wheel_speeds(v_l, v_r)
				
		if ok:
			self.get_logger().debug(
				f"SET_WHEEL_SPEEDS command executed successfully! {info}"
			)
			
		elif not ok and info is not None:
			self.get_logger().warn(
				f"HTTP error (the command may have reached the robot and been executed anyway): {info}"
			)
		
		
	def terminal_loop(self) -> None:
		while rclpy.ok():
			try:
				line = input("http cmd> ").strip()

				if not line:
					continue

				if line == "quit":
					break


				# Get IMU info and publish it to corresponding topics
				if line == "get_imu":
					ok, info = self.hw_interface.get_imu_info()

					# IMU info was obtained successfully
					if ok:
						self.get_logger().debug(
							f"GET_IMU command executed successfully! {info}"
						)

						# Publish temperature
						temp_msg = Float32()
						temp_msg.data = float(info['temp'])

						self.temp_pub.publish(temp_msg)
						self.get_logger().info(
							f"Publishing temperature: {temp_msg.data}"
						)

						# Publish roll pitch yaw (x=roll, y=pitch, z=yaw)
						ori_msg = Vector3()
						ori_msg.x = float(info['roll'])
						ori_msg.y = float(info['pitch'])
						ori_msg.z = float(info['yaw'])

						self.orientation_pub.publish(ori_msg)
						self.get_logger().info(
							f"Publishing roll, pitch, yaw: {ori_msg.x, ori_msg.y, ori_msg.z}"
						)

						# Publish acceleration
						acce_msg = Vector3()
						acce_msg.x = float(info['acce_X'])
						acce_msg.y = float(info['acce_Y'])
						acce_msg.z = float(info['acce_Z'])

						self.acce_pub.publish(acce_msg)
						self.get_logger().info(
							f"Publishing acceleration: {acce_msg.x, acce_msg.y, acce_msg.z}"
						)

						# Publish gyro
						gyro_msg = Vector3()
						gyro_msg.x = float(info['gyro_X'])
						gyro_msg.y = float(info['gyro_Y'])
						gyro_msg.z = float(info['gyro_Z'])

						self.gyro_pub.publish(gyro_msg)
						self.get_logger().info(
							f"Publishing gyro: {gyro_msg.x, gyro_msg.y, gyro_msg.z}"
						)

						# Publish magn
						magn_msg = Vector3()
						magn_msg.x = float(info['magn_X'])
						magn_msg.y = float(info['magn_Y'])
						magn_msg.z = float(info['magn_Z'])

						self.magn_pub.publish(magn_msg)
						self.get_logger().info(
							f"Publishing magn: {magn_msg.x, magn_msg.y, magn_msg.z}"
						)

					# IMU info was not obtained successfully
					elif not ok and info is not None:
						self.get_logger().warn(
							f"HTTP error (the command may have reached the robot and been executed anyway): {info}"
						)


				# Get INA219 info and publish it to corresponding topics
				if line == "get_ina":
					ok, info = self.hw_interface.get_ina219_info()

					# INA219 info was obtained successfully
					if ok:
						self.get_logger().debug(
							f"GET_INA219 command executed successfully! {info}"
						)

						# Publish shunt_mV
						shunt_msg = Float64()
						shunt_msg.data = float(info['shunt_mV'])

						self.shunt_mV_pub.publish(shunt_msg)
						self.get_logger().info(
							f"Publishing shunt_mV: {shunt_msg.data}"
						)

						# Publish load_V
						load_msg = Float64()
						load_msg.data = float(info['load_V'])

						self.load_V_pub.publish(load_msg)
						self.get_logger().info(
							f"Publishing load_V: {load_msg.data}"
						)

						# Publish bus_V
						bus_msg = Float64()
						bus_msg.data = float(info['bus_V'])

						self.bus_V_pub.publish(bus_msg)
						self.get_logger().info(
							f"Publishing bus_V: {bus_msg.data}"
						)

						# Publish current_mA
						current_msg = Float64()

						# TODO: decidir se vale a pena publicar ou não quando o valor for None
						if info['current_mA'] == None:
							current_msg.data = 0.0
						else:
							current_msg.data = float(info['current_mA'])

						self.current_mA_pub.publish(current_msg)
						self.get_logger().info(
							f"Publishing current_mA: {current_msg.data}"
						)

						# Publish power_mW
						power_msg = Float64()

						# TODO: decidir se vale a pena publicar ou não quando o valor for None
						if info['power_mW'] == None:
							power_msg.data = 0.0
						else:
							power_msg.data = float(info['power_mW'])

						self.power_mW_pub.publish(power_msg)
						self.get_logger().info(
							f"Publishing power_mW: {power_msg.data}"
						)
					
					# ina219 info was not obtained successfully
					elif not ok and info is not None:
						self.get_logger().warn(
							f"HTTP error (the command may have reached the robot and been executed anyway): {info}"
						)


				# Get device info and publish it to corresponding topics
				if line == "get_device_info":
					ok, info = self.hw_interface.get_device_info()
				
				# Device info was obtained successfully
					if ok:
						self.get_logger().debug(
							f"GET_DEVICE_INFO command executed successfully! {info}"
						)

						# Publish voltage
						volt_msg = Float32()
						volt_msg.data = float(info['V'])

						self.voltage_pub.publish(volt_msg)
						self.get_logger().info(
							f"Publishing voltage: {volt_msg.data}"
						)

						# Publish roll pitch yaw (x=roll, y=pitch, z=yaw)
						# These values for rpy are less precise than those obtained by IMU_GET
						ori_msg = Vector3()
						ori_msg.x = float(info['r'])
						ori_msg.y = float(info['p'])
						ori_msg.z = float(info['y'])

						self.orientation_pub.publish(ori_msg)
						self.get_logger().info(
							f"Publishing roll, pitch, yaw: {ori_msg.x, ori_msg.y, ori_msg.z}"
						)

						# Publish magn
						# These values for magn are less precise than those obtained by IMU_GET
						magn_msg = Vector3()
						magn_msg.x = float(info['mX'])
						magn_msg.y = float(info['mY'])
						magn_msg.z = float(info['mZ'])

						self.magn_pub.publish(magn_msg)
						self.get_logger().info(
							f"Publishing magn: {magn_msg.x, magn_msg.y, magn_msg.z}"
						)

						# Publish IP
						ip_msg = String()
						ip_msg.data = info['IP']

						self.ip_pub.publish(ip_msg)
						self.get_logger().info(
							f"Publishing IP: {ip_msg.data}"
						)

						# Publish MAC
						mac_msg = String()
						mac_msg.data = info['MAC']

						self.mac_pub.publish(mac_msg)
						self.get_logger().info(
							f"Publishing MAC: {mac_msg.data}"
						)

						# Publish RSSI
						rssi_msg = Float32()
						rssi_msg.data = float(info['RSSI'])

						self.rssi_pub.publish(rssi_msg)
						self.get_logger().info(
							f"Publishing RSSI: {rssi_msg.data}"
						)
					
					# Device info was not obtained successfully
					elif not ok and info is not None:
						self.get_logger().warn(
							f"HTTP error (the command may have reached the robot and been executed anyway): {info}"
						)


			except EOFError:
				break
			except Exception as e:
				print(f"Terminal command error: {e}")


def main(args : list[str] | None = None) -> None:
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


if __name__ == '__main__':
	main()
