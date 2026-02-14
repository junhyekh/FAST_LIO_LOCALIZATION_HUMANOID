#!/usr/bin/env python3
"""Publish map -> world transform using TF lookups.

Lookups:
	- map -> imu_link
	- world -> body

Assumption:
	- imu_link and body are the same frame.
"""

import subprocess
import math
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException


def _quat_multiply(q1, q2):
	x1, y1, z1, w1 = q1
	x2, y2, z2, w2 = q2
	return (
		w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
		w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
		w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
		w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
	)


def _quat_conjugate(q):
	x, y, z, w = q
	return (-x, -y, -z, w)


def _quat_rotate_vector(q, v):
	qv = (v[0], v[1], v[2], 0.0)
	qr = _quat_multiply(_quat_multiply(q, qv), _quat_conjugate(q))
	return (qr[0], qr[1], qr[2])


def _normalize_quat(q):
	x, y, z, w = q
	n = math.sqrt(x * x + y * y + z * z + w * w)
	if n == 0.0:
		return (0.0, 0.0, 0.0, 1.0)
	return (x / n, y / n, z / n, w / n)


def _compose_transform(t1, q1, t2, q2):
	q1n = _normalize_quat(q1)
	q2n = _normalize_quat(q2)
	t2r = _quat_rotate_vector(q1n, t2)
	t = (t1[0] + t2r[0], t1[1] + t2r[1], t1[2] + t2r[2])
	q = _quat_multiply(q1n, q2n)
	return t, _normalize_quat(q)


class MapToWorldPublisher(Node):
	def __init__(self):
		super().__init__("map_to_world_publisher")

		self.declare_parameter("map_frame", "map")
		self.declare_parameter("imu_frame", "imu_link")
		self.declare_parameter("body_frame", "body")
		self.declare_parameter("world_frame", "world")
		self.declare_parameter("publish_period_sec", 0.1)

		self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
		self.imu_frame = self.get_parameter("imu_frame").get_parameter_value().string_value
		self.body_frame = self.get_parameter("body_frame").get_parameter_value().string_value
		self.world_frame = (
			self.get_parameter("world_frame").get_parameter_value().string_value
		)
		period = self.get_parameter("publish_period_sec").get_parameter_value().double_value
		if period <= 0.0:
			period = 0.1

		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.tf_broadcaster = TransformBroadcaster(self)
		self.last_transform = None
		self.timer = self.create_timer(period, self._on_timer)
		self.get_logger().info(
			"Publishing map->world using map->imu_link and world->body"  # noqa: E501
		)

	def _lookup_transform(self, target_frame, source_frame):
		return self.tf_buffer.lookup_transform(
			target_frame,
			source_frame,
			rclpy.time.Time(),
			timeout=Duration(seconds=1),
		)

	def _invert_transform(self,t, q):
		# t: (x,y,z), q: (x,y,z,w)  representing parent->child
		qn = _normalize_quat(q)
		q_inv = _quat_conjugate(qn)

		# t_inv = - R(q_inv) * t
		t_rot = _quat_rotate_vector(q_inv, t)
		t_inv = (-t_rot[0], -t_rot[1], -t_rot[2])

		return t_inv, _normalize_quat(q_inv)

	def _on_timer(self):
		if self.last_transform is None:
			try:
				world2body = self._lookup_transform(
					self.world_frame, self.body_frame
				)
				map2imu = self._lookup_transform(self.map_frame, self.imu_frame)

				# imu == body
				t_map = map2imu.transform.translation
				q_map = map2imu.transform.rotation
				t_wb = world2body.transform.translation
				q_wb = world2body.transform.rotation

				# map->world = map->body * body->world
				t_bw, q_bw = self._invert_transform(
					(t_wb.x, t_wb.y, t_wb.z),
					(q_wb.x, q_wb.y, q_wb.z, q_wb.w),
				)
				t_map_w, q_map_w = _compose_transform(
					(t_map.x, t_map.y, t_map.z),
					(q_map.x, q_map.y, q_map.z, q_map.w),
					(t_bw[0], t_bw[1], t_bw[2]),
					(q_bw[0], q_bw[1], q_bw[2], q_bw[3]),
				)

				msg = TransformStamped()
				msg.header.stamp = self.get_clock().now().to_msg()
				msg.header.frame_id = self.map_frame
				msg.child_frame_id = self.world_frame
				msg.transform.translation.x = t_map_w[0]
				msg.transform.translation.y = t_map_w[1]
				msg.transform.translation.z = t_map_w[2]
				msg.transform.rotation.x = q_map_w[0]
				msg.transform.rotation.y = q_map_w[1]
				msg.transform.rotation.z = q_map_w[2]
				msg.transform.rotation.w = q_map_w[3]

				self.last_transform = msg
				print("Initialized map->world transform")
				print(msg)
				subprocess.run(["pkill", "-f", "base_center_broadcaster"])
				subprocess.run(["pkill", "-f", "imulink2baselink"])
				subprocess.run(["pkill", "-f", "camera_init2odom"])
				subprocess.run(["pkill", "-f", "open3d_loc_g1"])
			except TransformException as exc:
				self.get_logger().warn(f"TF lookup failed: {exc}")
				return

		self.last_transform.header.stamp = self.get_clock().now().to_msg()
		self.tf_broadcaster.sendTransform(self.last_transform)


def main():
	rclpy.init()
	node = MapToWorldPublisher()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
