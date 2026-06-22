#!/usr/bin/env python3
"""
dock_pose_publisher.py  v2
Publica pe /detected_dock_pose tag-ul cel mai aproape de robot.
Nu mai necesita setarea manuala a dock_tag_id.
Cauta toate tag-urile configurate si il publica pe cel mai aproape.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import yaml
import os


DOCK_DATABASE_FILE = os.path.expanduser(
    '~/saim_nouzen/src/amr2ax_nav2/config/dock_database.yaml'
)


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class DockPosePublisher(Node):
    def __init__(self):
        super().__init__('dock_pose_publisher')

        self.declare_parameter('dock_tag_family', 'tag36h11')
        self.declare_parameter('dock_tag_id', -1)  # -1 = auto mode
        self.declare_parameter('approach_yaw', 0.0)
        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('max_tf_age', 5.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)

        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.publish_pose)

        # Incarca dock database pentru tag_id -> approach_yaw mapping
        self.dock_map = {}  # tag_id -> approach_yaw
        self.load_dock_database()

        # Tracking
        self.publish_count = 0
        self.stale_count = 0
        self.last_x = None
        self.last_y = None
        self.last_tag_id = None
        self.active_tag_id = None

        self.get_logger().info(
            f'dock_pose_publisher v2 pornit. '
            f'Tag-uri monitorizate: {list(self.dock_map.keys())}'
        )

    def load_dock_database(self):
        """Incarca dock_database.yaml si construieste mapping tag_id -> approach_yaw."""
        try:
            with open(DOCK_DATABASE_FILE, 'r') as f:
                db = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Nu pot citi dock_database: {e}')
            return

        # Tag ID mapping hardcodat (sau poate fi mutat in config)
        tag_id_map = {
            'nouzen_dock_station': 0,
            'home': 1,
            'input_a': 2,
            'input_b': 3,
            'output_a': 4,
            'output_b': 5,
            'SCSS1': 6,
            'SCSS2': 7,
        }

        docks = db.get('docks', {})
        for dock_name, dock_cfg in docks.items():
            if dock_name in tag_id_map:
                tag_id = tag_id_map[dock_name]
                approach_yaw = dock_cfg.get('approach_yaw', 0.0)
                self.dock_map[tag_id] = approach_yaw
                self.get_logger().info(
                    f'  Tag {tag_id} ({dock_name}): '
                    f'approach_yaw={math.degrees(approach_yaw):.1f}deg'
                )

    def publish_pose(self):
        family = self.get_parameter('dock_tag_family').value
        manual_tag_id = self.get_parameter('dock_tag_id').value
        fixed_frame = self.get_parameter('fixed_frame').value
        max_tf_age = self.get_parameter('max_tf_age').value

        # Daca dock_tag_id e setat manual (>= 0), foloseste modul vechi
        if manual_tag_id >= 0:
            approach_yaw = self.get_parameter('approach_yaw').value
            self._publish_single_tag(
                family, manual_tag_id, approach_yaw,
                fixed_frame, max_tf_age
            )
            return

        # Auto mode: cauta cel mai aproape tag vizibil
        now = self.get_clock().now()
        best_tag = None
        best_dist = float('inf')
        best_x = 0.0
        best_y = 0.0
        best_stamp = None

        for tag_id in self.dock_map:
            tag_frame = f'{family}:{tag_id}'
            try:
                transform = self.tf_buffer.lookup_transform(
                    fixed_frame, tag_frame, Time(),
                    timeout=Duration(seconds=0.05)
                )
            except Exception:
                continue

            # Staleness check
            tf_time = Time.from_msg(transform.header.stamp)
            age_s = (now.nanoseconds - tf_time.nanoseconds) / 1e9
            if age_s > max_tf_age:
                continue

            # Calculeaza distanta de la robot la tag
            tag_x = transform.transform.translation.x
            tag_y = transform.transform.translation.y

            # Distanta de la origine (robot e aproape de 0,0 in map)
            # Mai bine: distanta de la base_link
            try:
                robot_tf = self.tf_buffer.lookup_transform(
                    fixed_frame, 'base_link', Time(),
                    timeout=Duration(seconds=0.05)
                )
                rx = robot_tf.transform.translation.x
                ry = robot_tf.transform.translation.y
                dist = math.sqrt((tag_x - rx)**2 + (tag_y - ry)**2)
            except Exception:
                dist = math.sqrt(tag_x**2 + tag_y**2)

            if dist < best_dist:
                best_dist = dist
                best_tag = tag_id
                best_x = tag_x
                best_y = tag_y
                best_stamp = transform.header.stamp

        if best_tag is None:
            return

        # Log cand se schimba tag-ul activ
        if best_tag != self.active_tag_id:
            approach_yaw = self.dock_map[best_tag]
            self.get_logger().info(
                f'Auto: tag activ {family}:{best_tag} '
                f'(dist={best_dist:.2f}m, '
                f'yaw={math.degrees(approach_yaw):.1f}deg)'
            )
            self.active_tag_id = best_tag
            self.last_x = None
            self.last_y = None
            self.publish_count = 0

        approach_yaw = self.dock_map[best_tag]

        # Position jump detection
        self.publish_count += 1
        if self.last_x is not None:
            jump = math.sqrt(
                (best_x - self.last_x)**2 + (best_y - self.last_y)**2
            )
            if jump > 0.15:
                self.get_logger().warn(
                    f'Position jump {jump:.3f}m pe tag {best_tag}'
                )
        self.last_x = best_x
        self.last_y = best_y

        if self.publish_count % 100 == 1:
            self.get_logger().info(
                f'Tag {family}:{best_tag}: x={best_x:.3f}, y={best_y:.3f}, '
                f'dist={best_dist:.2f}m'
            )

        # Publica
        qx, qy, qz, qw = yaw_to_quaternion(approach_yaw)
        pose = PoseStamped()
        pose.header.stamp = best_stamp
        pose.header.frame_id = fixed_frame
        pose.pose.position.x = best_x
        pose.pose.position.y = best_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pub.publish(pose)

    def _publish_single_tag(self, family, tag_id, approach_yaw,
                            fixed_frame, max_tf_age):
        """Modul vechi: publica un singur tag setat manual."""
        tag_frame = f'{family}:{tag_id}'

        if tag_id != self.last_tag_id:
            self.get_logger().info(
                f'Manual: tag {tag_frame}, '
                f'approach_yaw={math.degrees(approach_yaw):.1f}deg'
            )
            self.last_tag_id = tag_id
            self.last_x = None
            self.last_y = None
            self.publish_count = 0
            self.stale_count = 0

        try:
            now = self.get_clock().now()
            transform = self.tf_buffer.lookup_transform(
                fixed_frame, tag_frame, Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(
                f'TF lookup failed pentru {tag_frame}: {e}',
                throttle_duration_sec=5.0
            )
            return

        tf_time = Time.from_msg(transform.header.stamp)
        age_s = (now.nanoseconds - tf_time.nanoseconds) / 1e9
        if age_s > max_tf_age:
            self.stale_count += 1
            if self.stale_count % 50 == 1:
                self.get_logger().warn(
                    f'TF stale: {age_s:.2f}s > {max_tf_age}s'
                )
            return

        self.stale_count = 0
        tag_x = transform.transform.translation.x
        tag_y = transform.transform.translation.y

        self.publish_count += 1
        if self.publish_count % 100 == 1:
            self.get_logger().info(
                f'Tag {tag_frame}: x={tag_x:.3f}, y={tag_y:.3f}'
            )

        if self.last_x is not None:
            jump = math.sqrt(
                (tag_x - self.last_x)**2 + (tag_y - self.last_y)**2
            )
            if jump > 0.15:
                self.get_logger().warn(
                    f'Position jump {jump:.3f}m'
                )
        self.last_x = tag_x
        self.last_y = tag_y

        qx, qy, qz, qw = yaw_to_quaternion(approach_yaw)
        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = fixed_frame
        pose.pose.position.x = tag_x
        pose.pose.position.y = tag_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pub.publish(pose)


def main():
    rclpy.init()
    node = DockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()