#!/usr/bin/env python3
"""Record one fixed-command localization trial.

The script is intentionally kept outside ROS package installation so it can be
run directly from the workspace:

  python3 tools/localization_compare/record_trial.py --label 3d --drive
"""

import argparse
import csv
import json
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


TF_FIELDS = [
    "label",
    "target_frame",
    "source_frame",
    "wall_time_sec",
    "elapsed_sec",
    "ros_time_sec",
    "stamp_sec",
    "x",
    "y",
    "z",
    "qx",
    "qy",
    "qz",
    "qw",
    "yaw_rad",
    "yaw_deg",
]

RAW_POSE_FIELDS = [
    "label",
    "topic",
    "msg_type",
    "frame_id",
    "child_frame_id",
    "wall_time_sec",
    "elapsed_sec",
    "ros_time_sec",
    "stamp_sec",
    "x",
    "y",
    "z",
    "qx",
    "qy",
    "qz",
    "qw",
    "yaw_rad",
    "yaw_deg",
]

CMD_FIELDS = [
    "label",
    "wall_time_sec",
    "elapsed_sec",
    "ros_time_sec",
    "phase",
    "linear_x",
    "linear_y",
    "linear_z",
    "angular_x",
    "angular_y",
    "angular_z",
]


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # ROS uses xyzw quaternion ordering.
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def sanitize_label(label: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9_.-]+", "_", label.strip())
    return clean or "trial"


def open_csv(path: Path, fields):
    handle = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()
    return handle, writer


class TrialRecorder(Node):
    def __init__(self, args, output_dir: Path):
        super().__init__("localization_compare_trial_recorder")
        self.args = args
        self.output_dir = output_dir

        self.wall_start = time.monotonic()
        self.drive_start = self.wall_start + args.pre_roll
        self.drive_end = self.drive_start + args.drive_duration
        self.wall_end = self.drive_end + args.post_roll

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)

        self.tf_handle, self.tf_writer = open_csv(output_dir / "tf_samples.csv", TF_FIELDS)
        self.raw_handle, self.raw_writer = open_csv(output_dir / "raw_pose_samples.csv", RAW_POSE_FIELDS)
        self.cmd_handle, self.cmd_writer = open_csv(output_dir / "cmd_vel_samples.csv", CMD_FIELDS)

        self.tf_counts: Dict[str, int] = {}
        self.tf_missing_counts: Dict[str, int] = {}
        self.raw_counts: Dict[str, int] = {}
        self.cmd_count = 0
        self.exit_reason = "completed"

        self.create_subscription(
            PoseWithCovarianceStamped,
            args.amcl_topic,
            lambda msg: self.pose_cov_callback(args.amcl_topic, msg),
            20,
        )
        self.create_subscription(
            Odometry,
            args.localized_topic,
            lambda msg: self.odom_callback(args.localized_topic, msg),
            20,
        )
        self.create_subscription(
            Odometry,
            args.odom_topic,
            lambda msg: self.odom_callback(args.odom_topic, msg),
            50,
        )

    def elapsed(self) -> float:
        return time.monotonic() - self.wall_start

    def ros_now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def pose_cov_callback(self, topic: str, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self.write_raw_pose(
            topic=topic,
            msg_type="geometry_msgs/PoseWithCovarianceStamped",
            frame_id=msg.header.frame_id,
            child_frame_id="",
            stamp_sec=stamp_to_sec(msg.header.stamp),
            pose_tuple=(
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def odom_callback(self, topic: str, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.write_raw_pose(
            topic=topic,
            msg_type="nav_msgs/Odometry",
            frame_id=msg.header.frame_id,
            child_frame_id=msg.child_frame_id,
            stamp_sec=stamp_to_sec(msg.header.stamp),
            pose_tuple=(
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def write_raw_pose(
        self,
        topic: str,
        msg_type: str,
        frame_id: str,
        child_frame_id: str,
        stamp_sec: float,
        pose_tuple: Tuple[float, float, float, float, float, float, float],
    ) -> None:
        x, y, z, qx, qy, qz, qw = pose_tuple
        yaw = quat_to_yaw(qx, qy, qz, qw)
        self.raw_writer.writerow(
            {
                "label": self.args.label,
                "topic": topic,
                "msg_type": msg_type,
                "frame_id": frame_id,
                "child_frame_id": child_frame_id,
                "wall_time_sec": f"{time.monotonic():.9f}",
                "elapsed_sec": f"{self.elapsed():.9f}",
                "ros_time_sec": f"{self.ros_now_sec():.9f}",
                "stamp_sec": f"{stamp_sec:.9f}",
                "x": f"{x:.9f}",
                "y": f"{y:.9f}",
                "z": f"{z:.9f}",
                "qx": f"{qx:.9f}",
                "qy": f"{qy:.9f}",
                "qz": f"{qz:.9f}",
                "qw": f"{qw:.9f}",
                "yaw_rad": f"{yaw:.9f}",
                "yaw_deg": f"{math.degrees(yaw):.6f}",
            }
        )
        self.raw_counts[topic] = self.raw_counts.get(topic, 0) + 1

    def phase_for_time(self, now: Optional[float] = None) -> str:
        now = time.monotonic() if now is None else now
        if now < self.drive_start:
            return "pre_roll"
        if now < self.drive_end:
            return "drive"
        return "post_roll"

    def command_for_time(self, now: Optional[float] = None) -> Twist:
        phase = self.phase_for_time(now)
        msg = Twist()
        if self.args.drive and phase == "drive":
            msg.linear.x = self.args.linear_x
            msg.linear.y = self.args.linear_y
            msg.angular.z = self.args.angular_z
        return msg

    def publish_command(self, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else now
        msg = self.command_for_time(now)
        self.cmd_pub.publish(msg)
        self.cmd_writer.writerow(
            {
                "label": self.args.label,
                "wall_time_sec": f"{now:.9f}",
                "elapsed_sec": f"{now - self.wall_start:.9f}",
                "ros_time_sec": f"{self.ros_now_sec():.9f}",
                "phase": self.phase_for_time(now),
                "linear_x": f"{msg.linear.x:.9f}",
                "linear_y": f"{msg.linear.y:.9f}",
                "linear_z": f"{msg.linear.z:.9f}",
                "angular_x": f"{msg.angular.x:.9f}",
                "angular_y": f"{msg.angular.y:.9f}",
                "angular_z": f"{msg.angular.z:.9f}",
            }
        )
        self.cmd_count += 1

    def publish_stop(self, repeats: int, period_sec: float) -> None:
        msg = Twist()
        for _ in range(repeats):
            self.cmd_pub.publish(msg)
            now = time.monotonic()
            self.cmd_writer.writerow(
                {
                    "label": self.args.label,
                    "wall_time_sec": f"{now:.9f}",
                    "elapsed_sec": f"{now - self.wall_start:.9f}",
                    "ros_time_sec": f"{self.ros_now_sec():.9f}",
                    "phase": "stop",
                    "linear_x": "0.000000000",
                    "linear_y": "0.000000000",
                    "linear_z": "0.000000000",
                    "angular_x": "0.000000000",
                    "angular_y": "0.000000000",
                    "angular_z": "0.000000000",
                }
            )
            self.cmd_count += 1
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)

    def record_tf_sample(self) -> None:
        for target, source in self.args.tf_pairs:
            key = f"{target}->{source}"
            try:
                transform = self.tf_buffer.lookup_transform(
                    target,
                    source,
                    Time(),
                    timeout=Duration(seconds=self.args.tf_timeout),
                )
            except TransformException as exc:
                self.tf_missing_counts[key] = self.tf_missing_counts.get(key, 0) + 1
                if self.args.verbose_missing_tf:
                    self.get_logger().warn(f"missing TF {key}: {exc}")
                continue

            t = transform.transform.translation
            q = transform.transform.rotation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            self.tf_writer.writerow(
                {
                    "label": self.args.label,
                    "target_frame": target,
                    "source_frame": source,
                    "wall_time_sec": f"{time.monotonic():.9f}",
                    "elapsed_sec": f"{self.elapsed():.9f}",
                    "ros_time_sec": f"{self.ros_now_sec():.9f}",
                    "stamp_sec": f"{stamp_to_sec(transform.header.stamp):.9f}",
                    "x": f"{t.x:.9f}",
                    "y": f"{t.y:.9f}",
                    "z": f"{t.z:.9f}",
                    "qx": f"{q.x:.9f}",
                    "qy": f"{q.y:.9f}",
                    "qz": f"{q.z:.9f}",
                    "qw": f"{q.w:.9f}",
                    "yaw_rad": f"{yaw:.9f}",
                    "yaw_deg": f"{math.degrees(yaw):.6f}",
                }
            )
            self.tf_counts[key] = self.tf_counts.get(key, 0) + 1

    def run(self) -> None:
        self.get_logger().info(f"writing trial data to {self.output_dir}")
        if self.args.drive:
            self.get_logger().warn(
                "drive mode enabled: non-zero /cmd_vel will be published during the drive window"
            )
        else:
            self.get_logger().info("dry record mode: pass --drive to publish non-zero /cmd_vel")

        next_cmd = self.wall_start
        next_sample = self.wall_start
        cmd_period = 1.0 / self.args.cmd_hz
        sample_period = 1.0 / self.args.sample_hz

        while rclpy.ok() and time.monotonic() < self.wall_end:
            now = time.monotonic()
            if now >= next_cmd:
                self.publish_command(now)
                next_cmd += cmd_period
            if now >= next_sample:
                self.record_tf_sample()
                next_sample += sample_period

            next_event = min(next_cmd, next_sample, self.wall_end)
            timeout = max(0.0, min(0.05, next_event - time.monotonic()))
            rclpy.spin_once(self, timeout_sec=timeout)

    def close(self) -> None:
        for handle in (self.tf_handle, self.raw_handle, self.cmd_handle):
            handle.flush()
            handle.close()

    def write_summary(self) -> None:
        summary = {
            "label": self.args.label,
            "output_dir": str(self.output_dir),
            "started_at": self.args.started_at,
            "finished_at": datetime.now().isoformat(timespec="seconds"),
            "exit_reason": self.exit_reason,
            "drive_enabled": bool(self.args.drive),
            "pre_roll": self.args.pre_roll,
            "drive_duration": self.args.drive_duration,
            "post_roll": self.args.post_roll,
            "sample_hz": self.args.sample_hz,
            "cmd_hz": self.args.cmd_hz,
            "cmd_topic": self.args.cmd_topic,
            "command": {
                "linear_x": self.args.linear_x if self.args.drive else 0.0,
                "linear_y": self.args.linear_y if self.args.drive else 0.0,
                "angular_z": self.args.angular_z if self.args.drive else 0.0,
            },
            "drive_start_elapsed_sec": self.args.pre_roll,
            "drive_end_elapsed_sec": self.args.pre_roll + self.args.drive_duration,
            "tf_counts": self.tf_counts,
            "tf_missing_counts": self.tf_missing_counts,
            "raw_counts": self.raw_counts,
            "cmd_count": self.cmd_count,
            "topics": {
                "amcl": self.args.amcl_topic,
                "localized_result": self.args.localized_topic,
                "odom": self.args.odom_topic,
            },
            "tf_pairs": [f"{target}->{source}" for target, source in self.args.tf_pairs],
        }
        with (self.output_dir / "summary.json").open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, ensure_ascii=False)


def parse_tf_pairs(values):
    pairs = []
    for value in values:
        if ":" in value:
            target, source = value.split(":", 1)
        elif "->" in value:
            target, source = value.split("->", 1)
        else:
            raise argparse.ArgumentTypeError(
                f"invalid TF pair '{value}', expected target:source or target->source"
            )
        target = target.strip()
        source = source.strip()
        if not target or not source:
            raise argparse.ArgumentTypeError(f"invalid TF pair '{value}'")
        pairs.append((target, source))
    return pairs


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record TF, localization topics, and fixed /cmd_vel for one localization trial."
    )
    parser.add_argument("--label", default="trial", help="Label written into outputs, e.g. 2d or 3d.")
    parser.add_argument("--output-root", default="~/loc_compare", help="Root directory for trial outputs.")
    parser.add_argument("--drive", action="store_true", help="Actually publish non-zero cmd_vel during drive.")
    parser.add_argument("--linear-x", type=float, default=0.10, help="Drive linear.x in m/s.")
    parser.add_argument("--linear-y", type=float, default=0.0, help="Drive linear.y in m/s.")
    parser.add_argument("--angular-z", type=float, default=0.0, help="Drive angular.z in rad/s.")
    parser.add_argument("--drive-duration", type=float, default=5.0, help="Non-zero command duration in seconds.")
    parser.add_argument("--pre-roll", type=float, default=2.0, help="Zero-command recording time before drive.")
    parser.add_argument("--post-roll", type=float, default=3.0, help="Zero-command recording time after drive.")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="TF sampling rate.")
    parser.add_argument("--cmd-hz", type=float, default=20.0, help="cmd_vel publish rate.")
    parser.add_argument("--cmd-topic", default="/cmd_vel", help="Command topic.")
    parser.add_argument("--amcl-topic", default="/amcl_pose", help="2D AMCL pose topic.")
    parser.add_argument(
        "--localized-topic",
        default="/localizer/localized_result",
        help="3D localizer odometry topic.",
    )
    parser.add_argument("--odom-topic", default="/odom", help="Odometry topic.")
    parser.add_argument("--map-frame", default="map", help="Map frame.")
    parser.add_argument("--odom-frame", default="odom", help="Odom frame.")
    parser.add_argument("--base-frame", default="base_link", help="Base frame.")
    parser.add_argument("--tf-timeout", type=float, default=0.02, help="TF lookup timeout in seconds.")
    parser.add_argument("--stop-repeats", type=int, default=10, help="Zero cmd_vel messages on exit.")
    parser.add_argument("--stop-period", type=float, default=0.05, help="Delay between exit zero commands.")
    parser.add_argument("--verbose-missing-tf", action="store_true", help="Log every missing TF lookup.")
    parser.add_argument(
        "--tf-pair",
        action="append",
        default=None,
        help="Additional TF pair as target:source. May be repeated.",
    )
    return parser


def prepare_args(argv=None):
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.sample_hz <= 0.0 or args.cmd_hz <= 0.0:
        parser.error("--sample-hz and --cmd-hz must be positive")
    if args.drive_duration < 0.0 or args.pre_roll < 0.0 or args.post_roll < 0.0:
        parser.error("--drive-duration, --pre-roll, and --post-roll must be non-negative")
    if args.tf_timeout < 0.0:
        parser.error("--tf-timeout must be non-negative")

    default_pairs = [
        f"{args.map_frame}:{args.base_frame}",
        f"{args.map_frame}:{args.odom_frame}",
        f"{args.odom_frame}:{args.base_frame}",
    ]
    parsed_pairs = parse_tf_pairs(default_pairs + (args.tf_pair or []))
    args.tf_pairs = list(dict.fromkeys(parsed_pairs))
    args.started_at = datetime.now().isoformat(timespec="seconds")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    label = sanitize_label(args.label)
    args.output_dir = Path(args.output_root).expanduser() / f"{timestamp}_{label}"
    return args


def main(argv=None) -> int:
    args = prepare_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=False)

    rclpy.init(args=None)
    node = TrialRecorder(args, args.output_dir)
    try:
        node.run()
    except KeyboardInterrupt:
        node.exit_reason = "interrupted"
        node.get_logger().warn("interrupted, publishing stop commands")
    except Exception as exc:  # Keep the robot stop path active for unexpected failures.
        node.exit_reason = f"error: {exc}"
        node.get_logger().error(str(exc))
        raise
    finally:
        try:
            node.publish_stop(args.stop_repeats, args.stop_period)
            node.write_summary()
        finally:
            node.close()
            node.destroy_node()
            rclpy.shutdown()

    print(f"trial output: {args.output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
