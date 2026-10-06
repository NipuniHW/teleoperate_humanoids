#!/usr/bin/env python3
"""
teleoperation_humanoid/robots/unitree_g1.py

Drives the Unitree G1 via ZED2i BODY_18 skeleton tracking

For visualization, this repo's companion `robot_description` package
provides a G1 URDF and an RViz launch file (see the top-level README):
    ros2 launch robot_description teleop_robot.launch.py organization:=unitree robot_type:=g1

Run:
    ros2 run teleoperation_humanoid teleop_g1
"""

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from zed_msgs.msg import ObjectsStamped

from teleoperation_humanoid.core import body18
from teleoperation_humanoid.core.smoothing import ExponentialSmoother

# ======================================================
# G1 JOINT LIMITS (from main.urdf)
# G1 elbow zero pose: forearm points forward (+X). elbow=pi/2 = arm straight/hanging.
# Formula: pi/2 - arccos(dot) maps person straight->pi/2, person 90deg bent->0.
# ======================================================

JOINT_LIMITS = {
    "left_shoulder_pitch_joint": (-3.0892, 2.6704),
    "left_shoulder_roll_joint": (-1.5882, 2.2515),
    "left_shoulder_yaw_joint": (-2.618, 2.618),
    "left_elbow_joint": (-1.0472, 2.0944),
    "left_wrist_roll_joint": (-1.9722, 1.9722),
    "left_wrist_pitch_joint": (-1.6144, 1.6144),
    "left_wrist_yaw_joint": (-1.6144, 1.6144),
    "right_shoulder_pitch_joint": (-3.0892, 2.6704),
    "right_shoulder_roll_joint": (-2.2515, 1.5882),
    "right_shoulder_yaw_joint": (-2.618, 2.618),
    "right_elbow_joint": (-1.0472, 2.0944),
    "right_wrist_roll_joint": (-1.9722, 1.9722),
    "right_wrist_pitch_joint": (-1.6144, 1.6144),
    "right_wrist_yaw_joint": (-1.6144, 1.6144),
    "waist_yaw_joint": (-2.618, 2.618),
    "waist_roll_joint": (-0.52, 0.52),
    "waist_pitch_joint": (-0.52, 0.52),
}

# robot_state_publisher needs a position for every joint in the URDF, so the
# untracked lower body is held at 0 alongside the driven upper-body joints.
ALL_JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]


def clamp(joint, val):
    mn, mx = JOINT_LIMITS[joint]
    return float(np.clip(val, mn, mx))


def safe_norm(vec):
    return body18.safe_norm(vec)


# ======================================================
# ARMS
# ======================================================


def compute_right_arm(kp_list):
    shoulder = np.asarray(kp_list[body18.IDX_SHOULDER_R], dtype=float)
    elbow = np.asarray(kp_list[body18.IDX_ELBOW_R], dtype=float)
    wrist = np.asarray(kp_list[body18.IDX_WRIST_R], dtype=float)
    hip_r = np.asarray(kp_list[body18.IDX_HIP_R], dtype=float)
    hip_l = np.asarray(kp_list[body18.IDX_HIP_L], dtype=float)

    if np.linalg.norm(shoulder) < 1e-6 or np.linalg.norm(elbow) < 1e-6:
        return {}
    vrse_n = safe_norm(elbow - shoulder)
    if vrse_n is None:
        return {}

    Px, Py, Pz = vrse_n
    shoulder_pitch = np.arctan2(Px, -Pz)

    vhip_n = safe_norm(hip_r - hip_l)
    shoulder_roll = 0.0 if vhip_n is None else -(
        np.pi / 2 - np.arccos(np.clip(np.dot(vrse_n, vhip_n), -1.0, 1.0))
    )

    result = {
        "right_shoulder_pitch_joint": clamp("right_shoulder_pitch_joint", shoulder_pitch),
        "right_shoulder_roll_joint": clamp("right_shoulder_roll_joint", shoulder_roll),
    }
    if np.linalg.norm(wrist) > 1e-6:
        vrew_n = safe_norm(wrist - elbow)
        if vrew_n is not None:
            dot = np.clip(np.dot(vrew_n, vrse_n), -1.0, 1.0)
            elbow_angle = np.arccos(dot)
            result["right_elbow_joint"] = clamp("right_elbow_joint", np.pi / 2 - elbow_angle)
            # shoulder_yaw (forearm twist) is not observable from BODY_18
            # keypoints — the cross-product estimate is noisy near
            # degenerate poses and flips the palm orientation
            # unpredictably, so it is held at its neutral (0) value.
    return result


def compute_left_arm(kp_list):
    shoulder = np.asarray(kp_list[body18.IDX_SHOULDER_L], dtype=float)
    elbow = np.asarray(kp_list[body18.IDX_ELBOW_L], dtype=float)
    wrist = np.asarray(kp_list[body18.IDX_WRIST_L], dtype=float)
    hip_r = np.asarray(kp_list[body18.IDX_HIP_R], dtype=float)
    hip_l = np.asarray(kp_list[body18.IDX_HIP_L], dtype=float)

    if np.linalg.norm(shoulder) < 1e-6 or np.linalg.norm(elbow) < 1e-6:
        return {}
    vlse_n = safe_norm(elbow - shoulder)
    if vlse_n is None:
        return {}

    Px, Py, Pz = vlse_n
    shoulder_pitch = np.arctan2(Px, -Pz)

    vhip_n = safe_norm(hip_l - hip_r)
    shoulder_roll = 0.0 if vhip_n is None else (
        np.pi / 2 - np.arccos(np.clip(np.dot(vlse_n, vhip_n), -1.0, 1.0))
    )

    result = {
        "left_shoulder_pitch_joint": clamp("left_shoulder_pitch_joint", shoulder_pitch),
        "left_shoulder_roll_joint": clamp("left_shoulder_roll_joint", shoulder_roll),
    }
    if np.linalg.norm(wrist) > 1e-6:
        vlew_n = safe_norm(wrist - elbow)
        if vlew_n is not None:
            dot = np.clip(np.dot(vlew_n, vlse_n), -1.0, 1.0)
            elbow_angle = np.arccos(dot)
            result["left_elbow_joint"] = clamp("left_elbow_joint", np.pi / 2 - elbow_angle)
            # shoulder_yaw (forearm twist) is not observable from BODY_18
            # keypoints — the cross-product estimate is noisy near
            # degenerate poses and flips the palm orientation
            # unpredictably, so it is held at its neutral (0) value.
    return result


# ======================================================
# TORSO
# ======================================================


def compute_torso(kp_list):
    hip_r = np.asarray(kp_list[body18.IDX_HIP_R], dtype=float)
    hip_l = np.asarray(kp_list[body18.IDX_HIP_L], dtype=float)
    shldr_r = np.asarray(kp_list[body18.IDX_SHOULDER_R], dtype=float)
    shldr_l = np.asarray(kp_list[body18.IDX_SHOULDER_L], dtype=float)

    if (
        np.linalg.norm(hip_r) < 1e-6
        or np.linalg.norm(hip_l) < 1e-6
        or np.linalg.norm(shldr_r) < 1e-6
        or np.linalg.norm(shldr_l) < 1e-6
    ):
        return {}
    spine = safe_norm((shldr_r + shldr_l) / 2.0 - (hip_r + hip_l) / 2.0)
    if spine is None:
        return {}
    return {
        "waist_pitch_joint": clamp("waist_pitch_joint", np.arctan2(-spine[0], spine[2])),
        "waist_roll_joint": clamp("waist_roll_joint", np.arctan2(spine[1], spine[2])),
    }


# ======================================================
# ROS 2 SUBSCRIBER NODE
# ======================================================


class TeleoperateG1Node(Node):
    def __init__(self, skeleton_topic: str, joint_state_topic: str):
        super().__init__("teleop_g1")
        self._js_pub = self.create_publisher(JointState, joint_state_topic, 10)
        self._smoother = ExponentialSmoother(alpha=0.3)
        self._current = {j: 0.0 for j in ALL_JOINT_NAMES}
        self._frame = 0
        self._publish()
        self.create_subscription(ObjectsStamped, skeleton_topic, self._cb, 10)
        self.get_logger().info(f"G1 teleop ready — publishing to {joint_state_topic}")

    def _cb(self, msg):
        kp_list = body18.extract_keypoints(msg)
        if kp_list is None:
            return
        if np.linalg.norm(kp_list[body18.IDX_PELVIS_PROXY]) < 1e-6:
            return
        if np.linalg.norm(kp_list[body18.IDX_SHOULDER_R]) < 1e-6:
            return

        angles = {}
        angles.update(compute_right_arm(kp_list))
        angles.update(compute_left_arm(kp_list))
        angles.update(compute_torso(kp_list))
        angles = self._smoother.smooth(angles)

        self._current.update(angles)
        self._publish()

        if self._frame % 30 == 0:
            self.get_logger().info(
                f"Frame {self._frame}: "
                f"R_SP={angles.get('right_shoulder_pitch_joint', 0):.2f}  "
                f"L_SP={angles.get('left_shoulder_pitch_joint', 0):.2f}  "
                f"W_R={angles.get('waist_roll_joint', 0):.2f}"
            )
        self._frame += 1

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINT_NAMES
        msg.position = [self._current[j] for j in ALL_JOINT_NAMES]
        self._js_pub.publish(msg)


# ======================================================
# MAIN
# ======================================================


def main():
    parser = argparse.ArgumentParser(description="Teleoperate Unitree G1 from ZED2i BODY_18 skeleton tracking.")
    parser.add_argument(
        "--skeleton-topic",
        default="/zed/zed_node/body_trk/skeletons",
        help="ObjectsStamped topic to subscribe to",
    )
    parser.add_argument(
        "--joint-state-topic",
        default="/joint_states",
        help="JointState topic to publish to",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TeleoperateG1Node(args.skeleton_topic, args.joint_state_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
