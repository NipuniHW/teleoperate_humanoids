#!/usr/bin/env python3
"""
teleoperation_humanoid/robots/agibot_x2.py

Drives the AgiBot X2 via ZED2i BODY_18 skeleton tracking, by publishing
sensor_msgs/JointState. This backend has no direct hardware/sim binding of
its own — pair it with a URDF + robot_state_publisher (RViz) or your own
sim/hardware bridge that consumes /joint_states.

For visualization, this repo's companion `robot_description` package
provides an X2 URDF and an RViz launch file (see the top-level README):
    ros2 launch robot_description teleop_robot.launch.py organization:=agibot robot_type:=x2

Run:
    ros2 run teleoperation_humanoid teleop_agibot_x2
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
# AGIBOT X2 JOINT LIMITS (from mjcf/agibot/x2.xml)
# ======================================================
#
# Note: head_yaw_joint and head_pitch_joint are commented
# out in the X2 MJCF — they are not controllable in sim.
# Both elbow joints share the same range (0=straight,
# negative=bent), unlike Pepper where arms have opposite signs.
# ======================================================

JOINT_LIMITS = {
    "head_yaw_joint": (-0.366, 0.366),
    "head_pitch_joint": (-0.384, 0.384),
    "left_shoulder_pitch_joint": (-3.08, 2.04),
    "left_shoulder_roll_joint": (-0.061, 2.993),
    "left_shoulder_yaw_joint": (-2.556, 2.556),
    "left_elbow_joint": (-2.3556, 0.0),
    "left_wrist_yaw_joint": (-2.556, 2.556),
    "left_wrist_pitch_joint": (-0.558, 0.558),
    "left_wrist_roll_joint": (-1.571, 0.724),
    "right_shoulder_pitch_joint": (-3.08, 2.04),
    "right_shoulder_roll_joint": (-2.993, 0.061),
    "right_shoulder_yaw_joint": (-2.556, 2.556),
    "right_elbow_joint": (-2.3556, 0.0),
    "right_wrist_yaw_joint": (-2.556, 2.556),
    "right_wrist_pitch_joint": (-0.558, 0.558),
    "right_wrist_roll_joint": (-0.724, 1.571),
    "waist_yaw_joint": (-3.43, 2.382),
    "waist_pitch_joint": (-0.314, 0.314),
    "waist_roll_joint": (-0.488, 0.488),
}

# All 29 revolute joints in the URDF — robot_state_publisher requires every
# joint to be present in each JointState message or it glitches.
# Leg joints are held at 0 (not driven by ZED); upper-body joints are computed.
ALL_JOINT_NAMES = [
    "head_yaw_joint", "head_pitch_joint",
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
]


def clamp(joint, val):
    mn, mx = JOINT_LIMITS[joint]
    return float(np.clip(val, mn, mx))


def safe_norm(vec):
    return body18.safe_norm(vec)


def _shoulder_yaw(upper_arm_n, forearm_n, sign=1.0):
    """
    Estimate shoulder_yaw from the plane in which the elbow bends. Currently
    unused (not wired into compute_right_arm/compute_left_arm below) — kept
    as a documented starting point for anyone extending this backend with a
    shoulder-yaw estimate.

    At yaw=0 the elbow bends in the plane containing the upper arm and
    robot-forward (ZED -X = toward camera). Falls back to ZED +Z (up) when
    the arm points along the forward axis (degenerate case).
    """
    actual_normal = safe_norm(np.cross(upper_arm_n, forearm_n))
    if actual_normal is None:
        return 0.0
    for ref_vec in (np.array([-1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])):
        ref_normal = safe_norm(np.cross(upper_arm_n, ref_vec))
        if ref_normal is not None:
            break
    else:
        return 0.0
    dot = np.clip(np.dot(actual_normal, ref_normal), -1.0, 1.0)
    cross_v = np.cross(ref_normal, actual_normal)
    s = np.sign(np.dot(cross_v, upper_arm_n))
    return sign * s * np.arccos(dot)


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
    # atan2(Px, -Pz): arm-down -> 0, arm-toward-camera -> -pi/2 (robot
    # forward), arm-away-from-camera -> +pi/2 (robot backward). ZED X=depth,
    # so arm toward camera has Px<0 -> negative pitch -> robot arm swings
    # forward.
    shoulder_pitch = np.arctan2(Px, -Pz)

    vhip_n = safe_norm(hip_r - hip_l)
    if vhip_n is None:
        shoulder_roll = 0.0
    else:
        dot = np.clip(np.dot(vrse_n, vhip_n), -1.0, 1.0)
        shoulder_roll = -(np.pi / 2 - np.arccos(dot))

    result = {
        "right_shoulder_pitch_joint": clamp("right_shoulder_pitch_joint", shoulder_pitch),
        "right_shoulder_roll_joint": clamp("right_shoulder_roll_joint", shoulder_roll),
    }
    if np.linalg.norm(wrist) > 1e-6:
        vrew_n = safe_norm(wrist - elbow)
        if vrew_n is not None:
            dot = np.clip(np.dot(vrew_n, vrse_n), -1.0, 1.0)
            result["right_elbow_joint"] = clamp("right_elbow_joint", -np.arccos(dot))
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
    if vhip_n is None:
        shoulder_roll = 0.0
    else:
        dot = np.clip(np.dot(vlse_n, vhip_n), -1.0, 1.0)
        shoulder_roll = np.pi / 2 - np.arccos(dot)

    result = {
        "left_shoulder_pitch_joint": clamp("left_shoulder_pitch_joint", shoulder_pitch),
        "left_shoulder_roll_joint": clamp("left_shoulder_roll_joint", shoulder_roll),
    }
    if np.linalg.norm(wrist) > 1e-6:
        vlew_n = safe_norm(wrist - elbow)
        if vlew_n is not None:
            dot = np.clip(np.dot(vlew_n, vlse_n), -1.0, 1.0)
            result["left_elbow_joint"] = clamp("left_elbow_joint", -np.arccos(dot))
    return result


# ======================================================
# HEAD — yaw (Z axis, +-0.366 rad) and pitch (Y axis, +-0.384 rad)
# ======================================================


def compute_head(kp_list):
    nose = np.asarray(kp_list[body18.IDX_NOSE], dtype=float)
    neck = np.asarray(kp_list[body18.IDX_NECK], dtype=float)

    if np.linalg.norm(nose) < 1e-6 or np.linalg.norm(neck) < 1e-6:
        return {}

    vec = nose - neck
    dist = np.linalg.norm(vec)
    if dist < 0.02:
        return {}

    vx, vy, vz = vec / dist  # normalised nose-relative-to-neck direction

    # forward: how much the nose is in front of (closer to camera than) the
    # neck. ZED X = depth, so nose closer to camera -> vx < 0 -> forward =
    # -vx > 0.
    forward = max(-vx, 0.05)

    # Yaw (Z axis): person turns right -> nose shifts to +Y (ZED) -> negative
    # yaw (X2 positive yaw = turn robot-left, so negate vy to match sides).
    head_yaw = np.arctan2(-vy, forward)

    # Pitch (Y axis): X2 positive pitch rotates nose DOWN (right-hand rule,
    # axis +Y). So negate: person looks up (vz>0) -> negative pitch -> robot
    # nose goes up.
    head_pitch = -(np.arctan2(vz, forward) - np.pi / 4)

    return {
        "head_yaw_joint": clamp("head_yaw_joint", head_yaw),
        "head_pitch_joint": clamp("head_pitch_joint", head_pitch),
    }


# ======================================================
# TORSO — waist_pitch and waist_roll
# ======================================================


def compute_torso(kp_list):
    """
    X2 has dedicated waist joints for torso lean:
      waist_pitch_joint (axis Y): forward/back lean
      waist_roll_joint  (axis X): side lean

    ZED frame: X=depth, Y=lateral (person's right = +Y), Z=vertical (+Z=up).
    Spine upright ~ (0, 0, 1).
    """
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

    hip_mid = (hip_r + hip_l) / 2.0
    shldr_mid = (shldr_r + shldr_l) / 2.0
    spine = safe_norm(shldr_mid - hip_mid)
    if spine is None:
        return {}

    waist_pitch = np.arctan2(spine[0], spine[2])  # forward lean
    waist_roll = np.arctan2(spine[1], spine[2])  # lateral lean: person's left = -Y -> negative roll

    return {
        "waist_pitch_joint": clamp("waist_pitch_joint", waist_pitch),
        "waist_roll_joint": clamp("waist_roll_joint", waist_roll),
    }


# ======================================================
# ROS 2 NODE
# ======================================================


class TeleoperateAgibotX2Node(Node):
    def __init__(self, skeleton_topic: str, joint_state_topic: str):
        super().__init__("teleop_agibot_x2")
        self._frame_idx = 0
        self._smoother = ExponentialSmoother(alpha=0.3)

        # Publish JointState — a robot_state_publisher consumes this and
        # drives the RViz visualisation.
        self._js_pub = self.create_publisher(JointState, joint_state_topic, 10)

        # Initialise all joints to zero so RViz shows something immediately.
        self._current = {j: 0.0 for j in ALL_JOINT_NAMES}
        self._publish_joint_state()

        self.create_subscription(ObjectsStamped, skeleton_topic, self._skeleton_cb, 10)
        self.get_logger().info(
            f"Subscribed to {skeleton_topic} — publishing to {joint_state_topic}."
        )

    def _skeleton_cb(self, msg: ObjectsStamped):
        kp_list = body18.extract_keypoints(msg)
        if kp_list is None:
            self._frame_idx += 1
            return

        if np.linalg.norm(kp_list[body18.IDX_PELVIS_PROXY]) < 1e-6:
            self._frame_idx += 1
            return
        if np.linalg.norm(kp_list[body18.IDX_SHOULDER_R]) < 1e-6:
            self._frame_idx += 1
            return

        angles = {}
        angles.update(compute_right_arm(kp_list))
        angles.update(compute_left_arm(kp_list))
        angles.update(compute_torso(kp_list))
        angles.update(compute_head(kp_list))
        angles = self._smoother.smooth(angles)

        self._current.update(angles)
        self._publish_joint_state()

        if self._frame_idx % 30 == 0:
            self.get_logger().info(
                f"Frame {self._frame_idx}: "
                f"R_SP={angles.get('right_shoulder_pitch_joint', 0):.2f}  "
                f"R_SR={angles.get('right_shoulder_roll_joint', 0):.2f}  "
                f"R_EL={angles.get('right_elbow_joint', 0):.2f}  "
                f"L_SP={angles.get('left_shoulder_pitch_joint', 0):.2f}  "
                f"L_SR={angles.get('left_shoulder_roll_joint', 0):.2f}  "
                f"L_EL={angles.get('left_elbow_joint', 0):.2f}  "
                f"W_P={angles.get('waist_pitch_joint', 0):.2f}  "
                f"W_R={angles.get('waist_roll_joint', 0):.2f}"
            )

        self._frame_idx += 1

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINT_NAMES
        msg.position = [self._current[j] for j in ALL_JOINT_NAMES]
        self._js_pub.publish(msg)


# ======================================================
# MAIN
# ======================================================


def main():
    parser = argparse.ArgumentParser(description="Teleoperate AgiBot X2 from ZED2i BODY_18 skeleton tracking.")
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
    node = TeleoperateAgibotX2Node(args.skeleton_topic, args.joint_state_topic)
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
