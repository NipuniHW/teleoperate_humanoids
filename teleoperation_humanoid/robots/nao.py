#!/usr/bin/env python3
"""
teleoperation_humanoid/robots/nao.py

Drives SoftBank/Aldebaran NAO (H25) via ZED2i BODY_18 skeleton tracking.

Requires the SoftBank/Aldebaran NAOqi `qi` Python SDK, which is not
pip-installable — see the package README for how to obtain it.

Run:
    ros2 run teleoperation_humanoid teleop_nao --robot-ip <ip> --robot-port 9559
"""

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped

from teleoperation_humanoid.core import body18
from teleoperation_humanoid.core.smoothing import ExponentialSmoother

# ======================================================
# JOINT LIMITS (NAO H25)
# ======================================================

JOINT_LIMITS = {
    "HeadYaw": (-2.0857, 2.0857),
    "HeadPitch": (-0.6720, 0.5149),
    "LShoulderPitch": (-2.0857, 2.0857),
    "LShoulderRoll": (-0.3142, 1.3265),
    "LElbowYaw": (-2.0857, 2.0857),
    "LElbowRoll": (-1.5620, -0.0087),
    "LWristYaw": (-1.8238, 1.8238),
    "LHand": (0.0, 1.0),
    "RShoulderPitch": (-2.0857, 2.0857),
    "RShoulderRoll": (-1.3265, 0.3142),
    "RElbowYaw": (-2.0857, 2.0857),
    "RElbowRoll": (0.0087, 1.5620),
    "RWristYaw": (-1.8238, 1.8238),
    "RHand": (0.0, 1.0),
}


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
    if Px > 0:
        shoulder_pitch = np.arctan2(-Pz, Px)
    elif Px < 0 and Pz < 0:
        shoulder_pitch = -np.arctan2(-Px, -Pz) + np.pi / 2
    else:
        shoulder_pitch = -np.arctan2(-Px, -Pz) - np.pi / 2

    vhip_n = safe_norm(hip_r - hip_l)
    if vhip_n is None:
        shoulder_roll = 0.0
    else:
        dot = np.clip(np.dot(vrse_n, vhip_n), -1.0, 1.0)
        shoulder_roll = -(np.pi / 2 - np.arccos(dot))

    result = {
        "RShoulderPitch": clamp("RShoulderPitch", shoulder_pitch),
        "RShoulderRoll": clamp("RShoulderRoll", shoulder_roll),
    }
    if np.linalg.norm(wrist) > 1e-6:
        vrew_n = safe_norm(wrist - elbow)
        if vrew_n is not None:
            dot = np.clip(np.dot(vrew_n, vrse_n), -1.0, 1.0)
            result["RElbowRoll"] = clamp("RElbowRoll", np.arccos(dot))
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
    if Px > 0:
        shoulder_pitch = np.arctan2(-Pz, Px)
    elif Px < 0 and Pz < 0:
        shoulder_pitch = -np.arctan2(-Px, -Pz) + np.pi / 2
    else:
        shoulder_pitch = -np.arctan2(-Px, -Pz) - np.pi / 2

    vhip_n = safe_norm(hip_l - hip_r)
    if vhip_n is None:
        shoulder_roll = 0.0
    else:
        dot = np.clip(np.dot(vlse_n, vhip_n), -1.0, 1.0)
        shoulder_roll = np.pi / 2 - np.arccos(dot)

    result = {
        "LShoulderPitch": clamp("LShoulderPitch", shoulder_pitch),
        "LShoulderRoll": clamp("LShoulderRoll", shoulder_roll),
    }
    if np.linalg.norm(wrist) > 1e-6:
        vlew_n = safe_norm(wrist - elbow)
        if vlew_n is not None:
            dot = np.clip(np.dot(vlew_n, vlse_n), -1.0, 1.0)
            result["LElbowRoll"] = clamp("LElbowRoll", -np.arccos(dot))
    return result


# ======================================================
# HEAD
# ======================================================


def compute_head(kp_list):
    nose = np.asarray(kp_list[body18.IDX_NOSE], dtype=float)
    r_ear = np.asarray(kp_list[body18.IDX_EAR_R], dtype=float)
    l_ear = np.asarray(kp_list[body18.IDX_EAR_L], dtype=float)
    r_eye = np.asarray(kp_list[body18.IDX_EYE_R], dtype=float)
    l_eye = np.asarray(kp_list[body18.IDX_EYE_L], dtype=float)
    neck = np.asarray(kp_list[body18.IDX_NECK], dtype=float)
    r_shldr = np.asarray(kp_list[body18.IDX_SHOULDER_R], dtype=float)
    l_shldr = np.asarray(kp_list[body18.IDX_SHOULDER_L], dtype=float)
    hip_r = np.asarray(kp_list[body18.IDX_HIP_R], dtype=float)
    hip_l = np.asarray(kp_list[body18.IDX_HIP_L], dtype=float)

    ears_valid = np.linalg.norm(r_ear) > 1e-6 and np.linalg.norm(l_ear) > 1e-6
    eyes_valid = np.linalg.norm(r_eye) > 1e-6 and np.linalg.norm(l_eye) > 1e-6
    if ears_valid:
        ref_r, ref_l = r_ear, l_ear
    elif eyes_valid:
        ref_r, ref_l = r_eye, l_eye
    else:
        return {}
    if np.linalg.norm(nose) < 1e-6:
        return {}

    face_fwd = safe_norm(nose - (ref_r + ref_l) / 2.0)
    if face_fwd is None:
        return {}

    if np.linalg.norm(r_shldr) < 1e-6 or np.linalg.norm(l_shldr) < 1e-6:
        return {}
    if np.linalg.norm(neck) < 1e-6:
        return {}

    if np.linalg.norm(hip_r) > 1e-6 and np.linalg.norm(hip_l) > 1e-6:
        hip_mid = (hip_r + hip_l) / 2.0
    elif np.linalg.norm(hip_r) > 1e-6:
        hip_mid = hip_r
    elif np.linalg.norm(hip_l) > 1e-6:
        hip_mid = hip_l
    else:
        return {}

    body_right = safe_norm(r_shldr - l_shldr)
    body_up_rough = safe_norm(neck - hip_mid)
    if body_right is None or body_up_rough is None:
        return {}

    body_fwd = safe_norm(np.cross(body_up_rough, body_right))
    if body_fwd is None:
        return {}
    body_up = safe_norm(np.cross(body_right, body_fwd))
    if body_up is None:
        return {}

    ff_fwd = np.dot(face_fwd, body_fwd)
    ff_right = np.dot(face_fwd, body_right)
    ff_up = np.dot(face_fwd, body_up)

    return {
        "HeadYaw": clamp("HeadYaw", np.arctan2(-ff_right, ff_fwd)),
        "HeadPitch": clamp("HeadPitch", np.arctan2(-ff_up, ff_fwd)),
    }


# ======================================================
# HANDS
# ======================================================


def compute_hands(kp_list):
    result = {}
    for shoulder_idx, elbow_idx, wrist_idx, joint in [
        (body18.IDX_SHOULDER_R, body18.IDX_ELBOW_R, body18.IDX_WRIST_R, "RHand"),
        (body18.IDX_SHOULDER_L, body18.IDX_ELBOW_L, body18.IDX_WRIST_L, "LHand"),
    ]:
        shoulder = np.asarray(kp_list[shoulder_idx], dtype=float)
        elbow = np.asarray(kp_list[elbow_idx], dtype=float)
        wrist = np.asarray(kp_list[wrist_idx], dtype=float)

        if (
            np.linalg.norm(shoulder) < 1e-6
            or np.linalg.norm(elbow) < 1e-6
            or np.linalg.norm(wrist) < 1e-6
        ):
            continue

        v_upper = safe_norm(elbow - shoulder)
        v_fore = safe_norm(wrist - elbow)
        if v_upper is None or v_fore is None:
            continue

        elbow_angle = np.arccos(np.clip(np.dot(v_upper, v_fore), -1.0, 1.0))
        openness = 1.0 - elbow_angle / np.pi
        result[joint] = clamp(joint, openness)

    return result


# ======================================================
# CONNECTION
# ======================================================


def connect_nao(robot_ip: str, robot_port: int):
    """
    Connect to NAO via NAOqi. The session must be kept alive by the caller
    for the connection's lifetime — if it goes out of scope the socket
    closes and all subsequent service calls fail.
    """
    import qi

    session = qi.Session()
    session.connect(f"tcp://{robot_ip}:{robot_port}")
    motion = session.service("ALMotion")
    posture = session.service("ALRobotPosture")
    motion.wakeUp()
    posture.goToPosture("StandInit", 0.5)
    return session, motion, posture


def send_angles(motion, angles: dict, speed: float = 0.15) -> bool:
    """Send joint angles to NAO. Returns False if the socket dropped."""
    names, values = [], []
    for k, val in angles.items():
        val = float(val)
        if np.isnan(val) or np.isinf(val):
            continue
        names.append(k)
        values.append(val)
    if names:
        try:
            motion.setAngles(names, values, speed)
        except RuntimeError as e:
            print(f"[teleop_nao] Lost connection to NAO: {e}")
            return False
    return True


# ======================================================
# ROS 2 SUBSCRIBER NODE
# ======================================================


class TeleoperateNaoNode(Node):
    def __init__(self, motion, skeleton_topic: str):
        super().__init__("teleop_nao")
        self._motion = motion
        self._smoother = ExponentialSmoother(alpha=0.3)
        self._frame_idx = 0
        self.create_subscription(ObjectsStamped, skeleton_topic, self._skeleton_cb, 10)
        self.get_logger().info(f"Subscribed to {skeleton_topic} — driving NAO.")

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
        angles.update(compute_head(kp_list))
        angles.update(compute_hands(kp_list))
        angles = self._smoother.smooth(angles)

        if not send_angles(self._motion, angles, speed=0.15):
            self.get_logger().error("NAO socket disconnected — stopping spin.")
            raise SystemExit

        if self._frame_idx % 30 == 0:
            self.get_logger().info(
                f"Frame {self._frame_idx}: "
                f"RSP={angles.get('RShoulderPitch', 0):.2f}  "
                f"RSR={angles.get('RShoulderRoll', 0):.2f}  "
                f"RER={angles.get('RElbowRoll', 0):.2f}  "
                f"LER={angles.get('LElbowRoll', 0):.2f}  "
                f"HY={angles.get('HeadYaw', 0):.2f}  "
                f"HP={angles.get('HeadPitch', 0):.2f}  "
                f"RH={angles.get('RHand', 0):.2f}  "
                f"LH={angles.get('LHand', 0):.2f}"
            )
        self._frame_idx += 1


# ======================================================
# MAIN
# ======================================================


def main():
    parser = argparse.ArgumentParser(description="Teleoperate NAO from ZED2i BODY_18 skeleton tracking.")
    parser.add_argument("--robot-ip", default="localhost", help="NAO's IP address (default: localhost)")
    parser.add_argument("--robot-port", type=int, default=9559, help="NAOqi port (default: 9559, the standard NAOqi port)")
    parser.add_argument(
        "--skeleton-topic",
        default="/zed/zed_node/body_trk/skeletons",
        help="ObjectsStamped topic to subscribe to",
    )
    args = parser.parse_args()

    session, motion, posture = connect_nao(args.robot_ip, args.robot_port)

    rclpy.init()
    node = TeleoperateNaoNode(motion, args.skeleton_topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            print("Returning to StandInit.")
            posture.goToPosture("StandInit", 0.5)
        except RuntimeError:
            print("Could not return to StandInit — socket already closed.")


if __name__ == "__main__":
    main()
