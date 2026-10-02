"""
teleoperation_humanoid/core/body18.py

ZED2i BODY_18 keypoint constants, geometry helpers, and skeleton extraction.
Shared by every robot backend that consumes the BODY_18 skeleton format
(NAO, Unitree G1, AgiBot X2 in this package).

ZED2i coordinate frame:
  X = depth  (positive = away from camera)
  Y = lateral (positive = person's right when facing camera)
  Z = vertical (positive = up)
"""

import numpy as np
from zed_msgs.msg import ObjectsStamped

NUM_KEYPOINTS = 18

IDX_NOSE = 0
IDX_NECK = 1
IDX_SHOULDER_R = 2
IDX_ELBOW_R = 3
IDX_WRIST_R = 4
IDX_SHOULDER_L = 5
IDX_ELBOW_L = 6
IDX_WRIST_L = 7
IDX_HIP_R = 8
IDX_KNEE_R = 9
IDX_ANKLE_R = 10
IDX_HIP_L = 11
IDX_KNEE_L = 12
IDX_ANKLE_L = 13
IDX_EYE_R = 14
IDX_EYE_L = 15
IDX_EAR_R = 16
IDX_EAR_L = 17

# Convenience: best available pelvis proxy (BODY_18 has no dedicated pelvis point).
IDX_PELVIS_PROXY = IDX_HIP_R


def safe_norm(vec):
    """Return the unit vector, or None if the input is near-zero (untracked)."""
    n = np.linalg.norm(vec)
    return vec / n if n > 1e-6 else None


def extract_keypoints(msg: ObjectsStamped):
    """
    Pick the best-tracked BODY_18 body from an ObjectsStamped message.

    Returns a list of 18 np.array([x, y, z]) in ZED world coordinates. NaN
    keypoints become zero vectors so downstream norm-based validity checks
    treat them as untracked. Returns None when no bodies are detected.
    """
    if not msg.objects:
        return None

    best_obj = max(
        msg.objects,
        key=lambda o: sum(
            0 if np.any(np.isnan(kp.kp)) else 1
            for kp in o.skeleton_3d.keypoints[:NUM_KEYPOINTS]
        ),
    )

    kp_list = []
    for kp in best_obj.skeleton_3d.keypoints[:NUM_KEYPOINTS]:
        arr = np.array(kp.kp, dtype=float)
        kp_list.append(np.zeros(3) if np.any(np.isnan(arr)) else arr)
    return kp_list
