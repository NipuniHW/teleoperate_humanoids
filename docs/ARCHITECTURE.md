# Architecture: `teleoperation_humanoid`

This document describes the internal design of the `teleoperation_humanoid` ROS 2
package: how it's structured, why it's structured that way, how data moves through it,
and how it was validated. It assumes familiarity with the [README](../README.md)'s
usage instructions; this document is about *how the package is built*, not how to run it.

A note on provenance: this package was developed and extracted from a larger
monorepo (internally referred to below as "the parent repository") that also held
several non-ROS-packaged prototype scripts per robot. Every robot backend here was
ported from one of those scripts, not written from scratch — several sections below
reference that history and the specific source script a backend came from. Those
original scripts aren't included in this repository; the references are kept because
they explain *why* a given formula or structural choice is what it is, not because
they're reachable from here.

---

## 1. Purpose and scope

The package turns a ZED2i stereo camera's 3D body-skeleton stream into joint commands
for a humanoid robot, in real time, over ROS 2. It supports four robots today (SoftBank
Pepper, SoftBank/Aldebaran NAO, Unitree G1, AgiBot X2) with materially different output
mechanisms — two drive real hardware over a proprietary SDK, two publish a standard ROS
message for any URDF-based consumer — and is explicitly designed so a fifth, unrelated
humanoid can be added without touching any existing robot's code.

**Hard constraint that shaped every decision below**: this package must be *standalone*.
It does not import from, or depend on, anything else in the repository it ships
alongside (`scripts/`, `teleop/`, etc.). It can be copied out of this repository into a
different workspace and it will still build and run. This is why it duplicates rather
than imports some logic that superficially resembles code elsewhere in the repo — the
duplication is deliberate, not an oversight.

---

## 2. Design principles

- **Adapter pattern, one level deep.** A single robot-agnostic *core* (keypoint
  extraction, geometry helpers, smoothing) is shared by every robot. Each robot then gets
  exactly one *backend* file implementing its own inverse kinematics, joint limits, and
  output sink. There is no further abstraction forcing different robots' IK through a
  common interface — see §9 for why.
- **One file per robot, not one file per concern.** A robot backend is not split into
  `arms.py` / `head.py` / `torso.py` / `hands.py`. Adding a robot means writing (or
  copying and editing) one file, not scaffolding four.
- **Ported, not wrapped.** Every robot backend's math was copied from an already-running,
  independently-validated script elsewhere in the parent repository and only mechanically
  adapted (see §10) — never re-derived from scratch during the port. This is a
  correctness strategy as much as a structural one: it means the port's correctness can
  be checked by diffing outputs against the original, frame-by-frame, on real data,
  rather than by code review alone.
- **No behavioral surprises from refactoring.** Where the port did change something
  (e.g. exponential smoothing moved from a module-level global dict to a per-`Node`
  class instance — see §5.3), the change is chosen specifically to be numerically
  identical given the same call sequence, not just "equivalent in spirit."

---

## 3. Package structure

This is a standard ROS 2 `ament_python` package (not `ament_cmake` — the parent repo's
only other package, `robot_description`, is `ament_cmake` because it ships URDF/mesh
*data* with no Python to build; this package is pure Python, so `ament_python` is the
correct fit, and there's no other Python-packaging precedent in the parent repo to
diverge from).

```
teleoperation_humanoid/                    <- ROS 2 package root
├── package.xml                            <- ament_python package manifest (format 3)
├── setup.py                               <- setuptools entry_points define the 4 executables
├── setup.cfg                              <- ament_python boilerplate (script install dir)
├── resource/teleoperation_humanoid        <- empty marker file `ament_index` requires
├── LICENSE                                <- MIT, own copy (package must stand alone)
├── README.md                              <- user-facing usage docs
├── docs/
│   └── ARCHITECTURE.md                    <- this file
└── teleoperation_humanoid/                <- the actual Python package (same name as
    │                                         the ROS package — ament_python convention)
    ├── __init__.py                        <- empty
    ├── core/                              <- robot-agnostic shared layer (§5)
    │   ├── __init__.py
    │   ├── body18.py                      <- 71 lines — BODY_18 keypoints + extraction
    │   └── smoothing.py                   <- 38 lines — ExponentialSmoother
    └── robots/                            <- one self-contained backend per robot (§6)
        ├── __init__.py
        ├── pepper.py                      <- BODY_18, NAOqi
        ├── nao.py                         <- BODY_18, NAOqi
        ├── unitree_g1.py                  <- BODY_18, JointState publisher
        └── agibot_x2.py                   <- BODY_18, JointState publisher
```

All four backends consume the same BODY_18 skeleton format — there is currently no
BODY_38 (or any richer-format) backend in this package. Pepper originally shipped as a
BODY_38 backend with finger-informed hand control, a fixed clavicle-relative shoulder
frame, and an arm-twist (`ElbowYaw`/`WristYaw`) estimate; it was deliberately reverted to
match `scripts/teleoperate_pepper.py`'s BODY_18 IK exactly, at the explicit request of
the package's author, after the BODY_38 version's calculations were found not to match
expectations against real data. §8.1 covers what Pepper's IK looks like today; nothing
in this document describes the BODY_38 version, since it no longer exists in this
package — if you're looking for it, it's `scripts/body_38/teleoperate_pepper.py` in the
parent repository, which this package's Pepper backend does not use.

`setup.py`'s `console_scripts` entry points are the only place a robot backend is wired
into the package as a runnable command:

```python
entry_points={
    "console_scripts": [
        "teleop_pepper = teleoperation_humanoid.robots.pepper:main",
        "teleop_nao = teleoperation_humanoid.robots.nao:main",
        "teleop_g1 = teleoperation_humanoid.robots.unitree_g1:main",
        "teleop_agibot_x2 = teleoperation_humanoid.robots.agibot_x2:main",
    ],
},
```

`colcon build` turns each of these into an executable script under
`install/teleoperation_humanoid/lib/teleoperation_humanoid/`, which is what `ros2 run
teleoperation_humanoid <name>` actually launches — it is not running `python3
robots/<name>.py` directly, it's running that generated entry-point script, which
imports the module and calls `main()`. This matters practically: editing a robot file
requires a `colcon build` (not just a save) before `ros2 run` picks up the change,
because the installed copy under `install/` is what actually executes.

---

## 4. End-to-end data flow

```
┌──────────────┐
│ ZED2i camera │  (or a ros2 bag recorded from one)
└──────┬───────┘
       │
       ▼
┌─────────────────────────┐
│ zed-ros2-wrapper         │  external package, not part of this repo
│ (Stereolabs)              │
└──────┬───────────────────┘
       │ publishes
       ▼
/zed/zed_node/body_trk/skeletons        (zed_msgs/ObjectsStamped)
       │
       │  subscribed by whichever robot's Node is running
       ▼
┌──────────────────────────────────────────────────────────┐
│ core.body18.extract_keypoints() — pick the best-tracked   │
│ person in the frame, return a list of 18                  │
│ np.array([x, y, z]) keypoints                              │
└──────┬─────────────────────────────────────────────────────┘
       ▼
┌──────────────────────────────────────────────────────────┐
│ robots/<robot>.py: compute_right_arm(), compute_left_arm(),│
│ compute_head(), compute_torso(), compute_hands() — pure   │
│ functions, keypoints in, {joint_name: angle_rad} out      │
└──────┬─────────────────────────────────────────────────────┘
       ▼
┌──────────────────────────────────────────────────────────┐
│ core.smoothing.ExponentialSmoother.smooth() — one first-  │
│ order IIR low-pass filter per joint, held as Node state   │
└──────┬─────────────────────────────────────────────────────┘
       ▼
┌──────────────────────────────────────────────────────────┐
│ robot-specific sink:                                       │
│  • Pepper/NAO: NAOqi ALMotion.setAngles() over a qi.Session│
│  • G1/AgiBot X2: sensor_msgs/JointState publish            │
└──────────────────────────────────────────────────────────┘
```

Everything above the sink is identical in *shape* across all four robots — extract,
compute, smooth, send — even though the concrete functions differ per robot. That
uniform shape is what §9's "copy one file, edit three things" extensibility claim
depends on.

---

## 5. The core layer

### 5.1 `core/body18.py` — BODY_18 keypoints

Defines the 18-keypoint index constants (`IDX_NOSE`, `IDX_SHOULDER_R`, `IDX_HIP_L`, ...),
`safe_norm()` (unit-vector-or-`None` for near-zero vectors — the standard way every
IK function in this package guards against untracked/degenerate keypoints), and
`extract_keypoints(msg)`.

`extract_keypoints` does two things worth calling out explicitly:

1. **Best-person selection.** A `zed_msgs/ObjectsStamped` message can contain multiple
   tracked people; this picks the one with the most non-NaN keypoints in the first 18
   slots, i.e. the best-tracked person, not necessarily the first or closest one.
2. **NaN-to-zero conversion.** An untracked keypoint arrives as `NaN` in the message;
   this converts it to `np.zeros(3)`, so every downstream validity check in every robot
   backend can be a uniform `np.linalg.norm(point) > 1e-6` rather than a NaN check. This
   is why every IK function in every backend guards with a norm threshold rather than
   `math.isnan` — it's relying on this conversion having already happened.

`IDX_PELVIS_PROXY = IDX_HIP_R` exists because BODY_18 has no dedicated pelvis keypoint;
the right hip stands in for it wherever a "is a person tracked at all" gate is needed
(every `Node`'s skeleton callback checks this before doing any IK work).

### 5.2 `core/smoothing.py` — `ExponentialSmoother`

```python
class ExponentialSmoother:
    def __init__(self, alpha: float = 0.3): ...
    def smooth(self, angles: dict, alpha_overrides: dict = None) -> dict: ...
    def reset(self): ...
```

A first-order IIR low-pass filter, one independent state value per joint name:
`smoothed[t] = alpha * raw[t] + (1 - alpha) * smoothed[t-1]`. `alpha=1.0` is unfiltered;
`alpha=0.0` never updates past the first observation. `alpha_overrides` lets a caller
smooth specific joints harder than the default — used by `robots/pepper.py` to damp its
noisier arm-twist estimates (§8.1) without affecting every other joint's responsiveness.

This is a class, not a module-level function with a global `dict`, specifically so that
multiple robots (or multiple instances of the same robot) can run in one process without
one's filter history contaminating another's — each `Node` constructs its own
`ExponentialSmoother` instance in `__init__`. The math is unchanged from the
single-global-dict version this was ported from; only the state's *scope* changed.

---

## 6. Robot backend layer — the common internal shape

Every file under `robots/` follows the same internal structure, in the same order, even
though the concrete robot-specific pieces differ:

1. **Module docstring** — what robot, what skeleton format, what output sink, the exact
   `ros2 run` command.
2. **`JOINT_LIMITS`** — `{joint_name: (min_rad, max_rad)}`, sourced from the robot's own
   documentation/URDF, plus a `clamp(joint, val)` that clips into it. Every angle this
   file ever emits passes through `clamp()` before being returned — there is no code path
   that sends an unclamped value to a robot.
3. **Geometry helpers specific to this file** (if any) — most robots don't need anything
   beyond what `core.body18` already provides (`safe_norm` plus the index constants);
   AgiBot X2 is the one exception, carrying an unused `_shoulder_yaw()` helper (§8.3).
4. **IK functions**: `compute_right_arm`, `compute_left_arm`, `compute_head`,
   `compute_torso` (where applicable), `compute_hands` (Pepper/NAO only — G1/AgiBot X2
   have no end-effector actuation to speak of in this pipeline). Each is a **pure
   function**: keypoint list in, a `{joint_name: angle}` dict out, `{}` if the relevant
   keypoints aren't tracked. None of these functions touch ROS, the network, or any I/O —
   this is what made bit-for-bit validation against the originals tractable (§10).
5. **Connection / sink**:
   - Pepper/NAO: `connect_<robot>(robot_ip, robot_port)` opens a `qi.Session`, fetches
     `ALMotion`/`ALRobotPosture`, calls `wakeUp()` and `goToPosture("StandInit", 0.5)`,
     returns `(session, motion, posture)`. `send_angles(motion, angles, speed)` pushes a
     `{joint: angle}` dict via `motion.setAngles(...)`, returning `False` (rather than
     raising) if the socket has dropped, so the caller can shut down cleanly instead of
     crashing mid-motion.
   - G1/AgiBot X2: no connection function at all — the `Node` itself owns a
     `create_publisher(JointState, ...)` and republishes the full `ALL_JOINT_NAMES` set
     (driven joints from IK, everything else — legs, mostly — pinned at `0.0`) every time
     new angles arrive, because `robot_state_publisher` requires every joint present in
     every message or it produces TF errors for the missing ones.
6. **`Node` subclass** — constructor takes whatever the sink needs (a `motion` proxy, or
   topic names) plus subscribes to the skeleton topic; the subscription callback does the
   extract → per-limb `compute_*` calls → `ExponentialSmoother.smooth()` → send, in that
   fixed order, with an early return at each stage if the required keypoints aren't
   tracked yet (pelvis-proxy and right-shoulder are the two gates checked before any IK
   is attempted at all).
7. **`main()`** — `argparse` for `--robot-ip`/`--robot-port` (Pepper/NAO) or
   `--skeleton-topic`/`--joint-state-topic` (G1/AgiBot X2), connects (Pepper/NAO only,
   and deliberately *inside* `main()`, not at module import time — see §6.1), then
   `rclpy.init()` / construct the `Node` / `rclpy.spin()` / teardown.

### 6.1 Why connection happens inside `main()`, not at import time

The scripts these backends were ported from connected to the robot as a **module-level
side effect** — `qi.Session().connect(...)` ran the moment the file was imported, before
`main()` or any class was even defined. That pattern is convenient for a one-off script
but actively hostile to anything that wants to `import` the module without triggering a
real robot connection — including this package's own validation tooling (§10), which
imports every robot module and stubs `qi` to exercise the pure IK functions without a
robot present. `connect_pepper()`/`connect_nao()` in this package are ordinary functions
called explicitly from `main()` after argument parsing, specifically so the module is
safely importable on its own. (This exact pattern — connect-inside-a-function-called-
from-`main`, not connect-at-import — already existed once in the parent repository,
in a more modular but BODY_18-only Pepper implementation; this package generalizes it to
every backend rather than inventing it fresh.)

---

## 7. ROS 2 integration details

- **Message types**: `zed_msgs/ObjectsStamped` in (external package — the ZED ROS 2
  wrapper's custom skeleton message), `sensor_msgs/JointState` out for G1/AgiBot X2;
  Pepper/NAO have no ROS message out at all, since NAOqi is not a ROS system — the `Node`
  exists purely to receive the skeleton topic and drive an out-of-band SDK call.
- **Topics**: every backend defaults to subscribing
  `/zed/zed_node/body_trk/skeletons`, overridable per-instance via `--skeleton-topic` —
  there is no hardcoded topic name inside any IK or Node logic, only in `argparse`
  defaults, so running multiple robots against different topics (e.g. two cameras, or a
  camera plus a bag) in the same `ROS_DOMAIN_ID` is just a CLI flag, not a code change.
- **QoS**: all subscriptions use the default `rclpy` QoS depth of `10` — this was
  inherited unchanged from the original scripts and has not been tuned; see §12.
- **Node naming**: each backend's `Node` has a fixed name (`teleop_pepper`, `teleop_nao`,
  `teleop_g1`, `teleop_agibot_x2`) matching its console-script name, so `ros2 node list`
  / `ros2 node info` are self-explanatory without cross-referencing `setup.py`.
- **`package.xml` dependencies**: `rclpy`, `sensor_msgs`, `zed_msgs`,
  `python3-numpy` are declared `exec_depend`. The NAOqi `qi` SDK is **deliberately not
  declared** anywhere in `package.xml` — it isn't a rosdep-resolvable dependency, and its
  behavior has shown real environment sensitivity in practice (see README §Troubleshooting
  for a connection failure that was specific to one devcontainer and absent on the host
  with the identical robot session), so there's no single "correct" dependency spec to
  declare here even if the tooling supported it.

---

## 8. Per-robot architecture notes

### 8.1 Pepper (`robots/pepper.py`)

Pepper's IK is ported from `scripts/teleoperate_pepper.py` in the parent repository —
the BODY_18 implementation, not the BODY_38 one. An earlier version of this backend used
BODY_38 (a superset skeleton format with a real pelvis, clavicles, and per-hand finger
landmarks), with a clavicle-relative shoulder frame, a finger-informed hand-openness
estimate, and an `ElbowYaw`/`WristYaw` arm-twist estimate that BODY_18 can't provide at
all. It was deliberately reverted to the plain BODY_18 IK, at the explicit request of the
package's author, after the BODY_38 version's output was found not to match expectations
against real data in practice — the previous version of this document described that
BODY_38 design in detail; none of it applies to the current code, and this section
intentionally doesn't reconstruct it.

What's here today: `compute_right_arm`/`compute_left_arm` compute `ShoulderPitch` via a
piecewise `atan2` over the shoulder→elbow vector's depth(X)/vertical(Z) components, and
`ShoulderRoll` via the angle between that vector and a hip-to-hip reference vector
(`-(pi/2 - acos(dot))` for the right arm, `pi/2 - acos(dot)` for the left — the sign flip
is because "outward" is the opposite hip direction for each arm). `ElbowRoll` is
`acos(dot(forearm, upper_arm))`, sign-flipped per arm to match Pepper's joint convention
(`RElbowRoll` positive, `LElbowRoll` negative). `compute_head` builds a body frame from
shoulders/neck/hip-midpoint (not from clavicles — BODY_18 has none) and projects the
face-forward vector (nose relative to ear/eye midpoint) onto it for `HeadYaw`/`HeadPitch`.
`compute_torso` gives `HipPitch`/`HipRoll` from the hip-midpoint→shoulder-midpoint spine
vector. `compute_hands` is the elbow-extension proxy described in §8.2's NAO section —
identical logic, since BODY_18 gives both robots the same information to work with.

`RElbowYaw`/`RWristYaw`/`LElbowYaw`/`LWristYaw` are declared in `JOINT_LIMITS` (Pepper's
hardware does have these joints) but are never computed or sent — BODY_18 doesn't provide
enough keypoints to estimate forearm/wrist twist, so NAOqi just holds whatever value it
last had for them. This mirrors G1's own documented stance on the equivalent joint
(`unitree_g1.py`'s `compute_right_arm`/`compute_left_arm` carry an explicit comment:
"shoulder_yaw (forearm twist) is not observable from BODY_18 keypoints ... held at its
neutral (0) value") rather than being a gap unique to Pepper — see §8.3.

### 8.2 NAO (`robots/nao.py`)

Structurally the simplest backend: BODY_18, NAOqi sink, no torso IK (NAO's head/shoulder/
elbow/hand set has no equivalent of Pepper's `HipPitch`/`HipRoll` torso joints in this
pipeline). This is the file the README's [Adapting to a new humanoid] guide recommends
starting from, specifically because it has the fewest moving parts of the four.

### 8.3 Unitree G1 / AgiBot X2 (`robots/unitree_g1.py`, `robots/agibot_x2.py`)

Both publish `sensor_msgs/JointState` rather than driving anything directly — from this
package's point of view, "the robot" is whatever's downstream consuming that topic
(`robot_state_publisher` + RViz in the documented setup, but nothing in this package
assumes that specifically). Both maintain an `ALL_JOINT_NAMES` list broader than what IK
actually computes (untracked lower-body joints are held at `0.0`) because
`robot_state_publisher` requires a position for every joint in the URDF in every message,
or it reports TF errors for whichever joints are missing. AgiBot X2 additionally computes
head yaw/pitch directly from the nose/neck vector (no equivalent exists in the Pepper/NAO
files, which get head orientation from `compute_head`'s full body-frame projection) and
carries an unused `_shoulder_yaw()` helper, deliberately kept (not wired into
`compute_right_arm`/`compute_left_arm`) as a documented starting point for a future
shoulder-yaw estimate rather than deleted, since it was part of the original ported file.

---

## 9. Extensibility architecture: why an adapter, not a shared IK layer

An alternative design would push shoulder/elbow/head IK into `core` as robot-agnostic
functions parameterized by joint names and sign conventions, so every robot calls the
*same* formula. This was deliberately not done, for two reasons specific to this
package's history:

1. **The four robots' formulas are only superficially similar.** Shoulder pitch alone has
   two different concrete implementations across the four backends — a piecewise `atan2`
   on raw depth/vertical components for Pepper and NAO, a single continuous `atan2` with
   a different sign convention for G1 and AgiBot X2 — each tuned and validated against
   that specific robot family's hardware-documented zero pose. A forced shared
   abstraction would have to either lose those per-robot corrections or grow enough
   parameters to just become per-robot code again with extra indirection.
2. **This package has a direct, first-hand cautionary tale about "obviously equivalent"
   refactors**, from an earlier iteration of the Pepper backend (back when it used
   BODY_38 — see §8.1): a shoulder-pitch formula was generalized to project into a
   shared body-relative reference frame, on the reasoning that it was more
   physically principled than the original raw-camera-frame version. It silently rotated
   the pitch solution ~180° from what real Pepper hardware expected, because the
   generalization didn't re-verify the physical assumption that made the original
   formula correct for that specific hardware's zero pose. Keeping each robot's IK in
   its own file, reusing only the genuinely-shared *inputs* to that IK (keypoint
   extraction, geometry primitives, smoothing) rather than the IK itself, is a direct,
   deliberate response to that failure mode.

The adapter boundary is therefore drawn at "how do I get clean keypoints and a smoothing
filter," not at "how do I compute a shoulder angle." See the README's [Adapting to a new
humanoid](../README.md#adapting-to-a-new-humanoid) section for the concrete three-step
process this enables.

---

## 10. Validation methodology

Every robot backend in this package was **ported** from an existing, independently
proven script elsewhere in the parent repository (never written from scratch), and
validated by direct comparison against that source rather than by inspection alone:

1. **Frame-by-frame numerical diffing.** For each backend, a throwaway script imported
   both the original script and the new module, fed both the *identical* sequence of
   real recorded ROS bag frames, and asserted the computed joint angles matched (bit-for-
   bit, in practice `0.00e+00` max difference) for every joint on every frame — including
   through the smoothing step, which is stateful and therefore sensitive to call-order,
   not just the stateless IK functions. This was run across roughly 2,300-2,500 real
   BODY_18 bag frames per backend (Pepper, NAO, G1, AgiBot X2 each), drawn from real
   recordings, not synthetic data. (An earlier BODY_38 iteration of the Pepper backend —
   see §8.1 — was separately validated the same way against ~1,700 BODY_38 frames before
   being reverted; that validation no longer applies to the code as it exists today.)
2. **Full `rclpy` `Node`-level integration tests.** Real `Node` instances constructed,
   fed the same real bag messages through their actual subscription callbacks (not just
   the pure IK functions), asserting zero exceptions and all emitted angles within that
   robot's declared `JOINT_LIMITS`.
3. **Real `colcon build` + `ros2 run`/`ros2 launch` verification**, including installing
   this package into an isolated test workspace to confirm the packaging itself (not just
   the Python) was correct, and separately confirming RViz visualization end-to-end for
   the two `JointState`-publishing robots.

Where a fix changed actual behavior rather than just refactoring structure — the §9
Pepper shoulder-pitch regression being the clearest example from this package's history
— the same diffing methodology was used in reverse: to *prove* a formula was wrong on
specific synthetic reference poses (arm hanging down, arm raised, arm held horizontal to
the side) using a real body frame pulled from recorded data, not synthetic idealized
axes, before shipping the fix.

---

## 11. External dependencies and boundaries

| Dependency | Used by | Nature |
|---|---|---|
| `zed_msgs` | all four backends | ROS message definitions from the ZED ROS 2 wrapper; declared in `package.xml` |
| `rclpy`, `sensor_msgs` | all four backends | Standard ROS 2 |
| `numpy` | all core/robot math | Standard, declared via `python3-numpy` |
| `qi` (NAOqi SDK) | Pepper, NAO only | **Not** declared in `package.xml` — see §7. Has shown real environment sensitivity (works on a host, failed identically from inside this repo's own devcontainer against the same robot session) — see README §Troubleshooting |
| `robot_description` (sibling package) | G1, AgiBot X2 *visualization* only | Not a build or runtime dependency of this package at all — this package only ever publishes `/joint_states`; `robot_description`'s URDFs and launch files are an independent consumer, documented in the README because it's the tooling used to actually *see* those two robots move, not because this package needs it to run |

This package has **zero** import-time or build-time coupling to anything else in the
parent repository (`scripts/`, `teleop/`, `robot_description`) — every line above is
either a standard ROS/Python dependency or an out-of-band manual prerequisite (`qi`).

---

## 12. Known architectural limitations

- **QoS is untuned.** Every subscription uses the default depth-10 `rclpy` QoS profile,
  inherited unchanged from the scripts this was ported from. For a fast-moving skeleton
  topic this is probably fine in practice (observed to work across all validation
  bags), but it has not been deliberately chosen or load-tested against, e.g., a lossy
  network link.
- **No dedicated test suite.** Correctness rests entirely on the diff-against-original
  methodology in §10, run manually during development, not on a `pytest` suite that
  runs in CI or `colcon test`. `package.xml`'s `test_depend`s (`ament_copyright`,
  `ament_flake8`, `ament_pep257`, `python3-pytest`) are the standard `ament_python`
  boilerplate; no actual test files exist yet.
- **G1/AgiBot X2 sinks are IK-validated but sink-unvalidated against real hardware or a
  physics simulator** — only against `/joint_states` + RViz, as already noted in the
  README's Known Limitations and §8.3 above.
- **Single-person tracking only.** `extract_keypoints` always selects exactly one
  "best-tracked" person per frame; there's no path for controlling multiple robots from
  multiple simultaneously-tracked people, or for target-locking a specific person across
  frames rather than re-selecting "best-tracked" every message.
- **Smoothing is the only temporal filtering.** There's no outlier rejection at all — a
  single bad keypoint detection propagates through the exponential filter at whatever
  `alpha` that joint uses, rather than being detected and discarded as an outlier.
- **No forearm/wrist twist on any robot.** `ElbowYaw`/`WristYaw` (Pepper, NAO) and
  `shoulder_yaw` (G1, AgiBot X2) are either absent from the computed output or held at a
  neutral value on every backend — BODY_18 doesn't carry enough keypoints (no clavicle
  reference) to separate arm twist from torso twist. A richer skeleton format (e.g.
  BODY_38) would be needed to estimate these; see §8.1 for why that path was tried for
  Pepper specifically and reverted.

---

## 13. File reference index

| File | Role |
|---|---|
| `package.xml` | ROS 2 package manifest, `ament_python` build type, runtime deps |
| `setup.py` | Python packaging + the 4 `console_scripts` entry points |
| `setup.cfg` | `ament_python` script-install-path boilerplate |
| `resource/teleoperation_humanoid` | Empty `ament_index` resource marker |
| `teleoperation_humanoid/core/body18.py` | BODY_18 keypoint indices, `safe_norm`, `extract_keypoints` — used by all four backends |
| `teleoperation_humanoid/core/smoothing.py` | `ExponentialSmoother` |
| `teleoperation_humanoid/robots/pepper.py` | Pepper: BODY_18 IK, NAOqi sink, `Node`, `main()` |
| `teleoperation_humanoid/robots/nao.py` | NAO: BODY_18 IK, NAOqi sink, `Node`, `main()` |
| `teleoperation_humanoid/robots/unitree_g1.py` | G1: BODY_18 IK, `JointState` sink, `Node`, `main()` |
| `teleoperation_humanoid/robots/agibot_x2.py` | AgiBot X2: BODY_18 IK, `JointState` sink, `Node`, `main()` |
