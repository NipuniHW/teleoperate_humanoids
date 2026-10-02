# Teleoperation Humanoid

Real-time teleoperation of humanoid robots from a **ZED2i** stereo camera's 3D skeleton
tracking (BODY_18 format). A person stands in front of the camera; the robot mirrors
their arm, head, and (on Pepper/NAO) hand-openness movements in real time. This is a
standalone ROS 2 package — it has no dependency on the rest of the repository it ships
alongside, so it can be lifted out and reused on its own.

Validated on four humanoids, and designed to be extended to others — see
[Adapting to a new humanoid](#adapting-to-a-new-humanoid) below. For a full breakdown of
how the package is structured internally and why, see
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

---

## Supported Robots

| Robot | Skeleton format | Output | Hand control |
|---|---|---|---|
| SoftBank Pepper | BODY_18 | Real hardware (NAOqi) | Elbow-extension proxy |
| SoftBank/Aldebaran NAO | BODY_18 | Real hardware (NAOqi) | Elbow-extension proxy |
| Unitree G1 | BODY_18 | `sensor_msgs/JointState` (RViz) | — |
| AgiBot X2 | BODY_18 | `sensor_msgs/JointState` (RViz) | — |

All four robots consume the same BODY_18 skeleton format, via `teleoperation_humanoid.
core.body18`. Pepper and NAO estimate hand open/close from elbow-extension angle (no
finger keypoints exist in BODY_18) — straight arm reads as open, bent arm as closed. G1
and AgiBot X2 have no direct hardware/simulator binding of their own in this package:
they publish `/joint_states`, and you pair that with a URDF + `robot_state_publisher`
(or your own sim/hardware bridge) — see
[Visualizing G1 / AgiBot X2](#visualizing-g1--agibot-x2).

---

## Requirements

- ROS 2 Humble, on Ubuntu 22.04
- Python 3.10+, `numpy`
- [ZED SDK](https://www.stereolabs.com/developers/release) + the
  [ZED ROS 2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper) if you're using a
  **live camera** — see [Installing the ZED2i SDK](#installing-the-zed2i-sdk-for-a-live-camera)
  below for the full from-scratch walkthrough. If you're only ever replaying a provided
  `ros2 bag`, you still need the `zed_msgs` package installed (it defines the message
  type the bag's messages deserialize into), but you don't need the camera, the ZED SDK,
  or the wrapper process itself.
- **Pepper / NAO only**: the SoftBank/Aldebaran NAOqi `qi` Python SDK, importable in the
  same Python environment you run this package with. **Run Pepper/NAO teleoperation on
  the host, not inside a devcontainer** — we hit a reproducible `RuntimeError: No
  reachable endpoint was found for this service` (connection succeeds, the target service
  even shows up as registered, but calling it fails) from inside this repo's own
  devcontainer, which turned out to be specific to that container's environment: the
  identical connection, from the identical robot/virtual-robot session, succeeded
  immediately when run on the host instead. If you hit that error, try the host before
  anything else — see Troubleshooting below.
- **G1 / AgiBot X2 visualization only**: a URDF-providing package such as
  [`robot_description`](https://github.com/ioai-tech/robot_description) (MIT-licensed),
  plus these two apt packages if not already installed:
  ```bash
  sudo apt-get install -y ros-humble-xacro ros-humble-rviz2
  ```

---

## Installing the ZED2i SDK (for a live camera)

Skip this whole section if you're only ever replaying a provided `ros2 bag` — you don't
need a camera, the SDK, or any of the below. If you have a physical ZED2i and want to
drive a robot from it directly, here's the real, from-scratch path (verified against
Stereolabs' current official docs, not the parent repo's prebuilt Docker image, which
isn't something you have access to from this standalone package).

**1. Prerequisites**: Ubuntu 22.04, an NVIDIA GPU with driver 550+ installed, and a USB3
port for the camera. CUDA will be installed automatically by the SDK installer below if
it isn't already present.

**2. Install the ZED SDK:**

```bash
sudo apt install zstd
```

Download the Ubuntu 22.04 installer for your CUDA version from
[stereolabs.com/developers/release](https://www.stereolabs.com/developers/release/),
then:

```bash
chmod +x ZED_SDK_UbuntuXX_cudaYY.Y_vZ.Z.Z.zstd.run
./ZED_SDK_UbuntuXX_cudaYY.Y_vZ.Z.Z.zstd.run
```

Follow the prompts — accept the license, let it install the Python API and tools, and
say yes when it offers to download the AI models (body tracking needs these). Reboot
once it's done so updated paths take effect. It installs to `/usr/local/zed`.

**3. Verify the SDK sees your camera** — plug the ZED2i into a USB3 port, then:

```bash
/usr/local/zed/tools/ZED_Explorer     # shows the live raw video feed if detected
/usr/local/zed/tools/ZED_Diagnostic   # hardware/software diagnostic report
```

Don't move on to the ROS 2 wrapper until `ZED_Explorer` actually shows a live image —
anything else (camera not detected, USB errors) is an SDK/hardware problem, not
something fixable at the ROS layer.

**4. Build the ZED ROS 2 wrapper** (official source, no prebuilt image needed):

```bash
mkdir -p ~/zed_ros2_ws/src
cd ~/zed_ros2_ws/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ~/zed_ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
source install/local_setup.bash
```

**5. Launch it, with body tracking enabled:**

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i body_trk_enabled:=true
```

Confirm skeleton data is actually flowing before moving on to this package:

```bash
ros2 topic hz /zed/zed_node/body_trk/skeletons
```

If that hangs with no output, go back to step 3 before suspecting anything downstream.

Prefer Docker over a native install? Stereolabs publishes official ZED SDK images — see
[stereolabs.com/docs/docker](https://www.stereolabs.com/docs/docker/install-guide-linux) —
you'd still build `zed-ros2-wrapper` from source (step 4) inside that container, since
there's no prebuilt image bundling both the SDK and the wrapper.

---

## Getting Started, from scratch

This walks through everything needed to go from a clean ROS 2 Humble install to actually
watching a robot move, for someone who has never touched this repo before. It covers
**both** ways of getting skeleton data in: a live ZED2i camera, or a `ros2 bag` someone
recorded for you. The worked example at the end uses AgiBot X2 from a bag, since that
path needs no robot hardware and no camera — the fastest way to confirm everything works.

### 1. Build the package

Clone this repository into the `src/` folder of a colcon workspace (or wherever your
workspace keeps its packages), then build from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select teleoperation_humanoid
source install/setup.bash
```

Re-run `source install/setup.bash` (or open a new terminal) any time after a rebuild —
`ros2 run`/`ros2 launch` won't see a newly built package in a terminal whose overlay is
already sourced from before the build.

### 2. Get skeleton data flowing

Pick one:

**Option A — Live ZED2i camera.** See
[Installing the ZED2i SDK](#installing-the-zed2i-sdk-for-a-live-camera) above if you
haven't already — once the SDK and `zed-ros2-wrapper` are installed:

```bash
source ~/zed_ros2_ws/install/local_setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i body_trk_enabled:=true
```

If you're running this package and the ZED wrapper in separate containers rather than
both natively on the host, run the wrapper in its own container — don't share a
container with this package, since the two can conflict over host networking/NVIDIA
runtime access.

**Option B — A provided rosbag.** If someone gave you a recorded bag instead of camera
access, just play it back — no Docker, no camera, nothing else required:

```bash
ros2 bag play /path/to/the/bag --loop
```

`--loop` keeps it repeating so you don't need to keep restarting it while testing.

Either way, this container and the one producing skeleton data are two separate
processes/machines talking over DDS — FastDDS's shared-memory transport silently fails
across a Docker/host boundary, so set a UDP-only profile in **every terminal** you use
(including the one playing the bag, if it's in a different container):

```bash
mkdir -p ~/.ros && cat > ~/.ros/no_shm.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_only</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default" is_default_profile="true">
        <rtps>
            <userTransports><transport_id>udp_only</transport_id></userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/no_shm.xml
```

Sanity check that data is actually arriving before moving on:

```bash
ros2 topic hz /zed/zed_node/body_trk/skeletons
```

If that hangs with no output, skeleton data isn't reaching this terminal — re-check the
FastDDS profile above and that the camera/bag is actually running.

### 3. Run the teleop node for your robot

See [Quick Start](#quick-start) below for the exact command per robot. Pepper/NAO need a
reachable NAOqi endpoint; G1/AgiBot X2 need nothing beyond skeleton data — they just
publish `/joint_states`.

### 4. (G1 / AgiBot X2 only) Visualize it

These two have no visualization of their own — see
[Visualizing G1 / AgiBot X2](#visualizing-g1--agibot-x2) below.

### Worked example: AgiBot X2 from a bag, end to end

Three terminals, in this order. This assumes `robot_description` (or an equivalent
URDF-providing package) is built as a sibling package in the same workspace as this
one, so a single `source install/setup.bash` from the workspace root covers both.

```bash
# Terminal 1 — play the bag
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/no_shm.xml
ros2 bag play /path/to/the/bag --loop

# Terminal 2 — run the teleop node
source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/no_shm.xml
ros2 run teleoperation_humanoid teleop_agibot_x2

# Terminal 3 — visualize
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1   # see Troubleshooting if you don't need this
ros2 launch robot_description teleop_robot.launch.py organization:=agibot robot_type:=x2
```

RViz should open showing the AgiBot X2 model, moving in time with whatever body motion
is in the bag. If the model appears as a flat white silhouette with joints frozen at
zero, terminal 2 (`teleop_agibot_x2`) isn't actually running or isn't receiving skeleton
data — check terminal 1 first.

---

## Quick Start

Each robot is a separate `ros2 run` entry point. All of them subscribe to
`/zed/zed_node/body_trk/skeletons` by default (override with `--skeleton-topic`).

### Pepper

```bash
ros2 run teleoperation_humanoid teleop_pepper --robot-ip <pepper-ip> --robot-port 9559
```

### NAO

```bash
ros2 run teleoperation_humanoid teleop_nao --robot-ip <nao-ip> --robot-port 9559
```

`--robot-ip` defaults to `localhost` (a local NAOqi simulator/proxy); `--robot-port`
defaults to `9559`, NAOqi's standard port.

### Unitree G1

```bash
ros2 run teleoperation_humanoid teleop_g1
```

### AgiBot X2

```bash
ros2 run teleoperation_humanoid teleop_agibot_x2
```

Stand in front of the ZED2i camera (roughly 1.5-4 m), or have that motion present in
whatever bag is playing. Press **Ctrl+C** to stop — Pepper and NAO return to `StandInit`
on exit.

### Visualizing G1 / AgiBot X2

G1 and AgiBot X2 only publish `/joint_states` — pair that with a URDF and
`robot_state_publisher` to actually see the robot move. If you have a URDF-providing
package such as `robot_description` built as a sibling package in your workspace:

```bash
ros2 launch robot_description teleop_robot.launch.py organization:=unitree robot_type:=g1
# or: organization:=agibot robot_type:=x2
```

then run the matching `teleop_g1` / `teleop_agibot_x2` command above in another terminal.

---

## Troubleshooting

- **`ros2 run`/`ros2 launch` says "Package 'teleoperation_humanoid' not found" (or
  similar for `robot_description`)** — you haven't built it in *this* workspace, or built
  it but this terminal's `install/setup.bash` predates the build. Run
  `colcon build --packages-select teleoperation_humanoid` from the repo root, then
  `source install/setup.bash` again (or open a new terminal).
- **`ros2 launch robot_description ...` fails with `executable '...' not found on the
  PATH`** — `xacro` isn't installed: `sudo apt-get install -y ros-humble-xacro`.
- **Same launch fails with `package 'rviz2' not found`** — RViz itself isn't installed:
  `sudo apt-get install -y ros-humble-rviz2`.
- **RViz's own process starts (shows up in `ps aux`) but no window ever appears, and it
  doesn't even respond to Ctrl+C / needs a hard kill** — this is a hung OpenGL context,
  common in containerized/VNC/remote-desktop environments where `/dev/dri` GPU nodes
  exist but can't actually be driven for hardware GL from that display. Force software
  rendering: `export LIBGL_ALWAYS_SOFTWARE=1` before launching. You can confirm this is
  the cause beforehand with `glxinfo` (from `apt-get install -y mesa-utils x11-utils`) —
  if it hangs, this is why; with the env var set it should return instantly.
- **RViz opens, but the robot renders as a flat white silhouette with most links showing
  "No transform from [...]" errors in the Displays panel** — `robot_state_publisher` only
  has `/joint_states` (published by `teleop_g1`/`teleop_agibot_x2`) to compute the
  dynamic part of the TF tree; static links resolve fine, everything else won't until
  that node is actually running and receiving skeleton data. Check `ros2 topic hz
  /joint_states` — if nothing's arriving, `teleop_g1`/`teleop_agibot_x2` either isn't
  running or isn't getting skeleton data (see the `ros2 topic hz
  /zed/zed_node/body_trk/skeletons` check in step 2 above).
- **Pepper/NAO: `connect_pepper`/`connect_nao` raises `RuntimeError: No reachable
  endpoint was found for this service`** — the TCP connection to the broker succeeds and
  it even lists the service as registered, but the session redirect to the actual service
  process fails. This is essentially never a bug in this package (verify by reproducing
  it with a bare `qi.Session()` — same failure, zero `teleoperation_humanoid` code
  involved). **Try running on the host instead of inside a devcontainer first** — we hit
  this reproducibly from inside this repo's own devcontainer, and the identical
  connection to the identical virtual-robot session succeeded immediately from the host.
  If running on the host isn't an option or doesn't fix it, other things to check: (1) a
  stale/half-initialized virtual robot session — fully quit and relaunch it rather than
  just disconnecting/reconnecting in the UI; (2) wrong `--robot-ip`/`--robot-port` for
  what's actually running right now (Choregraphe virtual robots get a new random port
  every session); (3) a genuine `libqi`/NAOqi broker protocol mismatch — there's an open,
  unresolved upstream issue on `aldebaran/libqi-python` for exactly this symptom against
  older NAOqi versions.
- **FastDDS / no data arriving across a Docker↔host boundary** — see the
  `FASTRTPS_DEFAULT_PROFILES_FILE` setup in step 2 above; this must be set in *every*
  terminal on *both* sides of the boundary.

---

## Adapting to a new humanoid

Each `teleoperation_humanoid/robots/<name>.py` file is self-contained and deliberately
not split across multiple files, so you can copy one as a template for a new robot. Using
`robots/nao.py` as the starting point (it has the fewest moving parts of the four), you
need to change three things:

1. **`JOINT_LIMITS`** — a `{joint_name: (min_rad, max_rad)}` dict for your robot, and a
   `clamp()` that clips into it.
2. **The IK functions** (`compute_right_arm`, `compute_left_arm`, `compute_head`, ...) —
   each takes the extracted keypoint list and returns a `{joint_name: angle}` dict. Reuse
   `teleoperation_humanoid.core.body18` for keypoint indices, `safe_norm`, and
   `extract_keypoints` — don't reimplement skeleton parsing. The geometry itself
   (shoulder pitch/roll from the upper-arm vector, elbow
   flexion from the angle between upper arm and forearm, and so on) transfers across
   robots; only the sign conventions, axis choices, and joint-limit ranges are
   robot-specific, and each existing backend documents its own reasoning inline.
3. **A command sink** — either a real hardware/simulator connection (see
   `robots/pepper.py`'s `connect_pepper()` / `send_angles()` for the NAOqi pattern), or a
   `sensor_msgs/JointState` publisher (see `robots/unitree_g1.py` for the
   URDF/`robot_state_publisher` pattern, which is the more broadly reusable option if
   your robot doesn't have a bespoke SDK).

Then wire it up as a `Node` subclass (`__init__` subscribes to the skeleton topic; the
callback extracts keypoints, computes angles, smooths them with
`teleoperation_humanoid.core.smoothing.ExponentialSmoother`, and sends them), and register
a `console_scripts` entry point in `setup.py`.

---

## How the pipeline works

```
ZED2i camera
   -> zed-ros2-wrapper (external)
   -> /zed/zed_node/body_trk/skeletons  (zed_msgs/ObjectsStamped)
   -> core.body18: pick best-tracked person, extract keypoints
   -> robots/<robot>.py: per-joint IK from keypoint geometry
   -> core.smoothing.ExponentialSmoother: per-joint exponential low-pass filter
   -> robot-specific sink: NAOqi setAngles(), or sensor_msgs/JointState publish
```

---

## Known Limitations

- BODY_18 has no finger keypoints at all, so Pepper's and NAO's hand-openness estimate is
  purely an elbow-extension proxy (straight arm = open, bent arm = closed) — it reflects
  reach/grasp *intent* from arm pose, not actual finger state. There's no way to get a
  more direct signal without a richer skeleton format.
- Neither Pepper nor NAO's `ElbowYaw`/`WristYaw` (forearm/wrist twist) is computed at all
  — BODY_18 doesn't provide enough keypoints (no clavicle reference) to separate arm
  twist from torso twist, so both are left at NAOqi's default/last-commanded value.
- G1 and AgiBot X2 have only been exercised via `/joint_states` + RViz in this repo, not
  against real hardware or a physics simulator — the IK is validated, but the sink is not.
- Pepper/NAO's `qi` SDK connection has a known environment sensitivity: it failed
  reproducibly inside this repo's own devcontainer (`RuntimeError: No reachable endpoint
  was found for this service`) while working immediately against the identical robot
  session from the host — see Troubleshooting. The exact root cause (specific `qi`
  package version, something else about the devcontainer's environment) wasn't fully
  pinned down; "run it on the host" is the known-working workaround.

---

## License

MIT — see [LICENSE](LICENSE).
