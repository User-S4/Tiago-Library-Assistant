# sensors

Sensor-level processing for the **ERC 2026** head RGBD camera. Rebuilds the
depth-camera point cloud with an **Intel RealSense D435-like noise model** so the
simulated depth stream looks and behaves like a real sensor's, rather than a
perfect Gazebo surface.

This package holds the **sensor reconstruction** stage only; object detection
(the colored-book detector) lives in the separate `robot_perception` package.
The two are independent — each subscribes to the raw depth image directly, so
neither depends on the other.

## Node

- **`depth_to_cloud`** — the Gazebo `rgbd_camera` advertises
  `/head_front_camera/depth/points` but never fills it, so RViz's DepthCloud shows
  nothing. This node rebuilds the depth-camera point cloud the standard way
  (back-project the float32 depth image through `camera_info`, optionally colour
  it) and publishes a normal `PointCloud2`. It also applies a **RealSense-D435
  noise model** so the cloud looks like a real sensor's.

### RealSense realism

The sim head camera is retargeted to the **Intel RealSense D435 depth module** in
`erc_bringup` (640×360, 87°×~56° FoV, 0.2–8 m depth range — see
`erc_bringup/scripts/generate_urdf.py` PATCH 4b/4c). `depth_to_cloud` then models
the sensor's error:

- **Range-dependent noise** — depth → stereo disparity `d = f·B/Z`, add Gaussian
  disparity noise, back to `Z`. Error grows ~`Z²` (far surfaces get noisier, like
  a real D435), tuned by `disparity_sigma` (px) and `baseline` (m).
- **Quantisation shells** — disparity is snapped to `1/subpixel_bits` steps,
  reproducing the tell-tale depth banding.
- **Edge holes / dropouts** — points at depth discontinuities (`edge_hole_thresh`)
  and a small random fraction (`random_drop`) are removed.

Set `realsense_noise:=false` (or the individual params) for the clean cloud.

**Colouring.** A real depth camera outputs geometry, not RGB, so the cloud is
coloured by **distance** (turbo over `[color_min, color_max]` m) by default — the
RealSense-Viewer look. `color_source:=rgb` fuses the aligned colour image (the
RGBD product on a real sensor's `/depth/color/points`); `color_source:=none`
publishes plain XYZ.

## Topics

### Subscribed
| Topic | Type | Notes |
|---|---|---|
| `/head_front_camera/depth/depth_image` | `sensor_msgs/Image` (32FC1) | depth (param `depth_topic`) |
| `/head_front_camera/depth/camera_info` | `sensor_msgs/CameraInfo` | depth intrinsics (param `info_topic`) |
| `/head_front_camera/color` | `sensor_msgs/Image` (rgb8) | only used when `color_source:=rgb` (param `color_topic`) |

### Published
| Topic | Type | Description |
|---|---|---|
| `/head_front_camera/depth/points` | `sensor_msgs/PointCloud2` | **full depth-camera cloud** with the RealSense-D435 noise model, coloured by distance by default. Replaces the empty cloud gz advertises. |
| `/head_front_camera/depth/fov` | `visualization_msgs/Marker` (LINE_LIST) | cyan wireframe **frustum of the depth camera's field of view** (to `fov_range` m). |

## Build & run

`depth_to_cloud` is **brought up automatically by the sim** — `erc_bringup`'s
`simulation.launch.py` includes this package's launch file, so it starts with the
Gazebo sim and owns `/head_front_camera/depth/points`. You normally do **not**
launch it by hand (a second instance = two publishers on that topic = the mixed
rainbow+white cloud in RViz). Just build it once so the sim can find it:

```bash
./docker/attach.sh                                      # shell in the erc_sim container
colcon build --packages-select sensors --symlink-install && source install/setup.bash
```

Tune it via the sim launch args (forwarded to this node):

```bash
ros2 launch erc_bringup simulation.launch.py depth_cloud:=false     # don't start the cloud
```

Standalone (only if the sim was started with `depth_cloud:=false`):

```bash
ros2 launch sensors depth_to_cloud.launch.py color_source:=rgb      # fused colour
ros2 launch sensors depth_to_cloud.launch.py realsense_noise:=false # clean cloud
```

If RViz shows the cloud flat white, set that display's **Color Transformer** to
**RGB8**. Always run RViz with `use_sim_time` (the `robot_perception/rviz.launch.py`
preset does this and shows this cloud + the book detections together).

## Notes

- **Frame.** The cloud is published in the depth **optical** frame
  (`head_front_camera_depth_optical_frame`, Z forward). This requires the head
  camera's `gz_frame_id` to point at the optical frame — applied in
  `erc_bringup/scripts/generate_urdf.py` (PATCH 4b) and baked into
  `erc_description/urdf/tiago_pro.urdf`.
- **Range.** Only depths within the sensor's 0.2–8 m range appear; farther
  surfaces read as infinity and are dropped.
