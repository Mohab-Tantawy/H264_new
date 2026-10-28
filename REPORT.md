# Video Compression Protocol for Robotics Streaming — Progress Report

## Goal
Compare MJPEG (per-frame JPEG) against H.264 for a robot camera stream, in terms of encode/decode latency, bandwidth, and visual quality, to decide which is better suited for an FRC-style bandwidth-constrained robot radio link.

## What We Achieved

### 1. JPEG baseline (Windows, Python, OpenCV)
A `rclpy` publisher/subscriber pair using `cv2.imencode('.jpg', ...)` per frame, instrumented with timing.

**Result:** ~1.1 ms encode+publish per frame, consistently, no tuning required.

### 2. H.264 via plain OpenCV `VideoWriter` (Windows, Python)
Since `image_transport`'s ffmpeg plugin has no Windows build, we first tried encoding H.264 using `cv2.VideoWriter` per frame (via the Windows Media Foundation backend).

**Result:** ~66–200 ms per frame — dominated by per-frame container (MP4) open/write/finalize/read/delete overhead, not actual H.264 compression. File sizes were also larger than JPEG (~85 KB/frame) because every frame was an independent keyframe with no temporal compression — the approach discards H.264's main advantage.

**Conclusion:** per-frame-reinitialized H.264 is architecturally the wrong shape for this codec. Real H.264 benefit requires a persistent encoder.

### 3. H.264 via the real `image_transport`/`ffmpeg_image_transport` plugin (Linux VM, C++)
`ros-jazzy-ffmpeg-image-transport` only has Linux/macOS builds (no Windows package on the RoboStack/pixi channel), so this required a Linux environment. After working through several environment issues (detailed below), we got a genuine persistent-encoder H.264 stream running via `image_transport`'s native plugin system, in C++ (Python bindings — `image_transport_py` — are not yet packaged for ROS2 Jazzy at all, neither via apt nor in the `jazzy` branch of the source repo).

**Result after tuning** (`bit_rate=3 Mbps`, `preset:fast`, `tune:zerolatency`, `gop_size=15`, no B-frames):
- Encode+publish: roughly 10–90 ms/frame, varying run to run (software x264; no hardware encoder available in the VM)
- End-to-end latency: ~20–90 ms, fluctuating
- Default (untuned) settings gave ~1200 ms latency — B-frames and default x264 presets introduce heavy buffering/delay unsuitable for live streaming without explicit tuning.

**Known issue, unresolved:** a flat green/black artifact, varying in size and position frame-to-frame, appears in the decoded video (usually the bottom portion, sometimes additional horizontal bands). This was investigated thoroughly; the following causes were tested and **ruled out**:
- Resolution/macroblock padding mismatch — confirmed captured frames are genuinely 640×480 (16-aligned), logged directly from `cv::Mat` before encoding.
- Pixel format mismatch — forced `yuv420p` explicitly via the plugin's `pixel_format` parameter, no change.
- Encoder falling behind real-time (backlog) — persisted even at a reduced 8 FPS publish rate.
- Message loss over best-effort DDS transport — switched both publisher and subscriber to `RELIABLE` QoS, no change.
- OpenCV capture backend — tried both GStreamer (default) and explicit `cv::CAP_V4L2`, no change; artifact's shape/position differed between runs but didn't disappear with either backend.

The artifact's shape changes between runs even with no code changes, which suggests it originates somewhere non-deterministic in the pipeline (buffer reuse/timing, a libav-internal issue with this specific driver/resolution combination, or something in how VirtualBox's USB passthrough delivers camera frames). This was not resolved as of this report. **Recommended next step for whoever picks this up:** test on a real Linux machine (not a VM) with a real USB camera, to determine whether this is VM/passthrough-specific or a genuine plugin/pipeline bug — that single test would meaningfully narrow the remaining possibilities.


## Repository Layout

```
.gitignore                                   build/, install/, log/ are not committed
docs/REPORT.md                               this report
ros_ws/src/video_streaming_cpp/              the working H.264 implementation (ROS 2 Jazzy, C++)
    CMakeLists.txt
    package.xml
    src/camera_publisher_ffmpeg.cpp          camera -> H.264 via image_transport ffmpeg plugin
    src/viewer_ffmpeg.cpp                    subscribe, decode, display, log latency
experiments/original_jpeg/                   your original untimed JPEG publisher/viewer, unchanged
experiments/windows_python_baselines/        Windows/Python comparisons (timed JPEG baseline, per-frame OpenCV H.264)
experiments/image_transport_py_attempt/      Python version; does NOT run on Jazzy (see NOTE.md)
```

## Build and Run (Linux only)

Requires Ubuntu 24.04 + ROS 2 Jazzy. The H.264 plugin has no Windows build.

```bash
sudo apt install ros-jazzy-ffmpeg-image-transport ros-jazzy-cv-bridge ros-jazzy-vision-opencv
cd ros_ws
colcon build --packages-select video_streaming_cpp
source install/setup.bash

# terminal 1
ros2 run video_streaming_cpp camera_publisher_ffmpeg
# terminal 2
ros2 run video_streaming_cpp viewer_ffmpeg
```

Defaults are baked into the code (640x480, 3 Mbps, GOP 15, no B-frames, `preset:fast,tune:zerolatency`,
RELIABLE QoS, software decode). Override any of them at launch, e.g.
`--ros-args -p camera.image_raw.ffmpeg.bit_rate:=800000`. If you see `bitrate not specified`, the
encoder parameters were not applied; they are read once at encoder start, so set them at launch rather than
with `ros2 param set` on a running node.

The code in this package has been built and run on the VM, but `CMakeLists.txt` and `package.xml` were
re-created after that machine was lost; do a fresh `colcon build` before relying on them.

## Environment Notes (for whoever picks this up)
- Windows has **no path** to the real `ffmpeg_image_transport` plugin — not via apt-equivalent (pixi/RoboStack has no `win-64` build), not via WSL2 easily (we hit a persistent Windows-feature-enable loop that was never fully root-caused), and building from source on Windows would need vcpkg + MSVC + libav wired up manually.
- **Linux (we used a VirtualBox VM running Ubuntu 24.04 / ROS2 Jazzy)** is the only environment where this plugin actually works. VirtualBox USB passthrough is required for camera access (**Settings → USB → add a device filter for the webcam** — it may need re-adding after VM restarts).
- `cv_bridge` on recent Jazzy builds uses `cv_bridge/cv_bridge.hpp`, not the older `.h` — a common build error if copying code/examples from older ROS2 tutorials.
- `image_transport_py` does not exist for Jazzy (apt or source) as of this writing — C++ is currently the only option for using the ffmpeg transport plugin from ROS2 Jazzy.

## Next Steps

1. **Resolve the green-artifact bug.** Five hypotheses were tested and ruled out (see above). The strongest remaining lead is testing on bare-metal Linux with a real USB camera instead of a VM with USB passthrough, to isolate whether this is a VirtualBox-specific issue. If it reproduces on bare metal too, the next place to look is `ros2 bag record` the raw `/camera/image_raw` topic and inspect frame data directly (confirm whether corruption is present in the encoded bytes themselves, which would point to the encoder/plugin, versus only appearing after decode, which would point to the decoder).
2. **Re-run the JPEG baseline on the same Linux VM** for a true apples-to-apples comparison — all current JPEG numbers are from Windows, all current tuned-H.264 numbers are from the VM; different machines, not a clean comparison yet.
3. **Measure bandwidth/file size directly**, not just timing — this is H.264's actual core selling point (temporal compression) and hasn't been directly quantified yet, only inferred.
4. **Test on real target hardware** (an actual robot coprocessor, not a VM) — software x264 in a VM with no hardware encoder access is not representative of real deployment performance; a Raspberry Pi/Jetson/Linux coprocessor may have very different numbers (potentially faster with hardware encode, or slower with weaker CPU — worth measuring directly).
5. **Try the Python path again once `image_transport_py` is packaged for Jazzy** (or build against `rolling` if a quick compatibility check shows it works) — would let the team work in Python instead of C++ if preferred.
