# ESP32‑CAM → micro‑ROS (CompressedImage over Wi‑Fi/UDP)

Publishes JPEG frames from **ESP32‑CAM (AI Thinker)** as `sensor_msgs/msg/CompressedImage` via micro‑ROS (XRCE‑DDS, **UDP** transport). On the host, you run the micro‑ROS Agent and visualize with `rqt_image_view`.

> **Quick summary**
> - Topic: `image_raw/compressed` (format `jpeg`)
> - Node: `esp32cam_node`
> - Transport: Wi‑Fi (ESP32) → micro‑ROS Agent (UDP/8888)
> - Default frequency: `CAPTURE_PERIOD_MS` (default `500 ms` ≈ 2 FPS)
> - Tested with **AI Thinker** (adjust `camera.h` if you use a different module)

---

## Repository structure

```
include/
  app_config.h       # Wi‑Fi, agent, camera and ROS parameters (defines)
  camera.h           # Camera model selection (AI Thinker by default) + helpers
  camera_pins.h      # Pin mapping per module
  net.h              # Wi‑Fi + micro‑ROS transport (UDP)
  ros_image_pub.h    # CompressedImage publisher class
  state.h            # App state machine (agent up, down, etc.)

src/
  main.cpp           # Main loop (agent connection, capture, publish)
  camera.cpp         # Camera init (esp_camera_init)
  net.cpp            # Wi‑Fi connection and set_microros_wifi_transports
  ros_image_pub.cpp  # CompressedImage publisher implementation
```

---

## Requirements

- **Hardware**: ESP32‑CAM (AI Thinker) with PSRAM.
- **Firmware**: Arduino core for ESP32 (recommended via **PlatformIO**).
- **Libraries**: micro‑ROS for Arduino/PlatformIO (header used: `micro_ros_platformio.h`).

> ⚠️ If you use Arduino IDE directly, switch includes to `micro_ros_arduino.h` and configure the transport according to the official library. This repository targets **PlatformIO**.

---

## Configuration

Edit `include/app_config.h`:

- **Wi‑Fi**: `WIFI_SSID`, `WIFI_PSK`
- **micro‑ROS Agent**: `AGENT_IP` (e.g. `"192.168.65.3"`), `AGENT_PORT` (e.g. `8888`)
- **Camera**: `CAM_XCLK_FREQ`, `CAM_FRAME_SIZE`, `CAM_JPEG_QUALITY`, `CAM_FB_COUNT`
- **ROS**: `ROS_NODE_NAME`, `ROS_TOPIC` (default `image_raw/compressed`), `ROS_FRAME_ID`, `ROS_FORMAT` (`"jpeg"`)
- **Publish rate**: `CAPTURE_PERIOD_MS` (ms between captures)

> If your module is **not** AI Thinker, set the correct model in `include/camera.h` (options are commented at the top; pins come from `camera_pins.h`).

> If you want to use other ros distro, try to change the `board_microros_distro` in `platformio.ini`

---

## Build & Flash (PlatformIO)

1. Open the project folder in **VS Code** with **PlatformIO**.
2. Ensure the board is **esp32cam** (or equivalent) and the micro‑ROS lib is installed.
3. Put the ESP32‑CAM in **flash mode** (GPIO0 → GND, etc. per your board) and click **Upload**.
4. Open **Serial Monitor** at **115200 bps** to see logs.

> Tip: use a solid 2.4 GHz Wi‑Fi signal. Host firewalls can block the Agent UDP port.

---

## Run (ROS 2 side)

1. **Start the micro‑ROS Agent (UDP/8888)** — see *Quick commands* below.
2. Power the ESP32‑CAM; it connects to Wi‑Fi, configures transport, and pings the Agent.
3. Once connected, node `esp32cam_node` publishes on `image_raw/compressed` (type `sensor_msgs/msg/CompressedImage`).
4. Visualize with `rqt_image_view` on the host (also in *Quick commands*).

---

## Image and rate parameters

Currently, these *defines* control capture and payload size:

```c
#define CAM_FRAME_SIZE    FRAMESIZE_QVGA   // resolution (QVGA=320x240, etc.)
#define CAM_JPEG_QUALITY  10               // 0..63 (lower = better quality = larger file)
#define CAM_FB_COUNT      1                // framebuffers (2 = higher throughput, more RAM)
#define CAPTURE_PERIOD_MS 1000             // interval between publishes (ms)
```

**Tips**

- Higher resolutions (VGA, SVGA, UXGA) require PSRAM and increase bandwidth/latency.
- To reduce per‑frame size, **increase** `CAM_JPEG_QUALITY` (worse quality → smaller files) and/or **reduce** `CAM_FRAME_SIZE`.
- If frames freeze or the Agent drops, start with **QVGA** and `CAM_JPEG_QUALITY=15..25`, then tune.

> **Future**: a `ros_params.{h,cpp}` module can expose runtime changes for `frame_size`, `quality`, and `period_ms` via micro‑ROS parameters. The current code already has the right hooks (camera and publish loop).

---

## Published topic

- **Name**: `image_raw/compressed`
- **Type**: `sensor_msgs/msg/CompressedImage`
- **Header**: `frame_id = "esp32_camera"`, `format = "jpeg"`
- **Timestamp**: based on ESP32 `millis()` (seconds + nanoseconds)

---

## Troubleshooting

- **Nothing on ROS**: check `AGENT_IP`, `AGENT_PORT`, SSID/PSK, and whether the Docker Agent is listening on UDP/8888.
- **Latency/black frames in `rqt_image_view`**: increase `CAPTURE_PERIOD_MS`, increase `CAM_JPEG_QUALITY` (smaller files) or reduce `CAM_FRAME_SIZE`.
- **Camera init failure**: verify the selected model in `camera.h` and power/PSRAM.
- **Unstable Wi‑Fi**: place router closer; avoid crowded channels; disable extreme power‑saving.

---

## Quick commands (Docker)

> Run on the **host** (Linux). Commands kept **exactly** as provided.

**use for microros**
```bash
export ROS_DISTRO=humble docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v4
```

**use for rqt image view**
```bash
xhost +SI:localuser:root docker run -it --rm --name ros_rqt --net=host --env="DISPLAY" --env="XAUTHORITY=/root/.Xauthority" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$HOME/.Xauthority:/root/.Xauthority:ro" osrf/ros:humble-desktop-full bash
```

**and**
```bash
xhost +SI:localuser:root docker exec -it ros_rqt bash
```

> Inside the `ros_rqt` container, for example:
> ```bash
> source /opt/ros/humble/setup.bash
> ros2 topic list
> ros2 topic echo /image_raw/compressed  # should print JPEG bytes
> rqt_image_view
> ```

---

## Security notes

- Avoid committing `app_config.h` with real SSID/password. Use environment variables or an ignored `app_config.local.h`.
- `xhost +SI:localuser:root` relaxes X11 permissions for the container. Revert with `xhost -SI:localuser:root` when done.

---

## License

Use the code as you wish, im not your father

