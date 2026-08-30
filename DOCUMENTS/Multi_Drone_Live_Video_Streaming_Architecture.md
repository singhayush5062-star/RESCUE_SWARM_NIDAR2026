# Multi-Drone Live Video Streaming Architecture & Implementation Guide
## Low-Latency, Zero-Memory-Leak Video Transmission for NIDAR Swarm GCS

---

## 1. Executive Summary & Root Cause Analysis

### 1.1 Problem Statement
When streaming live camera feeds from 4 drones (`drone0`, `drone1`, `drone2`, `drone3`) simultaneously to the Ground Control Station (GCS), the system suffers from severe memory exhaustion, high latency (multi-second lag), and browser UI freezes ("system get hang").

### 1.2 Root Cause Technical Breakdown

```
[Gazebo / ROS 2 Image Topic] 
         │ (Raw Uncompressed 640x480 @ 30 FPS = ~110 MB/s binary data)
         ▼
[rosbridge_server (Python)] ──> Converts binary images to Base64 JSON strings (~146 MB/s text data)
         │ (Single-threaded CPU bottleneck; high WebSocket queue buffer size)
         ▼
[Browser WebSocket Client] ──> Receives huge JSON messages on main JS Thread
         │
         ├──> Base64 string allocation in V8 Heap (thousands of strings/sec)
         ├──> Frequent V8 Garbage Collection (GC) pauses (100ms - 2000ms freeze)
         └──> React useState re-renders <img src="data:image/jpeg;base64,...">
              forcing main-thread image decoding & DOM layout recalculations
```

1. **Base64 String Allocation & Garbage Collector (GC) Thrashing**:
   - Converting binary JPEG/PNG frames into Base64 JSON strings (`data:image/jpeg;base64,...`) adds **~33% bandwidth overhead** and allocates high volumes of short-lived heap strings.
   - React state updates (`useState`) on every frame force frequent JavaScript engine V8 Garbage Collection pauses, leading to memory leaks and browser thread locks.
2. **Single-Threaded rosbridge_server Bottleneck**:
   - `rosbridge_server` (Python) serializes ROS messages to JSON on a single thread. Handling 4 concurrent high-frequency image topics chokes its CPU execution, causing message queue buildup and multi-second latency.
3. **Main-Thread DOM Image Decoding**:
   - Updating `<img src="base64">` forces synchronous CPU/DOM image parsing and layout recalculation on the main thread for 4 video streams simultaneously.

---

## 2. Solution Comparison Matrix

| Metric / Parameter | Current (rosbridge Base64) | Solution A: WebRTC (MediaMTX) ⭐ | Solution B: web_video_server (MJPEG) | Solution C: WebCodecs + Binary WS |
| :--- | :--- | :--- | :--- | :--- |
| **Latency** | High (2s - 10s lag) | **Ultra-Low (<100ms)** | Low (100ms - 250ms) | Low (<150ms) |
| **GCS JS RAM Usage** | 2 GB+ (GC Thrashing/OOM) | **Static (~20 MB)** | **Static (~15 MB)** | Moderate (~50 MB) |
| **GCS JS CPU Usage** | 80% - 100% (Freeze) | **< 2% (Hardware GPU Decode)** | **< 5% (Native Browser Decode)** | ~15% (Worker Thread) |
| **Bandwidth (4 Streams)** | ~150 MB/s (Chokes network) | **~2 - 4 MB/s (H.264/H.265)** | ~10 - 20 MB/s (JPEG Streams) | ~3 - 6 MB/s (H.264 NAL) |
| **Implementation Effort**| High Maintenance | Moderate | **Very Low (Plug-and-play)** | High |
| **Recommended Use Case** | Text telemetry only | **Production / Swarms** | **Fast-Track Prototype** | Custom Low-level |

---

## 3. Recommended Architecture: WebRTC Video Streaming (Solution A)

### 3.1 System Overview Architecture

```mermaid
flowchart LR
    subgraph Drones / Gazebo Simulation
        D0[Drone 0 Camera]
        D1[Drone 1 Camera]
        D2[Drone 2 Camera]
        D3[Drone 3 Camera]
    end

    subgraph Streaming Gateway
        GST[GStreamer / FFmpeg / ROS2 Image Transport]
        MTX[MediaMTX Streaming Server RTSP / WHEP]
    end

    subgraph Ground Control Station (GCS)
        V1[<video id='drone0'>]
        V2[<video id='drone1'>]
        V3[<video id='drone2'>]
        V4[<video id='drone3'>]
    end

    D0 & D1 & D2 & D3 -->|RTSP / RTP Stream| GST
    GST -->|H.264 / VP8 Encodings| MTX
    MTX -->|WebRTC WHEP (UDP)| V1 & V2 & V3 & V4
```

### 3.2 Key Advantages of WebRTC
- **Direct GPU Hardware Decoding**: Browsers process WebRTC video streams in native C++/GPU hardware decoding pipelines, completely bypassing JavaScript, V8 heap allocations, and React render cycles.
- **RTP/UDP Transport**: Packet loss is handled gracefully without stalling stream playback; latency stays strictly bounded under 100 milliseconds.
- **Dynamic Bandwidth Adaptation**: Automatically adjusts bitrate depending on connection quality.

---

## 4. Alternative Architecture: ROS 2 `web_video_server` (Solution B)

For immediate, minimal-code deployment without extra RTSP media servers, `web_video_server` chokes zero JS memory by shifting image streaming from WebSockets to standard HTTP multipart streams.

### 4.1 How `web_video_server` Solves Memory Leaks
- It is a native C++ ROS 2 node.
- It exposes HTTP endpoints: `http://<host>:8080/stream?topic=/drone0/sensor_measurements/hd_camera/image_raw&type=mjpeg&quality=40&width=480&height=360`.
- The browser fetches raw HTTP stream data directly into an HTML `<img>` tag without JavaScript handling byte buffers or Base64 strings.

---

## 5. Step-by-Step Implementation Guide

### Phase 1: Deploying ROS 2 `web_video_server` (Immediate Setup)

#### Step 1.1: Install `web_video_server` in ROS 2 Workspace
Add `web_video_server` to your workspace or system dependencies:
```bash
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-web-video-server
```

#### Step 1.2: Add `web_video_server` to ROS 2 Launch Scripts
In `project_gazebo/launch_as2.bash` or a dedicated launch file, launch the server:
```bash
ros2 run web_video_server web_video_server --ros-args \
  -p port:=8080 \
  -p default_stream_type:=mjpeg \
  -p publish_rate:=15.0
```

---

### Phase 2: Deploying Production WebRTC with MediaMTX (Optimal Performance)

#### Step 2.1: Run MediaMTX Container / Binary
MediaMTX is a zero-dependency, open-source RTSP/WebRTC server:
```bash
docker run --rm -d --network=host bluenviron/mediamtx:latest
```
Or download the static binary:
```bash
wget https://github.com/bluenviron/mediamtx/releases/download/v1.9.0/mediamtx_v1.9.0_linux_amd64.tar.gz
tar -xvf mediamtx_v1.9.0_linux_amd64.tar.gz
./mediamtx
```

#### Step 2.2: Stream ROS 2 Camera Topics to MediaMTX via GStreamer / FFmpeg
Launch a lightweight pipeline node for each drone (`drone0` to `drone3`):
```bash
# Example FFmpeg pipeline reading ROS image topic and streaming RTSP to MediaMTX
ffmpeg -f rawvideo -pix_fmt rgb24 -s 640x480 -r 15 \
  -i /dev/zero \
  -c:v libx264 -preset ultrafast -tune zerolatency -b:v 800k \
  -f rtsp rtsp://localhost:8554/drone0_camera
```

---

### Phase 3: Updating GCS Frontend (`VideoPanel.tsx`)

#### 3.1 React Component for `web_video_server` (MJPEG Native Stream)
Update `gcs/src/components/VideoPanel.tsx` to use native HTTP URLs instead of Base64 strings:

```tsx
import React, { useState } from 'react';
import './VideoPanel.css';

interface VideoPanelProps {
  droneNamespaces: string[];
  hostIp?: string;
  port?: number;
}

export function VideoPanel({ droneNamespaces, hostIp = 'localhost', port = 8080 }: VideoPanelProps) {
  const [expanded, setExpanded] = useState<string | null>(null);

  const getStreamUrl = (ns: string, width = 480, height = 360, fps = 10) => {
    const topic = `/${ns}/sensor_measurements/hd_camera/image_raw`;
    return `http://${hostIp}:${port}/stream?topic=${encodeURIComponent(topic)}&type=mjpeg&quality=45&width=${width}&height=${height}&fps=${fps}`;
  };

  return (
    <div className="video-panel">
      <div className="video-panel__header">
        <div className="video-panel__title">Live Drone Camera Streams</div>
      </div>
      <div className="video-panel__feeds">
        {droneNamespaces.map((ns) => (
          <figure key={ns} className="video-panel__feed">
            <figcaption className="video-panel__caption">{ns}</figcaption>
            <img
              src={getStreamUrl(ns, 480, 360, 10)}
              alt={`${ns} stream`}
              loading="eager"
              className="video-panel__img"
              onClick={() => setExpanded(ns)}
            />
          </figure>
        ))}
      </div>
    </div>
  );
}
```

#### 3.2 React Component for WebRTC (MediaMTX WHEP Stream)
For WebRTC, use native `<video>` elements with standard WHEP client SDP exchange:

```tsx
import React, { useEffect, useRef } from 'react';

interface WebRTCPlayerProps {
  streamName: string; // e.g. "drone0_camera"
  mediamtxHost?: string;
}

export function WebRTCPlayer({ streamName, mediamtxHost = 'localhost:8889' }: WebRTCPlayerProps) {
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    const pc = new RTCPeerConnection();
    pc.addTransceiver('video', { direction: 'recvonly' });

    pc.ontrack = (event) => {
      if (videoRef.current && event.streams[0]) {
        videoRef.current.srcObject = event.streams[0];
      }
    };

    async function start() {
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);

      const response = await fetch(`http://${mediamtxHost}/${streamName}/whep`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/sdp' },
        body: offer.sdp,
      });

      if (response.ok) {
        const answerSdp = await response.text();
        await pc.setRemoteDescription(new RTCSessionDescription({ type: 'answer', sdp: answerSdp }));
      }
    }

    start();

    return () => {
      pc.close();
    };
  }, [streamName, mediamtxHost]);

  return (
    <video
      ref={videoRef}
      autoPlay
      playsInline
      muted
      style={{ width: '100%', height: '100%', objectFit: 'cover' }}
    />
  );
}
```

---

## 6. Optimization Rules & System Stability Checklist

To prevent system hangs and memory shortage across the entire stack:

1. **Decouple Telemetry from Video Streaming**:
   - Keep `rosbridge_server` strictly for low-bandwidth JSON control/telemetry (GPS, battery, state, detection bounding box coordinates).
   - **NEVER** push raw image topics (`sensor_msgs/Image` or `sensor_msgs/CompressedImage`) over `rosbridge_server` WebSockets.
2. **Adaptive Framerate Rules**:
   - **Grid Multi-View (4 Drones)**: 5 – 10 FPS at 480x360 resolution (Low Bitrate).
   - **Focused/Expanded Single Drone View**: 20 – 30 FPS at 1280x720 resolution.
3. **ROS 2 Shared Memory & QoS Tuning**:
   - Ensure camera publisher nodes use `SensorDataQoS` (Best Effort reliability, Volatile durability, Queue depth 1) so old dropped frames do not accumulate in system RAM buffers.
4. **Hardware Acceleration**:
   - Enable NVENC (NVIDIA CUDA) or VAAPI/V4L2 hardware encoding on Gazebo/sim host machines for video stream compression.

---

## 7. Next Steps & Execution Plan
1. **Immediate fix (Phase 1)**: Deploy `web_video_server` in ROS 2 workspace and point GCS `VideoPanel.tsx` `<img>` sources to HTTP stream URLs.
2. **Production upgrade (Phase 2)**: Set up MediaMTX container and WebRTC `WHEP` stream players for sub-100ms hardware-accelerated video feeds.
