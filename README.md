# Tennis Ball Tracker ROS 2 Package

The Tennis Ball Tracker ROS 2 package provides functionality for detecting and tracking tennis balls using computer vision techniques. It consists of two nodes: `tennis_ball_listener` and `tennis_ball_publisher`, which work together to detect tennis balls in a video stream.

## Nodes

### `tennis_ball_listener`

The `tennis_ball_listener` node subscribes to the `tennis_ball_image` topic, processes received images, and detects tennis balls in the frames. Detected tennis balls are highlighted, and the node displays the processed frames with visualizations.

### `tennis_ball_publisher`

The `tennis_ball_publisher` node captures frames from a video source (e.g., an MP4 file) and publishes them as ROS 2 image messages on the `tennis_ball_image` topic. These frames are processed by the `tennis_ball_listener` node for tennis ball detection.

## Dependencies
 - ROS2 (humble or later)
 - OpenCV

## Usage

1. Build the Tennis Ball Tracker package:

   ```bash
   colcon build --packages-select tennis_ball_tracker
   ```

2. Launch the Tennis Ball Tracker nodes using the provided launch file:

   ```bash
   colcon build --packages-select tennis_ball_tracker
   ```