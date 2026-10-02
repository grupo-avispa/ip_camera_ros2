# Fisheye camera calibration

This guide calibrates a fisheye camera (about 180 degrees) published by `ip_camera_ros2`, and makes the node publish a `CameraInfo` with the `equidistant` model that other nodes can use.

## Before calibrating

Use the main stream of the camera, not the low-quality substream: compression artifacts are amplified at the edges of a fisheye image.

Fix the image before calibrating. Leave `image_width`, `image_height`, `offset_x` and `offset_y` with their final values, as well as the resolution of the stream. The intrinsics are expressed in pixels of the calibrated image, so any later change invalidates the calibration. The node detects it if `calibration_width` and `calibration_height` are set: it stops publishing the `CameraInfo` and warns about the size mismatch.

Turn off the on-screen display (date and time stamp) in the camera if possible, so that it is not warped by any later processing.

While calibrating, keep `enable_cam_info: false` or leave the calibration template untouched. The node only publishes a `CameraInfo` with a valid calibration, so nothing is published until the calibration is applied.

## Pattern

Use a chessboard (`--pattern chessboard`, the default). ChArUco is not implemented for fisheye cameras in `camera_calibration`. The board must be rigid, flat and large (A1 or bigger) if the camera is on the ceiling. As an alternative the camera can be calibrated dismounted and closer to the board, as long as the focus is not touched and the image is published with the same crop and scale.

## Running the calibrator

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.08 \
  --fisheye-k-coefficients 4 \
  --fisheye-recompute-extrinsicsts \
  --ros-args -r image:=/image -r camera/set_camera_info:=/set_camera_info
```

`--size` is the number of **inner** corners of the board and `--square` is the side of a square in meters. Set them to the real board.

`--fisheye-recompute-extrinsicsts` enables `cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC`. The name of the option is spelled that way.

Do not use `--fisheye-fix-principal-point`: the optical center of a fisheye camera is rarely at the center of the image.

If the calibration fails because the problem is ill-conditioned, remove the most oblique views before trying `--fisheye-check-conditions`.

The model is chosen in the calibrator window with the "Camera type" control (0 is pinhole, 1 is fisheye). It can only be changed **after the first image has been received**. Without this a pinhole model is calibrated, which cannot represent 180 degrees.

The service name `/set_camera_info` is the sibling of the image topic: with `image_topic: /ceil01/image` it is `/ceil01/set_camera_info`.

## Coverage

Cover the outer ring of the image circle: it is what matters most and what is hardest to cover. Outside the range of angles that the pattern covers, the Kannala-Brandt polynomial is not reliable. Move the board all over the image, tilt it, and take views close to the edge of the circle.

## Applying and persisting the calibration

1. Press CALIBRATE and then COMMIT. The calibration is applied immediately, as parameters of the node, and `/camera_info` starts to be published with `distortion_model: equidistant`. The node logs the YAML block with the values.
2. Run `ros2 param dump /ip_camera_ros2` (or copy the YAML block from the log) and copy the values to the parameters file used by the launch file. That file is the only persistent source of the calibration.
3. Press SAVE to also store `/tmp/calibrationdata.tar.gz`, with the images used and `ost.yaml`. Move it out of `/tmp`, which is cleared on reboot on some systems.

The values applied are `distortion_model`, `camera_matrix`, `distortion_coefficients`, `rectification_matrix`, `projection_matrix`, `calibration_width`, `calibration_height` and `enable_cam_info: true`. The service refuses a calibration that is not valid, and answers with the reason.

## Without the service

For older versions of the calibrator, or for an offline calibration, start the calibrator with `--no-service-check`, press SAVE and copy the values from `ost.yaml` to the parameters file:

| `ost.yaml` | Parameter |
| --- | --- |
| `camera_matrix.data` | `camera_matrix` |
| `distortion_coefficients.data` | `distortion_coefficients` |
| `rectification_matrix.data` | `rectification_matrix` |
| `projection_matrix.data` | `projection_matrix` |
| `distortion_model` | `distortion_model` |
| `image_width`, `image_height` | `calibration_width`, `calibration_height` |

Set `enable_cam_info: true` as well. Every element of the arrays must be written as a floating-point number (`1.0`, not `1`), or the parameter file is rejected.

## Checking

```bash
ros2 topic echo --once /camera_info
```

It must show `distortion_model: equidistant`, 4 coefficients in `d`, the right `width` and `height`, and a `k` matrix that is not zero.

## Alternative

`camera_info_manager` with a `camera_info_url` is the standard way to store a calibration in ROS, and it saves the file by itself. This package keeps the calibration in its parameters file instead, and the service only feeds it.
