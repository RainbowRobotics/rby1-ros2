# Camera calibrations

Put a camera's calibration here and name it in the camera's settings
(`config/camera/realsense.yaml`):

```yaml
    intrinsics_file: "d435_head.yaml"   # a file in this folder; an absolute path also works
```

Rebuild `rby1_tools` after adding a file (`colcon build --packages-select rby1_tools`): the camera
node reads the installed copy of this folder. Empty `intrinsics_file` means no calibration file: the
RealSense uses its factory calibration.

The file is the camera matrix and distortion measured at one image size. Another size with the same
aspect ratio is scaled; another aspect ratio is refused (calibrate at that size). Either layout works.

## camera_ws (OpenCV) layout

As `camera_ws` writes it:

```yaml
width: 1280
height: 720
camera_matrix:            # 3 x 3, rows: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
- [906.57, 0.0, 639.34]
- [0.0, 907.25, 384.00]
- [0.0, 0.0, 1.0]
dist_coeffs: [0.1218, -0.3709, 0.0010, 0.0013, 0.3308]   # k1, k2, p1, p2, k3 (plumb_bob)
```

## ROS calibration layout

As `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=/camera/rgb/raw`
saves it:

```yaml
image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [600.0, 0.0, 320.0, 0.0, 601.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1, -0.2, 0.0, 0.0, 0.05]
```

Image size and focal lengths must be positive; fewer than five distortion coefficients are padded
with zeros. A file that does not fit stops the camera node with the reason.
