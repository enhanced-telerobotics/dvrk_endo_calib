# dvrk_endo_calib

ROS 2 tools for collecting a hand-guided dVRK ECM workspace and stereo endoscope images.

## Build

Source your ROS 2 and dVRK workspaces so `crtk_msgs` is available. Install the package dependencies with rosdep, then build and source this package from your ROS 2 workspace root.

```bash
colcon build --packages-select dvrk_endo_calib
source install/setup.bash
```

## Record a workspace

```bash
ros2 run dvrk_endo_calib workspace_collect
```

Hand-guide the ECM with the checkerboard visible in both views. Press Ctrl+C to save. The recorder subscribes to `/ECM/measured_js` and saves a timestamped JSON file under `~/.config/dvrk_endo_calib/`. It samples up to 10 times per second and skips nearby repeats. Use `--arm /namespace/ECM` or `--rate 5` to change those defaults.

## Collect stereo pairs

```bash
ros2 run dvrk_endo_calib image_collect                  # 100 valid stereo pairs
ros2 run dvrk_endo_calib image_collect --count 25
ros2 run dvrk_endo_calib image_collect --checkerboard 12 9 --square-size-mm 10
```

Start the dVRK ROS bridge and stereo publishers, then home and enable the ECM and release the clutch. Use a clear workspace for direct moves between recorded points. The collector loads the newest workspace, shuffles its recorded joint targets, and reshuffles when another pass is needed.

Each `/ECM/move_jp` command must finish and the measured joints must stay within the target tolerances during settling. The tolerances are 0.005 rad for yaw and pitch, 1 mm for insertion, and 0.02 rad for roll. The collector then waits for fresh compressed images from `/stereo_source/left/camera/image_raw/compressed` and `/stereo_source/right/camera/image_raw/compressed`. Both publishers must stamp images in the collector's ROS clock domain. Defaults are `--settle 1.0`, `--max-skew 0.02`, and `--timeout 20`, in seconds.

On a motion timeout, the collector reports the controller state and joint errors. It skips the pose and continues after a confirmed busy-to-idle transition with fresh joint feedback. If the move is interrupted or the arm's motion status cannot be confirmed, the collector requests `/ECM/hold` and stops the run.

Pass the printed square counts as `--checkerboard COLS ROWS` and the side length of one square as `--square-size-mm`. The defaults are 12 columns by 9 rows of squares and a 5 mm side length. The collector converts the square counts to 11 columns by 8 rows of inner corners for OpenCV. Both frames must show the full inner-corner grid for the pair to be saved. A failed detection or stereo capture timeout skips that pose and collection continues.

`--count` specifies the number of valid pairs to save. The collector tries up to 1.5 times that many poses, rounded down to a whole attempt. If it reaches the limit before collecting the requested count, it keeps the saved pairs and exits with an error.

Each run is saved to `~/.config/dvrk_endo_calib/captures/<timestamp>/` by default. Give MATLAB that run's `left/` and `right/` folders. Images have matching sequential numbers across the two folders. The collector writes `run.yaml` at the start with the initial measured ECM pose, the source workspace, the checkerboard dimensions, and the collection settings. Use `--workspace` to select a saved workspace or `--output-dir` to change the capture parent directory.

## Calibrate in MATLAB and export to ROS

In MATLAB with the Computer Vision Toolbox, run these commands from this repository after capture finishes.

```matlab
addpath('matlab');
runDir = open_stereo_calibrator();
```

This opens a new Stereo Camera Calibrator session with both image folders from the latest run and reads the square size from `run.yaml`. Use `open_stereo_calibrator("/path/to/run")` for another run, or pass the square size in millimeters as a second argument. To resume a saved GUI session, call `stereoCameraCalibrator("/path/to/session.mat")`.

For the 60° endoscope, start with these Camera Model options.

- **Radial Distortion — 2 Coefficients.** If distortion remains near the edges, try 3. Keep the third coefficient if it improves rectification on checkerboard pairs held out from calibration.
- **Skew — unchecked.** `save2yaml` uses `stereoParametersToOpenCV`, which rejects nonzero skew.
- **Tangential Distortion — checked.** MATLAB estimates `p1` and `p2`, which the ROS export supports.

Calibrate in the app, review the reprojection errors, and remove outliers as needed. Click **Show Rectified Images** and check that matching checkerboard corners land on the same rows in both views, especially near the edges. Export `stereoParams` to the MATLAB workspace and run the exporter.

```matlab
save2yaml(stereoParams, runDir);
```

`left.yaml` and `right.yaml` contain the ROS `CameraInfo` fields `D`, `K`, `R`, and `P` and use the default dVRK stereo-source camera names. Pass `LeftName` and `RightName` if your driver uses different names. The exporter computes rectification in MATLAB and centers its valid view on the raw-size canvas that ROS `image_proc` uses. `stereo_extrinsics.yaml` contains the raw left-to-right transform `T` and baseline in meters. Multiply a homogeneous point in the unrectified left optical frame by `T` to express it in the unrectified right optical frame.
