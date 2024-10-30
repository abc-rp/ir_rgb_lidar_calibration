# IR-RGB-LiDAR Calibration

**Based on (IV 2023) LVT2Calib: Automatic and Unified Extrinsic Calibration Toolbox for Different 3D LiDAR, Visual Camera, and Thermal Camera**

This fork enhances the original LVT2Calib with specific improvements tailored to xRI's use case, focusing on streamlining IR-RGB-LiDAR calibration workflows. Notable enhancements include an improved calibration board design and better support for fisheye lenses.

For troubleshooting, please refer to the `docs` folder, which contains the original documentation. Alternatively, you can visit the original repository's **"Issues"** section for support.

![fig_lvt2calib_overview](./fig/fig_lvt2calib_overview.png)

## Introduction

This solution provides an automatic and unified method for extrinsic calibration between repetitive and non-repetitive scanning 3D LiDAR, sparse and dense 3D LiDAR, as well as visual and thermal cameras. The repository has been modified to better suit the needs of xRI.

A four-circular-hole calibration board is adopted, enabling the calibration of the aforementioned sensors using the same board. This is because the four circle centers can be detected by the three different types of sensors. To unify the calibration process, the proposed method uses automatic target detection based on template matching. Additionally, two types of output are provided: minimizing 2D re-projection error (Min2D) and minimizing 3D matching error (Min3D), depending on the user's LiDAR-Camera setup.

This approach is ideal for xRI's use case, allowing for accurate extrinsic calibration between Ouster LiDAR, Velodyne LiDAR, RGB cameras, and IR cameras interchangeably.

## How to Use

### Step 1: Environment Configuration

This method can be used in real-time or with pre-recorded ROS bags. As a rough guideline when running the program, it's recommended to use at least 6 positions, each held for at least 10 seconds, with the calibration board placed in varying positions and orientations.

#### 1.1 Install Environment and Drivers

Install the ROS environment, along with the SDK and driver for the specific LiDAR being used. You can skip this step if the environment and drivers are already installed (which they should be on BESS2).

#### 1.2 Dependencies

Tested with Ubuntu 20.04 64-bit.

- ROS Noetic running on a container called "noetic"
- PCL 1.10
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Ceres Solver](http://ceres-solver.org/) (1.14.0)
- OpenCV 4.2.0
- `package.xml`

### Step 2: Preparation

#### 2.1 Download and Installation

Download this repository and compile it:

```bash
git clone https://github.com/abc-rp/ir-rgb-lidar-calibration
cd path_to_your_ir-rgb-lidar-calibration_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

To trial the repository, you may want to download and test it with their calibration ROS bags from OneDrive:
```
https://entuedu-my.sharepoint.com/:f:/g/personal/jzhang061_e_ntu_edu_sg/ElG9hWBSDrRAjaftVeatWzcBDZI-JxeKb3jmu5lMEPfyGw?e=jZvjdj

```
<img src="./fig/fig_lvt2calib_demobag.png" alt="fig_lvt2calib_demobag" style="zoom: 80%;" />

#### 2.2 Preparing the Calibration Board

<img src="./fig/fig_lvt2calib_calibboard.png" alt="fig_lvt2calib_calibboard" style="zoom: 50%;" />

Due to difficulties sourcing the silicone heating pad used in the original repo, we have created a more accessible alternative. Our calibration board is made from an 18mm thick plywood board with 1.2mm thick aluminum sheets mounted on top. The plywood is routed with a trench to fit a 5m pipe heating cable, which is easy to source. The cable is then covered with a thick layer of aluminum tape, followed by the aluminum sheeting. Instructions on fabricating the board can be found in the ```docs``` folder.

<img src="./fig/fig_lvt2calib_calibboard_real.png" alt="fig_lvt2calib_calibboard_real" style="zoom:80%;" />

To ensure smooth and successful calibration, consider the following recommendations:

1. Choose a relatively empty environment with flat and uncluttered backdrop for the calibration scene.
2. Avoid obvious reflections on the ground (to prevent interference in the thermal camera's view).
3. The heating temperature of the calibration plate should not be too high to prevent deformation but should still have a noticeable difference from the ambient temperature (Our soutlion cannot reach these temps).
4. Place the calibration board on a stable support during the calibration process.
5. The distance from the thermal camera to the calibration board should be between 3m and 6m (to ensure image clarity).
6. A heated blanket is recommended to insulate the board between positions and heating before calibration when calirbating thermal sensors.

### Step3: Calibration

#### 3.0 Camera Intrinsics

##### Fisheye Lenses (xRI RGB Cameras)
The original repository does not account for fisheye lens distortion, only conventional lenses. To address this, ```undistort_bag.py``` and ```undistort_real_time.py``` may be used to publish and rectify undistorted images to ```"/("left" or "right")_camera/image_color/undistorted/compressed"``` or within ROS bags. In this case, an undistorted rectification matrix (for a theoretically undistorted image) should be parsed when the camera parameter file is requested when running ```start_up.bash```. If you do not specify a path to a calibration file the defualt is an undistorted rectification matrix. 

##### Conventional Lenses (xRI IR Cameras)

For cameras with conventional lenses (such as xRI's thermal cameras), the aforementioned scripts are unnecessary. Instead, the camera's intrinsic rectification matrix should be parsed into the script when prompted after running ```start_up.bash```.

##### Intrinsic File Format

The cameras intrinsic calibration parameters should be saved as `xxx.txt` in folder `(ir-rgb-lidar-calibration path)/data/camera_info`. The file format should be:

<img src="./fig/fig_lvt2calib_camintrinsic.png" alt="fig_lvt2calib_camintrinsic"  />

*This format applies to all cases (both fisheye and conventional lenses).*

#### 3.1 Quick Start

#### IR Cameras:
The original repository does not allow for you to vary the contrast of the input IR image. To address this ```colourmap_bag.py``` and ```colourmap_real_time.py``` may be used to publish and rectify IR images to:```"/("left" or "right")_ir_camera/image_colormap/compressed"``` or within rosbags. In this case, an undistorted rectification matrix (for a theoretically undistorted image) should be parsed when the camera parameter file is requested when running ```start_up.bash```. If you do not specify a path to a calibration file the defualt is an undistorted rectification matrix. 

#### Run

```shell
cd ~/catkin_ws/src/ir-rgb-lidar-calibration/launch/
bash start_up.bash
```

1. LiDAR-Camera Calibration:

   - If the calibration board is flipped to be **dark** (default as white), please run:

     ```shell
     bash start_up.bash --darkBoard
     ```

      *The board appears white when calibrating the thermal cameras, such ```--darkBoard``` is not required when calibrating the thermal cameras*

   - If calibrating xRI's cameras, then use the **compressed** image topic, as these topics are compressed. Run:

     ```shell
     bash start_up.bash --compressedImg
     ```

   - These two parameters can be used at the same time (no order is required), like:

     ```shell
     bash start_up.bash --compressedImg --darkBoard
     ```

   The terminal feedback:

   <img src="./fig/fig_lvt2calib_bashfeedback.png" alt="fig_lvt2calib_bashfeedback"  />

   According to the prompts, enter topic and namespace of the two sensors. <u>Noted: Each sensor corresponds to a specific namespace, please refer to the table in *Appendix* for details.
   </u> 

   Take their example using the *Livox Horizon* and *RGB camera*:

   <img src="./fig/fig_lvt2calib_bashinput.png" alt="fig_lvt2calib_bashinput"  />

   ##### In our cases:
      - Ouster 128
         ```shell
         sensor1 topic (only LiDAR): /ouster/points
         namespace1: os1_128
         ```
      - RGB Cameras (In this example "right_camera"):
         ```shell
         sensor2 topic (LiDAR or Camera): /right_camera/image_color/undistorted
         namespace2: rgb
         ```
      - IR Cameras (In this example "right_camera"):
         ```shell
         sensor2 topic (LiDAR or Camera): /right_ir_camera/image_colormap/
         namespace2: rgb
         ```

         *Note: We use ```rgb``` not ```thermal```*

         *Note: We do not need to include ```/compressed``` at the end of the topic, as this should have been specfied during ```start_up.bash```*
  
   The type of sensor used is then displayed in the terminal. If it is judged to be a LiDAR-Camera suite, it will continue to prompt for <u>the full path of the camera parameter file (set in *Step 3.0*)</u>.

   You will then be able to input the path to the camera intrinsic file. If you press 'Enter' without any input, it will use the default parameter file *path_to_your_pkg/data/camera_info/intrinsic.txt* which has no effect on the distortion of the input image.

   <img src="./fig/fig_lvt2calib_bashjudge.png" alt="fig_lvt2calib_bashjudge"  />

2. LiDAR-LiDAR Calibration:

   Using their example of a Livox Mid-70 and Ouster OS1-32 suite, the terminal feedback should be:

   <img src="./fig/fig_lvt2calib_LL_startup_t1.png" alt="fig_lvt2calib_LL_startup_t1" style="zoom:50%;" />
   
   **Noted**: If <u>two sensors of the same type</u> are used, that is, the input namespace are the same, for example, two Livox Horizon LiDARs (livox_horizon), just use the same namespace `livox_horizon`. The program will automatically determine, and <u>add suffixes</u> `_1`and `_2` respectively. This may affect users viewing feature extraction results in *Rviz*, requiring manual changed to the observed topic.

Refering to the current terminal as T0. Upon entering the command and pressing 'Enter,' three new terminals will appear, which we weill refer to: T1, T2, and T3.

<img src="./fig/fig_lvt2calib_terminal_pop.png" alt="fig_lvt2calib_terminal_pop" style="zoom: 25%;" />

- T0: At this point, it will enter the calibration process and pause, waiting for the completion of the feature collection process.
- T1: Feature Collection Window. Provide feedback on the progress of feature collection from the two sensors and for interactive control.
- T2 & T3: Pattern Detection Windows. Providing feedback on calibration board detection and feature extraction results from the two sensors, respectively.

<u>Continue with the following steps....</u>

#### 3.2 Feature Extraction

##### Setting parameters

During calibration, it may be necessary to edit the parameters, specfically the Pass-through filter which limits the amount of points in the point cloud.

To do this, run:

```shell
rosrun rqt_reconfigure rqt_reconfigure
```

Vary the X and Y ranges, a good starting point is 5m.

It may be required to view the point cloud in the rviz window (View Topic "(ns_lidar)/laser_pattern/cloud_in") and vary the values in reconfig as required.

***This step is crucial for accurate feature extraction and obtaining precise extrinsic parameters.**

1. For Camera

   Three/Four image windows will pop up: the raw image, the undistorted image, the grayed image (only for rgb cameras) and the result image of circle-center-detection. **Please confirm the successful detection of the circle center features through the result image and terminal T2/T3.**

   <img src="./fig/fig_lvt2calib_circle_cam.png" alt="fig_lvt2calib_circle_cam" style="zoom: 50%;" />

2. For LiDAR

   Note: Pass-through filter can be used in ahead to reduce the operand of point cloud. But please **ensure the background wall of the calibration board is retained in the input point cloud**. Here is and example of input point cloud from OS1-32 LiDAR.

   <img src="./fig/fig_lvt2calib_input_pc_example.png" alt="fig_lvt2calib_input_pc_example" style="zoom: 50%;" />

   Focus on the Rviz window. **Ensure you see the correct calibration board point cloud** on the topic `/(ns_lidar)/laser_pattern/calib_board_cloud`, as shown in the figures: the left one from `/livox_mid70/laser_pattern/calib_board_cloud` and the right one from `/os1_32/laser_patter/calib_board_cloud`. **Prompts will also appear in terminal T2/T3**.

   <img src="./fig/fig_lvt2calib_board_laser.png" alt="fig_lvt2calib_board_laser" style="zoom:45%;" />

***If any issues at this stage, like the camera not detection the four-hole pattern or the LiDAR not recongnizing the carlibtion board, please check the [HELP.md](./docs/HELP.md) for solutions.**

<u>Continue with the following steps....</u>

#### 3.3 Feature Data Collection

1. Focus on terminal T1. The program will ask if you are ready to collect feature data. **Simply type 'y'/'Y' and press 'Enter', and the feature collection will start**.

   <img src="./fig/fig_lvt2calib_t1_ready.png" alt="fig_lvt2calib_t1_ready" style="zoom:33%;" />

2. Point cloud of four-circle-centers detected by LiDAR will be shown in rviz (topic: `/(ns_lidar)/laser_pattern_circle/circle_center_cloud`)

   <img src="./fig/fig_lvt2calib_circle_laser.png" alt="fig_lvt2calib_circle_laser" style="zoom:45%;" />

3. In terminal T1, real-time feedback on how many frames of feature data have been collected by each of the two sensors will be provided.

   <img src="./fig/fig_lvt2calib_t1_feedback_1.png" alt="fig_lvt2calib_t1_feedback_1" style="zoom: 33%;" />

4. Once the predetermined number of features has been collected, the **T1 program will pause and ask if you wish to gather data for the next position**.

   <img src="./fig/fig_lvt2calib_t1_feedback_2.png" alt="fig_lvt2calib_t1_feedback_2" style="zoom:80%;" />

   - If yes, **input 'y'/'Y' followed by 'Enter'. The program will return to the 'READY' interface in step 3.3.1**. You can then adjust the calibration board's position and proceed with a **new round of feature data collection from step 3.2**. (Our paper suggests 9 positions or more)
   - If no, input 'n'/'N' followed by 'Enter' (Note: Please do not directly key *ctrl+c*). The feature detection and collection programs will end while the extrinsic parameter calculation starts (continue to step 3.4).

​	***If the collection takes to long, refer to [HELP.md](./docs/HELP.md) to adjust the preset accumulation frame number.**

#### 3.4 Extrinsic Parameter Calculation

Come back to terminal T0 (running start_up.bash), you will see the extrinsic parameter calculation progress, including the number of feature positions used, the regressed extrinsic matrix, 2D re-projection error (only for camera-related calibration), and 3D matching error. The extrinsic parameters are saved in CSV files:

- LiDAR-LiDAR suite:
  - `(ns_lidar1)_to_(ns_lidar2)_exParam.csv`: the extrinsic parameters from Sensor1 to Sensor2;
  - `L2L_CalibLog.csv`: the log file of each extrinsic parameter calculation result and errors


- LiDAR-Camera suite:

  - `(ns_lidar)_to_(ns_camera)_exParam_min3d.csv`: extrinsic parameters from LiDAR to Camera calculated by minimizing 3d matching error;
  - `(ns_lidar)_to_(ns_camera)_exParam_min2d.csv`: extrinsic parameters from LiDAR to Camera calculated by minimizing 2d re-projection error;

  - `L2C_CalibLog.csv`: the log file of each extrinsic parameter calculation result and errors;

### Step4: Full Usage

1. Feature extraction in LiDAR point cloud

   ```shell
   roslaunch lvt2calib (ns_lidar)_pattern.launch cloud_tp:=(your_lidar_tp) ns_:=(ns_lidar)
   ```

2. Feature extraction in camera image

   ```
   roslaunch lvt2calib (ns_camera)_pattern.launch image_tp:=(your_img_yp) ifCompressed:=(true or false) isDarkBoard:=(true or false) ns_:=(ns_camera) cam_info_dir:=(your_camera_param_filepath)
   ```

   The default value of 'cam_info_dir' is `$(find lvt2calib)/data/camera_info/intrinsic.txt`

3. Feature collection

   ```shell
   # for lidar-lidar calibration
   roslaunch lvt2calib pattern_collection_ll.luanch ns_l1:=(ns_lidar_1) ns_l2:=(ns_lidar_2)
   # for lidar_camera calibration
   roslaunch lvt2calib pattern_collection_lc.launch ns_l:=(ns_lidar) ns_c:=(ns_camera)
   ```

4. Extrinsic Parameter Calculation

   ```shell
   # for lidar-lidar calibration
   roslaunch lvt2calib extrinsic_calib_ll.luanch ns_l1:=(ns_lidar_1) ns_l2:=(ns_lidar_2)
   # for lidar_camera calibration
   roslaunch lvt2calib extrinsic_calib_lc.launch ns_l:=(ns_lidar) ns_c:=(ns_camera)
   ```

## Appendix

#### I. Table of arguments corresponding to sensors

| No.  |      Sensor      | Type | Namespace (`ns`) | Feature extraction .launch file |
| :--: | :--------------: | :--: | :--------------: | :-----------------------------: |
|  1   |  Livox Horizon   | NRL  |  livox_horizon   |  livox_horizon_pattern.launch   |
|  2   |   Livox Mid 70   | NRL  |   livox_mid70    |   livox_mid70_pattern.launch    |
|  3   |   Livox Mid 40   | NRL  |   livox_mid40    |   livox_mid40_pattern.launch    |
|  4   |    Livox Avia    | NRL  |    livox_avia    |    livox_avia_pattern.launch    |
|  5   |   Robosense M1   | NRL  |      rs_m1       |   robosense_m1_pattern.launch   |
|  6   | Velodyne VLP-16  | RL_S |     velo_16      |  livox_velo_16_pattern.launch   |
|  7   | Velodyne VLP-32  | RL_S |     velo_32      |  livox_velo_32_pattern.launch   |
|  8   | Velodyne VLP-64  | RL_D |     velo_64      |  livox_velo_64_pattern.launch   |
|  9   | Velodyne VLP-128 | RL_D |     velo_128     |  livox_velo_128_pattern.launch  |
|  10  |  Ouster OS1-32   | RL_D |      os1_32      |      os1_32_pattern.launch      |
|  11  |  Ouster OS1-64   | RL_D |      os1_64      |      os1_64_pattern.launch      |
|  12  |  Ouster OS1-128  | RL_D |     os1_128      |     os1_128_pattern.launch      |
|  13  |    RGB Camera    |  VC  |       rgb        |     rgb_cam_pattern.launch      |
|  14  |  Thermal Camera  |  TC  |     thermal      |   thermal_cam_pattern.launch    |

NRL: Non-repetitive Scanning LiDAR

RL_S: Sparse Repetitive Scanning LiDAR

RL_D: Dense Repetitive Scanning LiDAR

VC: Visual Camera

TC: Thermal Camera

#### II. Parameter Description for Nodes

Please refer to [HELP.md](./docs/HELP.md).

#### III. Paper

[L2V2T2Calib: Automatic and Unified Extrinsic Calibration Toolbox for Different 3D LiDAR, Visual Camera and Thermal Camera (IEEE Xplore)](https://ieeexplore.ieee.org/document/10186657)

[L2V2T2Calib: Automatic and Unified Extrinsic Calibration Toolbox for Different 3D LiDAR, Visual Camera and Thermal Camera (ResearchGate)](https://www.researchgate.net/publication/371377845_L2V2T2Calib_Automatic_and_Unified_Extrinsic_Calibration_Toolbox_for_Different_3D_LiDAR_Visual_Camera_and_Thermal_Camera)

## Thanks

[1] J. Zhang, R. Zhang, Y. Yue, C. Yang, M. Wen, and D. Wang, “Slat-calib: Extrinsic calibration between a sparse 3d lidar and a limited-fov low-resolution thermal camera,” in *2019 IEEE International Conference on Robotics and Biomimetics (ROBIO)*, pp. 648–653, 2019.

[2] C. Guindel, J. Beltrán, D. Martín, and F. García, “Automatic extrinsic calibration for lidar-stereo vehicle sensor setups,” in *2017 IEEE 20th International Conference on Intelligent Transportation Systems (ITSC)*, pp. 1–6, Oct 2017.

## Citation

If you find this work useful for your research, please consider citing:
```
@INPROCEEDINGS{ZhangLiu2023IV,
  author={Zhang, Jun and Liu, Yiyao and Wen, Mingxing and Yue, Yufeng and Zhang, Haoyuan and Wang, Danwei},
  booktitle={2023 IEEE Intelligent Vehicles Symposium (IV)}, 
  title={L2V2T2Calib: Automatic and Unified Extrinsic Calibration Toolbox for Different 3D LiDAR, Visual Camera and Thermal Camera}, 
  year={2023},
  volume={},
  number={},
  pages={1-7},
  doi={10.1109/IV55152.2023.10186657}}
```
