# Snapdragon Flight DFS-ROS Sample Code

This repo provides the sample code and instructions to run Depth-from-Stereo (DFS) as a ROS node on the [Qualcomm Snapdragon Platform](https://developer.qualcomm.com/hardware/snapdragon-flight)<sup>TM</sup>. 

This example assumes that you are familiar with ROS framework.  If you are new to ROS, refer to [ROS Start Guide](http://wiki.ros.org/ROS/StartGuide) first to get started.

1. [Stereo vision background](#stereo-vision-background)
1. [High-level block diagram](#high-level-block-diagram)
1. [Setup and build process](#setup-and-build-process)
  * [Pre-requisites](#pre-requisites)
    * [Platform BSP](#platform-bsp)
    * [Install ROS on Snapdragon platform](#install-ros-on-snapdragon-platform)
    * [Install additional dependency](#install-additional-dependency)
    * [Install Snapdragon Machine Vision SDK](#install-snapdragon-machine-vision-sdk)
    * [Install Machine Vision SDK license](#install-machine-vision-sdk-license)
  * [Clone and build sample code](#clone-and-build-sample-code)
  * [Create stereo calibration file](#create-stereo-calibration-file)
1. [Run sample code](#run-sample-code)
  * [Launch DFS ROS node](#launch-dfs-ros-node)
  * [Verification](#verification)
1. [Achieving higher frame rates](#achieving-higher-frame-rates)

## Stereo vision background

For background on the fundamentals of computer stereo vision, please see this Wikipedia entry: https://en.wikipedia.org/wiki/Computer_stereo_vision.

## High-level block diagram
![SnapDfsRosNodeBlockDiagram](images/SnapDfsRosNodeBlockDiagram.png)

## Setup and build process

The current build process is supported only on-target (i.e. on the Snapdragon Flight<sup>TM</sup> Board).  Future release(s) will support off-target cross-compilation on a host computer.

### Pre-requisites

#### Platform BSP

These instructions were tested with version **Flight_3.1.3.1**. The latest version of the software can be downloaded from [here](http://support.intrinsyc.com/projects/snapdragon-flight/files) and installed by following the instructions found [here](http://support.intrinsyc.com/projects/snapdragon-flight/wiki)

**NOTE**: By default the HOME environment variable is not set.  Set this up doing the following:

```
adb shell
chmod +rw /home/linaro
echo "export HOME=/home/linaro/" >> /home/linaro/.bashrc
```

If you use SSH, the home environment variable should be set correct after the above step. 
If you use ADB, do the following for each session:

```
adb shell
source /home/linaro/.bashrc
```

#### Install ROS on Snapdragon Platform

Refer to the following [page](https://github.com/ATLFlight/ATLFlightDocs/blob/master/SnapdragonROSInstallation.md) for ROS installation on Snapdragon Flight<sup>TM</sup> platform.

#### Install additional dependency

The ROS node uses tinyxml2 to parse the stereo calibration xml file.

Install the library on target using the following command (make sure you are in station mode so you have internet access):

```
adb shell
apt-get install -y libtinyxml2-dev
```

#### Install Snapdragon Machine Vision SDK

* These instructions were tested with version **mv1.0.2**. Download the Snapdragon Machine Vision SDK from [here](https://developer.qualcomm.com/hardware/snapdragon-flight/machine-vision-sdk)
* The package name will be mv\<version\>.deb.  
** Example: *mv_1.0.2_8x74.deb*
* push the deb package to the target and install it.

```
adb push mv<version>.deb /home/linaro
adb shell sync
adb shell
dpkg -i /home/linaro/mv<version>.deb
```

#### Install Machine Vision SDK license

The Machine Vision SDK will need a license file to run.  Obtain a research and development license file from [here](https://developer.qualcomm.com/sdflight-key-req)

The license file needs to be placed at the same location as the MV SDK library **libmv1.so**.

Push the license file to the target using the following command:

```
adb push snapdragon-flight-license.bin /usr/lib
adb shell sync
```

### Clone and build sample code

#### Clone the sample code
* The repo may be cloned from here directly on the target, or cloned on the host computer and then pushed to the target using ADB. The recommended method is to clone directly on the target.

```
adb shell
source /home/linaro/.bashrc
roscd
cd ../src
git clone https://github.com/ATLFlight/dfs-ros-example.git
```

* Build the code

```
adb shell
source /home/linaro/.bashrc
roscd
cd ..
catkin_make install
```

**NOTE**: To clean the code, remove the "build" folder

```
adb shell
source /home/linaro/.bashrc
roscd
cd ../
rm -rf build
```

### Create stereo calibration file

To be able to compute depth from a pair of stereo images, the DFS algorithm needs a calibration file specifying the intrinsic parameters (focal length, principal point, radial distortion coefficients) of each camera, and extrinsic parameters (rotation, translation).

The calibration procedure is defined [here](StereoCalibration.md) to compute the above calibration parameters.

The sample code provided here includes an XML parsing function, which assumes a certain format of the XML file. See the example provided in \<ros_ws\>/src/dfs-ros-example/calibration_example/Configuration.Stereo.vga.xml for how to format your stereo calibration file.

## Run sample code

This example assumes that the user is familiar with ROS framework.  If you are new to ROS, refer to [ROS Start Guide](http://wiki.ros.org/ROS/StartGuide) first to get started.

This assumes that the ROS build command is successful.

**NOTE**: For convenience, a sample ROS launch file is provided for testing the ROS node. 

### Launch DFS ROS node

You can launch the sample launch file using the roslaunch command as follows. Note that you must pass the full path to your stereo calibration XML file as an argument to roslaunch.

```
adb shell
source /home/linaro/.bashrc
roscd
roslaunch snap_dfs dfs_test.launch config_filename:=<full_path_to_your_stereo_calibration_file> param_filename:=dfs_test.yaml
```

Example roslaunch command

```
roslaunch snap_dfs dfs_test.launch config_filename:=/home/linaro/ros_ws/src/dfs-ros-example/calibration_example/Configuration.Stereo.vga.xml param_filename:=dfs_test.yaml
```

### Verification

To verify that it is running, run the following command in a different shell instance (optional):

```
adb shell
source /home/linaro/.bashrc
rostopic list
rostopic hz /camera/depth/image_raw
```

When running on the GPU at VGA resolution with 32 disparity levels, you should see a rate of around 2.5-3 Hz. 

Note that in the current implementation, the depth processing blocks the getting of the next camera frame (see Snapdragon::DfsManager::DfsCamProcessingMain()). As a result, you will see the same frame rate for the raw images as for the depth images. To get raw camera frames at the full frame rate (e.g. 30 Hz) without depth images, comment out the call to mvDFS_GetDepths() and rebuild.

To further verify the functionality, a ROS visualization tool like RViz can be used to view the image topics (e.g. /camera/depth/image_raw for the depth image, or /left/image_raw for the raw left stereo image).

## Achieving higher frame rates

Here are a few ways to increase the frame rate of DFS:

* Use QVA resolution. In order to use QVGA resolution, you need a calibration file corresponding to QVGA. (The resolution is set by reading the 'size' attribute in the calibration file.) So for QVGA, set the size to 320x240, and make sure to divide the principal points and focal lengths by 2 as well.

* Run in performance mode, using the provided script, setperfmode.sh.

* Decrease the number of disparity levels. For 28 disparity levels, assuming narrow FOV cameras with focal length ~217, this would correspond to a closest detectable distance of 0.6 m.

Here are some example performance measurements of DFS:

VGA, 32 disparity levels: 2.5 Hz

QVGA, 32 disparity levels: 14 Hz

QVGA, 32 disparity levels, performance mode: 19 Hz

QVGA, 28 disparity levels, performance mode: 22 Hz
