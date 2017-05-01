# Snapdragon Flight DFS-ROS Sample Code

This repo provides the sample code and instructions to run Depth-from-Stereo (DFS) as a ROS node on the [Qualcomm Snapdragon Platform](https://developer.qualcomm.com/hardware/snapdragon-flight)<sup>TM</sup>. 

This example assumes that you are familiar with ROS framework.  If you are new to ROS, refer to [ROS Start Guide](http://wiki.ros.org/ROS/StartGuide) first to get started.

1. [High-level block diagram](#high-level-block-diagram)
1. [Setup and build process](#setup-and-build-process)
  * [Pre-requisites](#pre-requisites)
    * [Platform BSP](#platform-bsp)
    * [Cross-compile build environment](#cross-compile-build-environment)
    * [Install ROS on Snapdragon platform](#install-ros-on-snapdragon-platform)
    * [Install Snapdragon Machine Vision SDK](#install-snapdragon-machine-vision-sdk)
    * [Install Machine Vision SDK license](#install-machine-vision-sdk-license)
    * [Install additional dependency](#install-additional-dependency)
  * [Clone and build sample code](#clone-and-build-sample-code)
  * [Create stereo calibration file](#create-stereo-calibration-file)
1. [Run sample code](#run-sample-code)
  * [Launch DFS ROS node](#launch-dfs-ros-node)
  * [Verification](#verification)

## High-level block diagram
![SnapDfsRosNodeBlockDiagram](images/SnapDfsRosNodeBlockDiagram.png)

## Setup and build process

The current build process is supported only on-target (i.e. on the Snapdragon Flight<sup>TM</sup> Board).  Future release(s) will support off-target cross-compilation on a host computer.

### Pre-requisites

#### Platform BSP

These instructions were tested with version **Flight_3.1.2**. The latest version of the software can be downloaded from [here](http://support.intrinsyc.com/projects/snapdragon-flight/files) and  installed by following the instructions found [here](http://support.intrinsyc.com/projects/snapdragon-flight/wiki)

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

#### Cross-compile build environment

Get the latest Snapdragon Flight<sup>TM</sup> qrlSDK for your Ubuntu 14.04 host computer by following the instructions [here](https://github.com/ATLFlight/ATLFlightDocs/blob/master/AppsGettingStarted.md)

**NOTE**: For this example, you will need the qrlSDK to get the missing files on to the target (see below).  


  1. Platform build setup for camera headers

    **NOTE**: For on-target camera development there are few header files missing on the target, but are part of the qrlSDK.  To fix this, the missing files need to be pushed on to the target.
    This is an interim solution and will be addressed in future releases.

    Push the following missing files to the target:

```
cd <sdk_root_install_path>/sysroots/eagle8074/usr/include
adb push camera.h /usr/include
adb push camera_parameters.h /usr/include 
adb shell sync
```

#### Install ROS on Snapdragon Platform

Refer to the following [page](https://github.com/ATLFlight/ATLFlightDocs/blob/master/SnapdragonROSInstallation.md) for ROS installation on Snapdragon Flight<sup>TM</sup> platform.

#### Install Snapdragon Machine Vision SDK

* These instructions were tested with version **mv0.9.1**. Download the Snapdragon Machine Vision SDK from [here](https://developer.qualcomm.com/sdflight-tools)
* The package name will be mv\<version\>.deb.  
** Example: *mv0.9.1_8x74.deb*
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

#### Install additional dependency

The ROS node uses tinyxml2 to parse the stereo calibration xml file.

Install the library on target using the following command:

```
adb shell
apt-get install -y libtinyxml2-dev
```


### Clone and build sample code

#### Set up ROS workspace on target

```
adb shell
source /home/linaro/.bashrc
mkdir -p /home/linaro/ros_ws/src
cd /home/linaro/ros_ws/src
catkin_init_workspace
cd ../
catkin_make
echo "source /home/linaro/ros_ws/devel/setup.bash" >> /home/linaro/.bashrc
```

This ensures that the ROS workspace is setup correctly.

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

When running on the GPU at VGA resolution with 32 disparity levels, you should see a rate of around 3 Hz. 

To further verify the functionality, a ROS visualization tool like RViz can be used to view the image topics (e.g. /camera/depth/image_raw for the depth image, or /left/image_raw for the raw left stereo image).
