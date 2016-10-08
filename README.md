# Kinect-v2-Tutorial

## Kinect v2 with ROS
### Install the Driver for Kinect v2

* Download libfreenect2 source

```bash
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
```

* (Ubuntu 14.04 only) Download upgrade deb files

```bash
cd depends; ./download_debs_trusty.sh
```

* Install build tools
```bash
sudo apt-get install build-essential cmake pkg-config
```
* Install libusb. The version must be >= 1.0.20 .
1. (Ubuntu 14.04 only) ```sudo dpkg -i debs/libusb*deb```
1. (Other) ```sudo apt-get install libusb-1.0-0-dev```

* Install TurboJPEG
1. (Ubuntu 14.04 and newer) ```sudo apt-get install libturbojpeg libjpeg-turbo8-dev```
1. (Debian) ```sudo apt-get install libturbojpeg0-dev```

* Install OpenGL
1. (Ubuntu 14.04 only) ```sudo dpkg -i debs/libglfw3*deb; sudo apt-get install -f; sudo apt-get install libgl1-mesa-dri-lts-vivid```
 (If the last command conflicts with other packages, don't do it.)
1. (Odroid XU4) OpenGL 3.1 is not supported on this platform. Use `cmake -DENABLE_OPENGL=OFF` later.
1. (Other) ```sudo apt-get install libglfw3-dev```

* Install OpenCL (optional)
If you are using **Intel GPU**:
(Ubuntu 14.04 only) 
```bash
sudo apt-add-repository ppa:floe/beignet; sudo apt-get update; sudo apt-get install beignet-dev; sudo dpkg -i debs/ocl-icd*deb	
```
If you are using **AMD GPU**:
Install the latest version of the AMD Catalyst drivers from [https://support.amd.com](https://support.amd.com) and ```apt-get install opencl-headers```.

* Install CUDA (optional, Nvidia only):
(Ubuntu 14.04 only) Download ```cuda-repo-ubuntu1404...*.deb``` ("deb (network)") from [Nvidia website](https://developer.nvidia.com/cuda-downloads), follow their installation instructions, including ```apt-get install cuda``` which installs Nvidia graphics driver.

* Install VAAPI (optional, Intel only)
(Ubuntu 14.04 only) ```sudo dpkg -i debs/{libva,i965}*deb; sudo apt-get install -f```

* Install OpenNI2 (optional)
(Ubuntu 14.04 only) ```sudo apt-add-repository ppa:deb-rob/ros-trusty && sudo apt-get update``` (You don't need this if you have ROS repos), then ```sudo apt-get install libopenni2-dev```

* Build
```bash
cd ..
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
```

* Set up udev rules for device access: ```sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/```, then **replug the Kinect**.
* Run the test program: ./bin/Protonect

If you encounter the following error, it's very likely that you haven't installed the Nvidia driver.
```bash
Version: 0.2.0
Environment variables: LOGFILE=<protonect.log>
Usage: ./bin/Protonect [-gpu=<id>] [gl | cl | cuda | cpu] [<device serial>]
        [-noviewer] [-norgb | -nodepth] [-help] [-version]
        [-frames <number of frames to process>]
To pause and unpause: pkill -USR1 Protonect
[Info] [Freenect2Impl] enumerating devices...
[Info] [Freenect2Impl] 8 usb devices connected
[Info] [Freenect2Impl] found valid Kinect v2 @2:5 with serial 035623243247
[Info] [Freenect2Impl] found 1 devices
libGL error: failed to load driver: swrast
[Error] [OpenGLDepthPacketProcessorImpl] GLFW error 65543 GLX: Failed to create context: BadMatch (invalid parameter attributes)
[Error] [OpenGLDepthPacketProcessor] Failed to create opengl window.
```
To solve this problem, just install the nvidia driver for your ubuntu system. For example, if you are using GeForce GTX 960, then run:
```bash
sudo apt-get install nvidia-352
```

* Run OpenNI2 test (optional): ```sudo apt-get install openni2-utils && sudo make install-openni2 && NiViewer2```. Environment variable LIBFREENECT2_PIPELINE can be set to cl, cuda, etc to specify the pipeline.

* Clone this repository into your catkin workspace, install the dependencies and build it
```bash
cd ~/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
```
Note: rosdep will output errors on not being able to locate **[kinect2_bridge]** and **[depth_registration]**. That is fine because they are all part of the iai_kinect2 package and rosdep does not know these packages.

Note: If you installed libfreenect2 somewhere else than in `$HOME/freenect2` or a standard location like `/usr/local` you have to specify the path to it by adding` -Dfreenect2_DIR=path_to_freenect2/lib/cmake/freenect2` to catkin_make.

### Running Example
* Connect your sensor and run kinect2_bridge:
```bash
roslaunch kinect2_bridge kinect2_bridge.launch
```

* View Cloud
```bash
rosrun kinect2_viewer kinect2_viewer sd cloud
```

## Calibration Tutorial
