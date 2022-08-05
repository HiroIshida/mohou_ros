## mohou_ros [![pytest](https://github.com/HiroIshida/mohou_ros/actions/workflows/test.yaml/badge.svg)](https://github.com/HiroIshida/mohou_ros/actions/workflows/test.yaml) [![lint](https://github.com/HiroIshida/mohou_ros/actions/workflows/format.yaml/badge.svg)](https://github.com/HiroIshida/mohou_ros/actions/workflows/format.yaml) [![rostest-noetic](https://github.com/HiroIshida/mohou_ros/actions/workflows/noetic_test.yaml/badge.svg)](https://github.com/HiroIshida/mohou_ros/actions/workflows/noetic_test.yaml) [![rostest-melodic](https://github.com/HiroIshida/mohou_ros/actions/workflows/melodic_test.yaml/badge.svg)](https://github.com/HiroIshida/mohou_ros/actions/workflows/melodic_test.yaml)

This packages provides a ros-wrapper for [mohou](https://github.com/HiroIshida/mohou), which enables collect data (via kinesthetic teaching or HTC vive controller), data conversion to mohou's format, train, and execute on the real robot.

## installation
install this package as pip 
```
pip3 install -e .
```
If you are scared, please use virtualenv or whatever.
Future direction would be using catkin virtual env.

If you get stuck at installing opencv-python because of skbuild, please refere to
https://stackoverflow.com/questions/63448467/installing-opencv-fails-because-it-cannot-find-skbuild

### Workspace build (Noetic)
Currently, there is no ros pacakge dependency which must be installed from source.
So, no need to create new workspace and you can install this package by
```
source /opt/ros/noetic/setup.bash
mkdir -p ~/mohou_ws/src
cd ~/mohou_ws/src
git clone https://github.com/HiroIshida/mohou_ros.git
wstool init
wstool merge mohou_ros/rosinstall.noetic
wstool update
rosdep install --from-paths . --ignore-src -y -r
cd ~/mohou_ws
catkin init
catkin build
```

### Workspace build (Melodic + Python3)
If you are using melodic, you need to build some packages from source with the following configuration to use python3.
```bash
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy
sudo apt-get install python-catkin-tools python-wstool ros-melodic-rostest
source /opt/ros/melodic/setup.bash
mkdir -p ~/mohou_ws/src
cd ~/mohou_ws/src
git clone https://github.com/HiroIshida/mohou_ros.git
wstool init
wstool merge mohou_ros/rosinstall.melodic
wstool update
rosdep install --from-paths . --ignore-src -y -r
cd ~/mohou_ws
catkin init
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build
```

## Usage
Please note that in the many commands in the following instruction you can omit `-pn {your_project_name}` option by
```bash
echo "primary_project_name: your_project_name" >> ~/.mohou/seting.yaml
```
where `~/.mohou` is the root of the data directory. And "~/.mohou/setting.yaml" configuration file for `mohou`.
Actually if you don't want `~/.mohou` be the root of the data directory, you can change the root by
```bash
echo "root_path: your_root_path" >> ~/.mohou/setting.yaml"
```

### save rosbag
Please save your rosbag files under `~/.mohou/{project_name}/rosbag`. Each rosbag file name must be ended with `.bag` extension.

You can use whatever your favorite way to collect rosbag. To make is easier, this package provides

save_rosbag.py
```bash
rosrun mohou_ros save_rosbag.py -pn {your_project_name} # with `--gif` option, gif files of rgb image will be dumped for debugging
```

Also you can use vive controller to teach pr2-robot and save:
```bash
rosrun mohou_ros vive_controller_pr2.py -pn {your_project_name}
```
the buhttps://github.com/knorth55/eus_vivetton to usage map of vive controllers is follows. Note that right and left arm controllers have a little bit different usage. The following image is took from https://github.com/knorth55/eus_vive
![Vive controller](https://www.vive.com/media/filer_public/e3/da/e3daf208-4d4e-4adf-b911-22f9458ab883/guid-2d5454b7-1225-449c-b5e5-50a5ea4184d6-web.png)

Right arm controller
| Button | Usage |
|:-:|:-:|
| 1 | start / stop saving rosbag |
| 2 | start / stop tracking |
| 7 | grasp / open |
| 8 | return to home position |

Left arm controller
| Button | Usage |
|:-:|:-:|
| 1 | delete the latest rosbag |
| 2 | start / stop tracking |
| 7 | grasp / open |
| 8 | return to home position |

NOTE: when you delete that latest rosbag after stop saving rosbag, please wait few seconds.


### save home position
```
rosrun mohou_ros save_home_position.py 
```


### Tuning the image config
Interactively create image config, which include crop and gaussian blur and hsv filter.
```bash
# press Ctrl-C to quit and save the configuration as `image_config.yaml` under the project folder.
rosrun mohou_ros tune_image_filter.py -pn {your_project_name}
```
`GaussianBlurFilter:kernel_width = 5` is recommended. Altering HSV value is not recommended.
ResolutionChangeResizer:resol change the image resolution. This must be 112 or 224 due to the implementation of `mohou` side.

### Creating chunks
#### convert to the rosbag data to chunk for training the autoencoder.

For better training of autoencoder, much image is required. So we want to create a chunk
with high hz. (20 hz or higher is recommended)
```bash
rosrun mohou_ros bags2chunk.py -hz 20 -remove_policy donothing -pn {your_project_name} -postfix autoencoder -untouch 5
```
Here, untouch means number of episodes which will be kept untouch (will not used in the training). This is helpful when 
you want to use it only for visualization or debugging.

#### convert to the rosbag data to chunk for training the lstm

On the other hand, lstm training require lower frequency data (5hz ~ 8hz) is recommended.
```bash
rosrun mohou_ros bags2chunk.py -hz 5 -remove_policy remove -pn {your_project_name} -untouch 5
```
Here, untouch means number of episodes which will be kept untouch (will not used in the training). This is helpful when 
you want to use it only for visualization or debugging.

#### remove init policy

Sometimes, in the initial phase of the episode, data is static, which is usually bad for learning lstm because the policy becomes long-time-dependent. `remove_policy` may fix such data.

1. `remove_policy = remove`, the such too long static initial sequence will be removed and the removeed sequence will be added to chunk. (recommended if your many of your episode needs to be ammended)

2. `remove_policy = skip`, too long static initial sequence will be skipped and will not be added to the chunk. 

3. `remove_policy = donothing`, regardless of the initial static phase, any episode will be added to the chunk. (recommended for autoencoder training)

Currently theses remover handles only initial state.


### training
first arg is `n_autoencoder_epoch` and the second is `n_lstm_epoch`
```
rosrun mohou_ros train -pn {your_project_name}
```
