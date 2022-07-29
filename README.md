## mohou_ros [![pytest](https://github.com/HiroIshida/mohou_ros/actions/workflows/test.yaml/badge.svg)](https://github.com/HiroIshida/mohou_ros/actions/workflows/test.yaml) [![lint](https://github.com/HiroIshida/mohou_ros/actions/workflows/format.yaml/badge.svg)](https://github.com/HiroIshida/mohou_ros/actions/workflows/format.yaml) [![rostest-noetic](https://github.com/HiroIshida/mohou_ros/actions/workflows/noetic_test.yaml/badge.svg)](https://github.com/HiroIshida/mohou_ros/actions/workflows/noetic_test.yaml)

Collection of peripheral functions around [mohou](https://github.com/HiroIshida/mohou) and ros wrapper.

## installation
install this package as pip 
```
pip3 install -e .
```
If you are scared, please use virtualenv or whatever.
Future direction would be using catkin virtual env.

If you get stuck at installing opencv-python because of skbuild, please refere to
https://stackoverflow.com/questions/63448467/installing-opencv-fails-because-it-cannot-find-skbuild

Currently, there is no ros pacakge dependency which must be installed from source.
So, no need to create new workspace and you can install this package by
```
rosdep install --from-paths . -i -r -y
catkin bt
```

### Workspace build (Melodic + cv-bridge-python3)
If you are using melodic, you need to build cv-bridge with the following configuration to use python3.
```bash
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy
sudo apt-get install python3-opencv
sudo apt-get install ros-melodic-catkin
source /opt/ros/melodic/setup.bash
mkdir -p ~/mohou_ws/src
cd ~/mohou_ws/src
git clone https://github.com/HiroIshida/mohou_ros.git
rosdep install --from-paths . --ignore-src -y -r
cd ~/mohou_ws
catkin init
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
rosrun mohou_ros train 1500 20000
```
