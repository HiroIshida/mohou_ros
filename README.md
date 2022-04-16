## Usage

1. Save home position (interactive)
```
rosrun mohou_ros save_home_position.py 
```

2. save rosbags (interactive)
with `--gif` option, gif files of rgb and depth images will be dumped.
```
rosrun mohou_ros save_rosbag.py
```

3. Tune crop filter (interactive)
Quit and save by pressing `q`
```
rosrun mohou_ros tune_image_filter.py 
```

4. Convert to chunk
```
rosrun mohou_ros bags2chunk.py
```

5. Execute 
```
rosrun mohou_ros execute.py
```
