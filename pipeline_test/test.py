# /usr/bin/env python3
import subprocess

import gdown
from mohou.file import get_project_path


def drive_url(file_id):
    url = "https://drive.google.com/uc?id={}".format(file_id)
    return url


# prepare rosbag
project_name = "_mohou_ros_utils_test"
project_path = get_project_path(project_name)

url = drive_url("18EtBZHK1SIxgGMKOrITd_Eg5MK3KnQXA")
zip_path = project_path / "rosbag.zip"
gdown.download(url=url, output=str(zip_path), quiet=False)

rosbag_path = project_path
rosbag_path.mkdir(exist_ok=True)
subprocess.call("unzip -o {} -d {}".format(zip_path, rosbag_path), shell=True)

# prepare config files
url = drive_url("1LAF9JklX0NLUK_3isQOyFjNHSj-UzzWI")
image_config_path = project_path / "image_config.yaml"
gdown.download(url=url, output=str(image_config_path), quiet=True)

url = drive_url("1_d2ijjxXTzmsfADccuwY2t8DqaYC6OyK")
main_config_path = project_path / "main_config.yaml"
gdown.download(url=url, output=str(main_config_path), quiet=True)

# run commands
subprocess.run(
    "rosrun mohou_ros bags2chunk.py -hz 5 -remove_policy donothing -pn {}".format(project_name),
    shell=True,
)
subprocess.run(
    "rosrun mohou_ros bags2chunk.py -hz 20 -postfix autoencoder -remove_policy remove -pn {}".format(
        project_name
    ),
    shell=True,
)
n_epoch_ae = 1
n_epoch_lstm = 1
subprocess.run(
    "rosrun mohou_ros train {} {} {}".format(project_name, n_epoch_ae, n_epoch_lstm),
    shell=True,
)
