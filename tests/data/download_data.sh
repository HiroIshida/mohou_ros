#/bin/bash

function download_pickle {
    local test_data_dir=$(dirname "$0")
    local filename=$test_data_dir/$1
    local drive_id=$2
    wget -nc "https://drive.google.com/uc?export=download&id=$drive_id" -O $filename
}

download_pickle rgb_image.pkl 16LsK7sJSHg1q2QuLz-4Ty_ayKYCebEmy
download_pickle depth_image.pkl 13OlDuoGGnRhiNT7zKZO2IsMY-Dbq7WAv
download_pickle joint_states.pkl 1qennNrT6r-LYg_iBo5D9oliDO9UE9WwP
