#/bin/bash

function download_pickle {
    local test_data_dir=$(dirname "$0")
    local filename=$test_data_dir/$1
    local drive_id=$2
    wget -nc "https://drive.google.com/uc?export=download&id=$drive_id" -O $filename
}

download_pickle rgb_image.pkl 1kSjcV7owDd7qpCetXEXHsiDUupMT0n62
download_pickle depth_image.pkl 1zHLPCm3FcuL92tyIBaG9ElB3PV4Lj_mW
download_pickle joint_states.pkl 1JAWfQrZ1FJgJSlg-aoKvJLPIOHkoQ2eO
