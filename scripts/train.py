#!/usr/bin/env python3
import argparse
import subprocess
from pathlib import Path
from typing import Optional

from mohou.file import get_project_path


def train_and_visualize(
    project_path: Path,
    n_epoch_vae: int,
    n_epoch_lstm: int,
    n_hidden: int = 200,
    n_layer: int = 2,
    n_aug_vae: int = 2,
    n_aug_lstm: int = 19,
    valid_ratio: float = 0.1,
    n_pb_dim: Optional[int] = None,
):
    cmd_train_vae = "python3 -m mohou.script.train_autoencoder -pp {pp} -n {n} -image RGBImage -latent {n_latent} --vae -bundle_postfix autoencoder -valid-ratio {vr} -aug {aug}".format(
        pp=project_path, n=n_epoch_vae, n_latent=16, vr=valid_ratio, aug=n_aug_vae
    )
    cmd_visualize_vae = "python3 -m mohou.script.visualize_autoencoder_result -pp {pp}".format(
        pp=project_path
    )

    # create train command
    if n_pb_dim is not None:
        lstm_train_script_name = "mohou.script.train_pblstm"
        additional_option = "-pb {}".format(n_pb_dim)
    else:
        lstm_train_script_name = "mohou.script.train_lstm"
        additional_option = ""

    cmd_train_lstm = "python3 -m {script_name} -pp {pp} -n {n} -hidden {hidden} -layer {layer} -valid-ratio {vr} -aug {aug} {add}".format(
        script_name=lstm_train_script_name,
        pp=project_path,
        n=n_epoch_lstm,
        hidden=n_hidden,
        layer=n_layer,
        vr=valid_ratio,
        aug=n_aug_lstm,
        add=additional_option,
    )

    cmd_history = "python3 -m mohou.script.visualize_train_history -pp {}".format(project_path)
    cmd_visualize_lstm = "python3 -m mohou.script.visualize_lstm_result -pp {}".format(project_path)

    cmd_list = []
    cmd_list.append(cmd_train_vae)
    cmd_list.append(cmd_history)
    cmd_list.append(cmd_visualize_vae)
    cmd_list.append(cmd_train_lstm)
    cmd_list.append(cmd_history)
    cmd_list.append(cmd_visualize_lstm)

    for cmd in cmd_list:
        print("running command: {}".format(cmd))
        subprocess.run(cmd, shell=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-pn", type=str, help="project name")
    parser.add_argument("-n_vae", type=int, default=1500, help="number of epoch")
    parser.add_argument("-n_lstm", type=int, default=20000, help="number of epoch")
    parser.add_argument("-n_pb_dim", type=int, help="number of parametric bias dimension")
    parser.add_argument("--test", action="store_true", help="test mode")

    args = parser.parse_args()
    project_name: Optional[str] = args.pn
    n_epoch_vae: int = args.n_vae
    n_epoch_lstm: int = args.n_lstm
    is_testing: bool = args.test
    n_pb_dim: Optional[int] = args.n_pb_dim

    project_path = get_project_path(project_name)
    if is_testing:
        # as fast as possible. run only by rostest
        # note that n_aug_lstm must be required to avoid data generation with small number of samples
        train_and_visualize(
            project_path,
            1,
            1,
            n_hidden=1,
            n_layer=1,
            valid_ratio=0.5,
            n_aug_vae=0,
            n_aug_lstm=4,
            n_pb_dim=n_pb_dim,
        )
    else:
        train_and_visualize(
            project_path, n_epoch_vae, n_epoch_lstm, n_hidden=400, n_layer=2, n_pb_dim=n_pb_dim
        )
