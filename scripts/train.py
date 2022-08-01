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
    valid_ratio: float = 0.1,
):
    cmd_train_vae = "python3 -m mohou.script.train_autoencoder -pp {pp} -n {n} -image RGBImage -latent {n_latent} --vae -bundle_postfix autoencoder -valid-ratio {vr}".format(
        pp=project_path, n=n_epoch_vae, n_latent=16, vr=valid_ratio
    )
    cmd_visualize_vae = "python3 -m mohou.script.visualize_autoencoder_result -pp {pp}".format(
        pp=project_path
    )
    cmd_train_lstm = "python3 -m mohou.script.train_lstm -pp {pp} -n {n} -aug 19 -hidden {hidden} -layer {layer} -valid-ratio {vr}".format(
        pp=project_path, n=n_epoch_lstm, hidden=n_hidden, layer=n_layer, vr=valid_ratio
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
    parser.add_argument("--test", action="store_true", help="test mode")

    args = parser.parse_args()
    project_name: Optional[str] = args.pn
    n_epoch_vae: int = args.n_vae
    n_epoch_lstm: int = args.n_lstm
    is_testing: bool = args.test

    project_path = get_project_path(project_name)
    if is_testing:
        # as fast as possible. run only by rostest
        train_and_visualize(project_path, 1, 1, n_hidden=1, n_layer=1, valid_ratio=0.5)
    else:
        train_and_visualize(project_path, n_epoch_vae, n_epoch_lstm, n_hidden=400, n_layer=2)
