import numpy as np
import os
import pickle

root = "/home/stepjam/.ros/arm_xarm/demo_saved/2022-05-31T16:00:33.011959"

pkl_files = sorted(os.listdir(root))
pkls = []

for _pkl in pkl_files:
    with open(os.path.join(root, _pkl), "rb") as f:
        data = pickle.load(f)
    pkls.append(data)

from IPython import embed
embed()
