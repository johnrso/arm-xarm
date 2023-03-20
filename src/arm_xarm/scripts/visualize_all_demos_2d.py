import path
import os
import pickle
import numpy as np
import imgviz
import argparse

def main():
    # argparse for a directory
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("saved_dir", type=path.Path, help="dir of all demos")
    args = parser.parse_args()

    all_good_demos = []
    for idx, demo_file in enumerate(sorted(args.saved_dir.listdir())):
        os.system("python /home/xingyu/xarm/src/arm_xarm/scripts/visualize_demos_2d.py " + str(demo_file) + f" {idx}")
        # good_demo = input(f"{str(demo_file)}; is the demo good? (y/n): ")
        # if good_demo.lower() in ['true', '1', 't', 'y', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']:
        #     print(f"adding {str(demo_file)}")
        #     all_good_demos.append(demo_file)

    # print the list nicely
    print("All good demos:")
    for demo_file in all_good_demos:
        print(demo_file)


if __name__ == "__main__":
    main()