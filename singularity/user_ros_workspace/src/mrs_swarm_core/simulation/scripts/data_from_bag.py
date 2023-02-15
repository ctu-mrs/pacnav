#!/usr/bin/env python3

##
# @file data_from_bag.py
# @brief This script processes all the bags from a given experiment to extract data.
# It is assmued that the bag files have following dir heirarchy:
# exp_dir/exp_number/bag_file.bag, where exp_dir is provided as an arg
# @author Afzal Ahmad
# @version 0.1
# @date 2021-10-29


import math
import subprocess, shlex, psutil
import os
import sys
import time
import argparse
import rospkg


if __name__ == '__main__':
    # read params from command line
    parser = argparse.ArgumentParser(description='extracting data from a collection of bag files from an experiment')

    # root dir of the experiment data
    parser.add_argument('--exp_dir', type=str, help='path to dir with experiment bag files')
    parser.add_argument('--config', type=str, help='config file for params of extractor node')

    args, unk = parser.parse_known_args()
    # print(args)

    exp_paths = os.scandir(args.exp_dir)

    # get path of the extractor script
    rp = rospkg.RosPack()
    package_path = rp.get_path('data_extraction')
    extractor_path = package_path + '/scripts/extractor.py'

    # start roscore needed for the extractor script
    ros_proc = subprocess.Popen(shlex.split('roscore'), stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)

    # sleep to let roscore start
    time.sleep(5)

    # go thorugh each experiment dir
    for p in exp_paths:

        if p.is_dir():
            bag_paths = os.scandir(p.path)

            # go through each bag file in the experiment
            for bp in bag_paths:

                # prevents from reading the dir created by extractor for data
                if bp.is_file() and bp.path.find('active') == -1:

                    file_path = os.path.abspath(bp.path)
                    uav_name = bp.path.split('/')[-1].split('.')[0]
                    save_path = os.path.abspath(p.path)
                    config = args.config

                    print('processing bag file: {}'.format(file_path))

                    extract_cmd = shlex.split('python ' + extractor_path + ' --uav_name ' + uav_name + ' --bag_path ' + file_path + ' --save_path ' + save_path + ' --config_file ' + config + ' --time_crop True')

                    process = subprocess.Popen(extract_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)

                    # waits for the process to end, VERY IMPORTANT, duplicate ros servers can emerge
                    try:
                        stdout, stderr = process.communicate(timeout=600)
                        # print(stdout)
                        # print(stderr)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        stdout, stderr = process.communicate()
                        # print(stdout)
                        # print(stderr)

    # kill roscore before exit
    ros_proc.kill()
