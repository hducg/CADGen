# -*- coding: utf-8 -*-
"""
Created on Tue Jan 22 17:26:25 2019

@author: 2624224
"""
import os
import subprocess
import argparse
import random

import file_utils

def generate_octree_list(root_dir, depth):
    shape_dir = root_dir + '/shape/'
    shape_names = [name.split('.')[0] for name in file_utils.file_names_from_dir(shape_dir, '.step')]
    random.seed()
    random.shuffle(shape_names)

    octree_list_path = root_dir + '/test_octree_list.txt'
    with open(octree_list_path, 'w') as file:
        for name in shape_names[:int(len(shape_names) * 0.2)]:
            for i in range(12):
                file.write(name + '_' + depth + '_2_%03d.octree\n'%i)

    octree_list_path = root_dir + '/train_octree_list.txt'
    with open(octree_list_path, 'w') as file:
        for name in shape_names[int(len(shape_names) * 0.2):]:
            for i in range(12):
                file.write(name + '_' + depth + '_2_%03d.octree\n'%i)


def generate_lmdb(caffe_root, data_root, depth):
    convert_octree_data = caffe_root + '/build/tools/Release/convert_octree_data.exe'
    octree_dir = data_root + '/octree_' + depth + '_12/'

    lmdb_dir = data_root + '/' + 'test_lmdb'
    if os.path.exists(lmdb_dir):
        os.remove(lmdb_dir)
    octree_list = data_root + '/test_octree_list.txt'
    subprocess.check_call([convert_octree_data, octree_dir, octree_list, lmdb_dir])

    lmdb_dir = data_root + '/' + 'train_lmdb'
    if os.path.exists(lmdb_dir):
        os.remove(lmdb_dir)
    octree_list = data_root + '/train_octree_list.txt'
    subprocess.check_call([convert_octree_data, octree_dir, octree_list, lmdb_dir])
    
    
# modify prototxt specified by --model: data source; num_output of deconv2_ip layer
# set --weights to the right trained caffemoddel
# set --iterations to the number of models to be tested


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument('--cafferoot',
                        '-c',
                        type=str,
                        required=True)
    PARSER.add_argument('--rootdir',
                        '-r',
                        type=str,
                        help='root dir containning the shape dir, points dir, \
                        octree dir, lmdb dir, feature dir, and list dir',
                        required=True)
    PARSER.add_argument('--depth',
                        '-d',
                        type=str,
                        required=True)

    ARGS = PARSER.parse_args()
    
    generate_octree_list(ARGS.rootdir, ARGS.depth)

    generate_lmdb(ARGS.cafferoot, ARGS.rootdir, ARGS.depth)


    solver = ARGS.cafferoot + '/examples/o-cnn/solver_segmentation_6_cad.prototxt'
    weights = ARGS.cafferoot + '/examples/o-cnn/M40_6.caffemodel'

    caffe = ARGS.cafferoot + '/build/tools/Release/caffe.exe'
    subprocess.check_call([caffe, 'train', '--solver=' + solver, '--weights=' + weights])
