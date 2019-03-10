# -*- coding: utf-8 -*-
"""
Created on Fri Feb 22 16:24:02 2019

@author: 2624224
"""
import os
import subprocess
import argparse
import random

import file_utils

class DatasetGenerator:
    def __init__(self, caffe_root, ocnn_root, root_dir, batch_name, num_shapes):
        '''
        '''

        self.caffe_root = caffe_root
        self.ocnn_root = ocnn_root
        root_dir += '/'
        self.category_name = batch_name
        self.num_shapes = num_shapes

        self.shape_dir = root_dir + 'shape/'
        self.points_dir = root_dir + 'points/'
        self.octree_dir = root_dir + self.category_name + '_octree/'
        self.lmdb_dir = root_dir + self.category_name + '_lmdb/'
        self.list_dir = root_dir + self.category_name + '_list/'

        dir_list = [self.octree_dir, self.list_dir]
        for path in dir_list:
            if not os.path.exists(path):
                os.mkdir(path)

        points_list_path = root_dir + 'points_list.txt'
        with open(points_list_path) as file:
            points_list = file.readlines()
            
        random.seed()
        random.shuffle(points_list)
        points_list_path = self.list_dir + 'points_list.txt'
        with open(points_list_path, 'w') as file:
            file.writelines(points_list[:num_shapes])


    def generate_octree(self, depth='6', rot_num='1'):
        '''
        '''
        octree = self.ocnn_root + '/octree.exe'
        points_list = self.list_dir + 'points_list.txt'
        subprocess.check_call([octree, '--filenames', points_list, '--output_path', self.octree_dir,
                               '--depth', depth, '--rot_num', rot_num])
        octree_names = [octree_name + '\n' for octree_name in file_utils.file_names_from_dir(self.octree_dir, '.octree')]
        file_path = self.list_dir + 'octree_list.txt'
        with open(file_path, 'w') as file:
            file.writelines(octree_names)


    def generate_lmdb(self):
        convert_octree_data = self.caffe_root + '/convert_octree_data.exe'
        if os.path.exists(self.lmdb_dir):
            os.remove(self.lmdb_dir)
        octree_list = self.list_dir + 'octree_list.txt'
        subprocess.check_call([convert_octree_data, self.octree_dir, octree_list, self.lmdb_dir])


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument('--cafferoot',
                        '-c',
                        type=str,
                        required=True)
    PARSER.add_argument('--ocnnroot',
                        '-o',
                        type=str,
                        required=True)
    PARSER.add_argument('--rootdir',
                        '-r',
                        type=str,
                        help='root dir containning the shape dir, points dir, \
                        octree dir, lmdb dir, feature dir, and list dir',
                    required=True)
    PARSER.add_argument('--batchname',
                        '-b',
                        type=str,
                        help='category name of the dataset, used to name other dirs',
                        required=False,
                        default='')
    PARSER.add_argument('--num',
                        '-n',
                        type=int,
                        help='number of shapes to generate',
                        required=False,
                        default=1)
    ARGS = PARSER.parse_args()
    DATASET_GENERATOR = DatasetGenerator(ARGS.cafferoot, ARGS.ocnnroot,
                                         ARGS.rootdir, ARGS.batchname, ARGS.num)

#1. filenames, output_path --> octree.exe --> *.octree, *.label_index
    DATASET_GENERATOR.generate_octree()
#2. rootfolder, listfile, db_name --> convert_octree_data --> lmdb, octree_list_name
    DATASET_GENERATOR.generate_lmdb(ARGS.rootdir)
#3. prototxt, caffemodel --> caffe test --> *.label_groundtruth, *.label_predicted
#    blob_prefix = feature_dir
#    model_path = 'D:/Weijuan/caffe/examples/o-cnn/segmentation_6_test.prototxt'
#    weights_path = 'D:/Weijuan/caffe/examples/o-cnn/seg_6_cad.caffemodel'
#    subprocess.check_call([caffe, 'test', '--model=' + model_path, '--weights=' + weights_path, '--gpu=0', '--blob_prefix=' + blob_prefix,
#    '--binary_mode=false', '--save_seperately=true', '--iterations=' + str(num_shapes)])
#4. generate label files
#    DATASET_GENERATOR.generate_label_files()
