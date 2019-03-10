# -*- coding: utf-8 -*-
"""
Created on Tue Jan 22 17:26:25 2019

@author: 2624224
"""
import os
import subprocess
import pickle
import operator
import argparse

import file_utils

def generate_label_files(root_dir, batch_name):
    list_dir = root_dir + '/' + batch_name + '_list/'
    octree_list_path = list_dir + 'octree_list_shuffle.txt'
    with open(octree_list_path) as file:
        lines = file.readlines()

    feature_dir = root_dir + '/' + batch_name + '_feature/'
    labels_paths = file_utils.file_paths_from_dir(feature_dir)

    octree_dir = root_dir + '/' + batch_name + '_octree/'
    points_dir = root_dir + '/points/'
    shape_dir = root_dir + '/shape/'
    for i in range(len(lines)):
        shape_name = lines[i].split('.')[0]

        labels_path = labels_paths[2 * i + 1]
        print(labels_path)
        labels = file_utils.labels_from_file(labels_path)

        label_index_path = octree_dir + shape_name + '.label_index'
        print(label_index_path)
        label_index = file_utils.label_index_from_file(label_index_path)
        
        shape_name = shape_name.split('_')[0]
        points_predicted = [labels[index] for index in label_index]
        points_predicted_path = points_dir + shape_name + '.points_predicted'
        print(points_predicted_path)
        with open(points_predicted_path, 'wb') as f:
            pickle.dump(points_predicted, f)

    # to be debugged
        face_index_path = points_dir + shape_name + '.face_index'
        with open(face_index_path, 'rb') as f:
            face_index = pickle.load(f)

        face_label_map = {}
        for i in range(len(points_predicted)):
            fid = face_index[i]
            label = points_predicted[i]
            if fid not in face_label_map:
                face_label_map[fid] = {label:1}
            else:
                if label in face_label_map[fid]:
                    face_label_map[fid][label] += 1
                else:
                    face_label_map[fid][label] = 1

        face_predicted = [max(face_label_map[key].items(), 
                              key=operator.itemgetter(1))[0] for key in sorted(face_label_map.keys())]
        face_predicted_path = shape_dir + shape_name + '.face_predicted'
        with open(face_predicted_path, 'wb') as f:
            pickle.dump(face_predicted, f)
                

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
    
    feature_dir = ARGS.rootdir + '/' + ARGS.batchname + '_feature/'
    if not os.path.exists(feature_dir):
        os.mkdir(feature_dir)
        
    caffe = ARGS.cafferoot + '/build/tools/Release/caffe.exe'        
    blob_prefix = '--blob_prefix=' + feature_dir
    model = ARGS.cafferoot + '/examples/o-cnn/segmentation_6_test.prototxt'
    weights = ARGS.cafferoot + '/examples/o-cnn/seg_6_cad.caffemodel'
    subprocess.check_call([caffe, 'test', '--model=' + model, '--weights=' + weights, 
                           '--gpu=0', blob_prefix, '--binary_mode=false', '--save_seperately=true', 
                           '--iterations=' + str(ARGS.num)])
    
    generate_label_files(ARGS.rootdir, ARGS.batchname)