# -*- coding: utf-8 -*-
"""
Created on Fri Feb 22 16:24:02 2019

@author: 2624224
"""
import pickle
import os
import argparse
from multiprocessing import Pool

import file_utils
import occ_utils
import shape_factory
import point_cloud


def generate_shape(root_dir):
    '''
    generate num_shapes random shapes in shape_dir
    '''
    shape_dir = root_dir + 'shape/'

    shape, label_map, id_map, shape_name = shape_factory.shape_drain()
    print(shape_name)
    step_path = shape_dir + shape_name + '.step'
#    print(step_path)
    if not os.path.exists(step_path):
        occ_utils.shape_with_fid_to_step(step_path, shape, id_map)

        face_truth_path = shape_dir + shape_name + '.face_truth'
        face_truth = [label_map[face] for face, fid in
                      sorted(id_map.items(), key=lambda kv: kv[1])]
        with open(face_truth_path, 'wb') as file:
            pickle.dump(face_truth, file)


def generate_points(shape_path):
    '''
    generate points for shapes listed in CATEGORY_NAME_step.txt
    '''
    shape_dir = shape_path.partition('shape')[0] + 'shape/'
    points_dir = shape_path.partition('shape')[0] + 'points/'

    shape_name = shape_path.split('/')[-1].split('.')[0]
    file_path = points_dir + shape_name + '.points'
    if os.path.exists(file_path):
        return
#    print(file_path)
    shape, id_map = occ_utils.shape_with_fid_from_step(shape_path)
    face_truth_path = shape_dir + shape_name + '.face_truth'
    with open(face_truth_path, 'rb') as file:
        face_truth = pickle.load(file)
    label_map = {face: face_truth[id_map[face]] for face in id_map}

    res = point_cloud.resolution_from_shape(shape)
#            print('resolution', res)
    pts, normals, feats, segs, face_ids = point_cloud.point_cloud_from_labeled_shape(shape, label_map, id_map, res)
    file_utils.upgraded_point_cloud_to_file(file_path, pts, normals, feats, segs)

    face_index_path = points_dir + shape_name + '.face_index'
    with open(face_index_path, 'wb') as file:
        pickle.dump(face_ids, file)


if __name__ == '__main__':
    '''
    input:
        rootdir, dir where to put generated shapes, points, octrees, lmdb, and
        features, must exist

        num, number of shapes to generate

    output:
        shape and points dir under rootdir

        num *.step files and num *.face_truth files in shape dir.
        In the step files, each face has a unique id, in a range of [0, number
        of faces)
        the face_truth file contains face labels ordered by face id.

        num *.points files and num *.face_index files in points dir
        points file contains a list of points coordinates, normals, labels and
        features
        face_index contains face id for each point, in the same order as in points
        file

        each shape has a unique name, and is associated with a step file,
        a face_truth file, a points file, a face_index file, all with the same
        prefix but different extensions
    '''
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument('--rootdir',
                        '-r',
                        type=str,
                        help='root dir containning the shape dir, points dir, \
                        octree dir, lmdb dir, feature dir, and list dir',
                        required=True)
    PARSER.add_argument('--num',
                        '-n',
                        type=int,
                        help='number of shapes to generate',
                        required=False,
                        default=1)
    ARGS = PARSER.parse_args()

    root_dir = ARGS.rootdir + '/'
    shape_dir = root_dir + 'shape/'
    points_dir = root_dir + 'points/'

    dir_list = [shape_dir, points_dir]
    for path in dir_list:
        if not os.path.exists(path):
            os.mkdir(path)

#1. shape_drain --> shape, label_map, id_map, shape_name
    Pool().map(generate_shape, [root_dir]*ARGS.num)

#2. shape, label_map, id_map --> point_cloud.py --> *.points, *.face_index, *.points_truth
    shape_paths = file_utils.file_paths_from_dir(shape_dir, '.step')
    Pool().map(generate_points, shape_paths)


