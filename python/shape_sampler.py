# -*- coding: utf-8 -*-
"""
Created on Fri Feb 22 16:24:02 2019

@author: 2624224
"""
import pickle
import os
import argparse

import file_utils
import shape_factory
import point_cloud

class ShapeSampler:
    '''
    sample shapes randomly
    '''
    def __init__(self, root_dir, num_shapes):
        '''
        initialize class constants
        create dirs
        '''
        self.root_dir = root_dir + '/'
        self.num_shapes = num_shapes
        print(self.num_shapes, 'shapes')

        self.shape_dir = self.root_dir + 'shape/'
        self.points_dir = self.root_dir + 'points/'

        dir_list = [self.shape_dir, self.points_dir]
        for path in dir_list:
            if not os.path.exists(path):
                os.mkdir(path)


    def generate_shapes(self):
        '''
        generate num_shapes random shapes in shape_dir
        '''
        for _ in range(self.num_shapes):
            shape, label_map, id_map, shape_name = shape_factory.shape_drain()

            step_path = self.shape_dir + shape_name + '.step'
            if not os.path.exists(step_path):
                file_utils.shape_with_fid_to_step(step_path, shape, id_map)

                face_truth_path = self.shape_dir + shape_name + '.face_truth'
                face_truth = [label_map[face] for face, fid in
                              sorted(id_map.items(), key=lambda kv: kv[1])]
                with open(face_truth_path, 'wb') as file:
                    pickle.dump(face_truth, file)
        shape_paths = [shape_path + '\n' for shape_path in file_utils.file_paths_from_dir(self.shape_dir, '.step')]
        file_path = self.root_dir + 'shape_list.txt'
        with open(file_path, 'w') as file:
            file.writelines(shape_paths)


    def generate_points(self):
        '''
        generate points for shapes listed in CATEGORY_NAME_step.txt
        '''
        shape_list_dir = self.root_dir + 'shape_list.txt'
        with open(shape_list_dir) as file:
            shape_paths = [line.strip() for line in file.readlines()]

        for shape_path in shape_paths:
            shape_name = shape_path.split('/')[-1].split('.')[0]
            file_path = self.points_dir + shape_name + '.points'
            if os.path.exists(file_path):
                continue

            shape, id_map = file_utils.shape_with_fid_from_step(shape_path)
            face_truth_path = self.shape_dir + shape_name + '.face_truth'
            with open(face_truth_path, 'rb') as file:
                face_truth = pickle.load(file)
            label_map = {face: face_truth[id_map[face]] for face in id_map}

            res = point_cloud.resolution_from_shape(shape)
            print('resolution', res)
            pts, normals, feats, segs, face_ids = point_cloud.point_cloud_from_labeled_shape(shape, label_map, id_map, res)
            file_utils.upgraded_point_cloud_to_file(file_path, pts, normals, feats, segs)

            face_index_path = self.points_dir + shape_name + '.face_index'
            with open(face_index_path, 'wb') as file:
                pickle.dump(face_ids, file)

        points_paths = [points_path + '\n' for points_path in file_utils.file_paths_from_dir(self.points_dir, '.points')]
        file_path = self.root_dir + 'points_list.txt'
        with open(file_path, 'w') as file:
            file.writelines(points_paths)


if __name__ == '__main__':
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
    SHAPE_SAMPLER = ShapeSampler(ARGS.rootdir, ARGS.num)

#1. shape_drain --> shape, label_map, id_map, shape_name
    SHAPE_SAMPLER.generate_shapes()

#2. shape, label_map, id_map --> point_cloud.py --> *.points, *.face_index, *.points_truth
    SHAPE_SAMPLER.generate_points()


