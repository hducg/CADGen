# -*- coding: utf-8 -*-
"""
Created on Wed May 22 16:30:39 2019

@author: 2624224
"""
import os
import argparse
import glob
from multiprocessing import Pool

import shape
import points

def generate_points(arg):
    '''
    generate points for shapes listed in CATEGORY_NAME_step.txt
    '''
    points_path = arg[0]
    shape_path = arg[1]
    shape_name = shape_path.split('/')[-1].split('.')[0]
    shape_path = shape_path.split('/')[0]

    a_shape = shape.LabeledShape()
    a_shape.load(shape_path, shape_name)
    
    a_points = points.LabeledPoints()
    a_points.convert(a_shape)
    a_points.save(points_path, shape_name)    
        
        
if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument('--rootdir',
                        '-r',
                        type=str,
                        help='root dir containning the shape dir, points dir, \
                        octree dir, lmdb dir, feature dir, and list dir',
                        required=True)
    ARGS = PARSER.parse_args()

    shape_dir = os.path.join(ARGS.rootdir, '/shape/')
    points_dir = os.path.join(ARGS.rootdir, '/points/')
    
    if not os.path.exists(points_dir):
        os.mkdir(points_dir)
        
    shape_paths = glob.glob(shape_dir + '*.step')
    
    Pool().map(generate_points, [(points_dir, shape_path) for shape_path in shape_paths])
    