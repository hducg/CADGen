# -*- coding: utf-8 -*-
"""
Created on Fri Feb 22 16:24:02 2019

@author: 2624224
"""

import os
import argparse
from multiprocessing import Pool
from itertools import combinations_with_replacement

import shape


def generate_shape(arg):
    '''
    generate num_shapes random shapes in shape_dir
    '''
    shape_dir = arg[0]
    combo = arg[1]
    ashape = shape.LabeledShape()
    ashape.directive(combo)
    ashape.save(shape_dir)


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument('--rootdir',
                        '-r',
                        type=str,
                        help='root dir containning the shape dir, points dir, \
                        octree dir, lmdb dir, feature dir, and list dir',
                        required=True)
    ARGS = PARSER.parse_args()
    
    if not os.path.exists(ARGS.rootdir):
        os.mkdir(ARGS.rootdir)
        
    shape_dir = ARGS.rootdir + '/shape/'

    if not os.path.exists(shape_dir):
        os.mkdir(shape_dir)

    combos = []
    for num_combo in range(2, 7):
        combos += list(combinations_with_replacement(range(24), num_combo))

    Pool().map(generate_shape, [(shape_dir, combo) for combo in combos])

