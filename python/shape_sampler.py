# -*- coding: utf-8 -*-
"""
Created on Fri Feb 22 16:24:02 2019

@author: 2624224
"""

import os
from multiprocessing import Pool
from itertools import combinations_with_replacement
import logging

import shape

logging.basicConfig(level=logging.INFO, filename='feature.log')

def generate_shape(arg):
    '''
    generate num_shapes random shapes in shape_dir
    '''
    shape_dir = arg[0]
    combo = arg[1]    
    ashape = shape.LabeledShape()
    ashape.directive(combo)
    logging.info(ashape.shape_name + ' done')
    ashape.save(shape_dir)


if __name__ == '__main__':
    rootdir = '../../dataset/machining_feature/'
    if not os.path.exists(rootdir):
        os.mkdir(rootdir)
        
    shape_dir = rootdir + '/shape/'

    if not os.path.exists(shape_dir):
        os.mkdir(shape_dir)

    combos = []
    for num_combo in range(2, 7):
        combos += list(combinations_with_replacement(range(24), num_combo))
    print(len(combos), 'models')
    Pool().map(generate_shape, [(shape_dir, combo) for combo in combos[1024:2048]])

