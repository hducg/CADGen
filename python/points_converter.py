# -*- coding: utf-8 -*-
"""
Created on Wed May 22 16:30:39 2019

@author: 2624224
"""
import os
import glob
from multiprocessing import Pool
import logging

import shape
import points

logging.basicConfig(level=logging.INFO, filename='points.log', filemode='w')

def generate_points(arg):
    '''
    generate points for shapes listed in CATEGORY_NAME_step.txt
    '''
    points_path = arg[0]
    shape_path = arg[1]
    shape_name = shape_path.split(os.sep)[-1].split('.')[0]
    shape_path = shape_path[:-len(shape_name)-5]
    
#    logging.info(shape_name)
    if os.path.exists(os.path.join(points_path, shape_name + '.pts')):
        return
    
    a_shape = shape.LabeledShape()
    a_shape.load(shape_path, shape_name)
    
    a_points = points.LabeledPoints()
    try:
        a_points.convert(a_shape)
    except:       
        logging.exception(shape_name)
        return
    
    a_points.save(points_path, shape_name)        
        
        
if __name__ == '__main__':
    rootdir = '../../dataset/machining_feature/'
    shape_dir = rootdir + 'shape/'
    points_dir = rootdir + 'points/'

    if not os.path.exists(points_dir):
        os.mkdir(points_dir)
        
    shape_paths = glob.glob(shape_dir + '*.step')    
    Pool().map(generate_points, [(points_dir, shape_path) for shape_path in shape_paths])
    