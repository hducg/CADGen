# -*- coding: utf-8 -*-
"""
Created on Sun Mar 10 16:48:08 2019

@author: 2624224
"""
#import file_utils

import multiprocessing
import glob




def test_pool(stl_path):
    points_path = stl_path.split('.')[0].replace('stl', 'points') + '.points'
    return points_path

    
if __name__ == '__main__':
    stl_list = glob.glob('F:/wjcao/datasets/Machining-feature-dataset-master/stl/*/*')
    print(stl_list[:8])
    result = multiprocessing.Pool().map(test_pool, stl_list[:8])
    print(result)
    