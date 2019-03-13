# -*- coding: utf-8 -*-
"""
Created on Fri Feb  1 15:58:44 2019

@author: 2624224
"""

import os

from PIL import Image

file_dir = 'F:/wjcao/datasets/TestCAD/test_feature/'


file_names = os.listdir(file_dir)

for name in file_names:
    if name.endswith('.jpeg'):
        img_groundtruth = Image.open(file_dir + name.split('_')[0] + '_groundtruth.jpeg')
        img_predicted = Image.open(file_dir + name.split('_')[0] + '_predicted.jpeg')
        
        w, h = img_groundtruth.size
        img = Image.new(img_groundtruth.mode, (w * 2, h)) 
        img.paste(img_groundtruth, box=(0, 0))
        img.paste(img_predicted, box=(w, 0))       
        
        img_path = file_dir + name.split('_')[0] + '.jpeg'
        img.save(img_path)


