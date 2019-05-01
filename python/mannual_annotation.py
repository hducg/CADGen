# -*- coding: utf-8 -*-
"""
Created on Mon Apr 29 18:42:04 2019

@author: 2624224
"""

from OCC.Extend.DataExchange import read_step_file
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Display.SimpleGui import init_display
from OCC.Display.OCCViewer import rgb_color
from OCC.Core.BRepTools import breptools_Read
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.BRep import BRep_Builder


import occ_utils

import pickle
import os
import numpy as np

display, start_display, add_menu, add_function_to_menu = init_display()

colors = []
rgb_list = np.array(np.meshgrid([0.9, 0.6, 0.3], [0.9, 0.6, 0.3], [0.9, 0.6, 0.3])).T.reshape(-1,3)
for rgb in rgb_list:
    colors.append(rgb_color(rgb[0], rgb[1], rgb[2]))

FEAT_NAMES = ['through_hole', 'triangular_passage', 'rectangular_passage',
              '6sides_passage', 'triangular_through_slot', 'rectangular_through_slot',
              'circular_through_slot', 'rectangular_through_step',
              '2sides_through_step', 'slanted_through_step', # 10 through features
              'Oring', 'blind_hole', 'triangular_pocket', 'rectangular_pocket', 
              '6sides_pocket', 'circular_end_pocket', 'rectangular_blind_slot', 
              'v_circular_end_blind_slot', 'h_circular_end_blind_slot',         #6
              'triangular_blind_step', 'circular_blind_step', 'rectangular_blind_step', #3
              'chamfer', 'round', 'stock']
    
class LabeledShape:
    def __init__(self, shape_dir):        
        self.face_ais = {}
        self.face_id = {}
        self.label = 0
        print(FEAT_NAMES[self.label])

    def load_shape(self, filename):
        self.shape_name = filename.split('/')[-1].split('.')[0]
        print(self.shape_name)
        self.shape = TopoDS_Shape()
        builder = BRep_Builder()
        breptools_Read(self.shape, filename, builder)
        
        cnt = 0
        for face in TopologyExplorer(self.shape).faces():
            self.face_ais[face] = display.DisplayShape(face)           
            self.face_id[face] = cnt
            cnt += 1
        self.face_label = [-1] * cnt
        
    def annotate_face(self, face):
        print('face {} label {}'.format(self.face_id[face], self.label))
        self.face_label[self.face_id[face]] = self.label
        display.Context.SetColor(self.face_ais[face], colors[self.label])

    def to_file(self):
        occ_utils.shape_with_fid_to_step(self.shape_name + '.step', self.shape, self.face_id)
        
        with open(self.shape_name + '.face_truth', 'wb') as file:
            pickle.dump(self.face_label, file)
                

shape_dir = './shape/'
ashape = LabeledShape(shape_dir)


def annotate_face(faces, *kwargs):
    for face in faces:
        ashape.annotate_face(face)
    
    
def save_shape():
    ashape.to_file()
    
def next_feature():
    ashape.label += 1
    print(FEAT_NAMES[ashape.label])
    
if __name__ == '__main__':
    display.SetSelectionModeFace()
    display.register_select_callback(annotate_face)
    
    add_menu('menu')
    add_function_to_menu('menu', save_shape)
    add_function_to_menu('menu', next_feature)
    
#   1. load a step file
    filename = 'F:/wjcao/github/pythonocc-demos/assets/models/cylinder_head.brep'
    ashape.load_shape(filename)

    display.FitAll()
    start_display()
