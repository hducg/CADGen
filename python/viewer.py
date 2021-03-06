# -*- coding: utf-8 -*-
"""
Created on Fri Jan 18 17:05:26 2019

@author: 2624224
"""
import os
import pickle
import argparse
import glob
import numpy as np

from OCC.AIS import AIS_PointCloud, AIS_ColoredShape
from OCC.Graphic3d import Graphic3d_ArrayOfPoints
from OCC.Display.OCCViewer import rgb_color
from OCC.Display.SimpleGui import init_display
from OCC.Core.gp import gp_Circ, gp_Ax2, gp_Pnt, gp_Dir, gp_Vec
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace

import file_utils
import occ_utils


colors = [rgb_color(0,0,0), rgb_color(0.75,0.75,0.75), rgb_color(1,0,0), rgb_color(1,0.5,0),
          rgb_color(0,1,1), rgb_color(1,0,1), rgb_color(1,0.8,0.8), rgb_color(1,1,0.8),
          rgb_color(0.8,1,1), rgb_color(1,0.8,1), rgb_color(0.4,0,0), rgb_color(0.4,0.4,0),
          rgb_color(0,0.4,0.4), rgb_color(0.4,0,0.4)]


class SegmentsViewer:
    def __init__(self, pts, occ_display):
        for i in range(len(pts) - 1):
            occ_display.DisplayShape(BRepBuilderAPI_MakeEdge(occ_utils.as_occ(pts[i], gp_Pnt), occ_utils.as_occ(pts[i + 1], gp_Pnt)).Edge())
            
class PointsViewer:
    def __init__(self, points, occ_display):
        n_points = len(points)
        points_3d = Graphic3d_ArrayOfPoints(n_points)
        for pt in points:
            points_3d.AddVertex(pt[0], pt[1], pt[2])

        point_cloud = AIS_PointCloud()
        point_cloud.SetPoints(points_3d)
        ais_context = occ_display.GetContext()                
        ais_context.Display(point_cloud)
        
        
class ShapeViewer:
    def __init__(self, shape_path, occ_display):
        self.colors = []
        rgb_list = np.array(np.meshgrid([0.9, 0.6, 0.3], [0.9, 0.6, 0.3], [0.9, 0.6, 0.3])).T.reshape(-1,3)
        for rgb in rgb_list:
            self.colors.append(rgb_color(rgb[0], rgb[1], rgb[2]))
            
        self.shape_path = shape_path
        self.names = [name.split(os.sep)[-1].split('.')[0] for name in glob.glob(shape_path + '*.step')]                
        self.shape_cnt = -1#random.randint(0, len(self.names))
        self.occ_display = occ_display
                
    def display_shape(self):
        self.occ_display.EraseAll()
        AIS = AIS_ColoredShape(self.shape)
        for a_face in self.label_map:
            AIS.SetCustomColor(a_face, self.colors[self.label_map[a_face]])
    
        self.occ_display.Context.Display(AIS)
        self.occ_display.View_Iso()
        self.occ_display.FitAll()
        
    def load_shape(self, shape, label_map):
        self.shape = shape
        self.label_map = label_map
        
    def read_shape(self):
        filename = os.path.join(self.shape_path, self.names[self.shape_cnt] + '.step')
        self.shape, face_ids = occ_utils.shape_with_fid_from_step(filename)
        
        filename = os.path.join(self.shape_path, self.names[self.shape_cnt] + '.face_truth')
        with open(filename, 'rb') as file:
            face_truth = pickle.load(file)
        self.label_map = {f:face_truth[face_ids[f]] for f in face_ids}
        
    def next_sample(self):
        self.shape_cnt = (self.shape_cnt + 1)%len(self.names)#random.randint(0, len(self.names))
        print(self.names[self.shape_cnt])
        self.read_shape()
        self.display_shape()
                       
    def next_batch(self):
        self.shape_cnt = (self.shape_cnt + 24)%len(self.names)#random.randint(0, len(self.names))
        print(self.names[self.shape_cnt])
        self.read_shape()
        self.display_shape()
        
    def save_image(self):
        image_name = os.path.join(self.shape_path, str(self.shape_cnt) + '.jpeg')
        self.occ_display.View.Dump(image_name)  
        
        
class SegShapeViewer:
    '''
    input
        octree_list_file_name:
            content example: test_octree/hole-1(1[triangle2])3(2[sweep]1[circle])2(2[sweep])4(1[circle]2[sweep]1[rectangle]).upgrade_6_2_000.octree
        dataset_dir: direcotry where directories of points, octrees and labels are
            example: 'F:/wjcao/datasets/TestCAD'
            points directory example: test_upgraded, octree directory example: test_octree
    '''
    def __init__(self, root_path, category_name):
        self.shape_points_mode = 'shape'
        self.truth_predicted_mode = 'truth'
        self.dataset_dir = root_path
        self.points_category = category_name
        self.current_index = -1

        list_path = root_path + '/' + category_name + '_list/octree_list.txt'
        with open(list_path, 'r') as file:
            lines = file.readlines()

        # list of points names, example: hole-1(1[triangle2])3(2[sweep]1[circle])2(2[sweep])4(1[circle]2[sweep]1[rectangle]).upgrade
        self.shape_names = [line.split('_')[0] for line in lines]
        # list of points category names, same length as shape_names, example: test


    def load_shape(self):

        # load shape and face_index
        shape_path = self.dataset_dir + '/shape/' + self.shape_names[self.current_index] + '.step'
        self.shape, id_map = occ_utils.shape_with_fid_from_step(shape_path)
        face_truth_path = self.dataset_dir + '/shape/' + self.shape_names[self.current_index] + '.face_truth'
        with open(face_truth_path, 'rb') as f:
            label_map = pickle.load(f)
        self.face_truth = {f:label_map[id_map[f]] for f in id_map}

        face_predicted_path = self.dataset_dir + '/shape/' + self.shape_names[self.current_index] + '.face_predicted'
        if os.path.exists(face_predicted_path):
            with open(face_predicted_path, 'rb') as f:
                label_map = pickle.load(f)
            self.face_predicted = {f:label_map[id_map[f]] for f in id_map}

        # load point cloud
        points_path = self.dataset_dir + '/points/' + self.shape_names[self.current_index] + '.points'
        self.points,normals,features,self.points_truth = file_utils.upgraded_point_cloud_from_file(points_path)

        points_predicted_path = self.dataset_dir + '/points/' + self.shape_names[self.current_index] + '.points_predicted'
        if os.path.exists(points_predicted_path):
            with open(points_predicted_path, 'rb') as f:
                self.points_predicted = pickle.load(f)


    def next_shape(self):
        self.current_index += 1
        if self.current_index >= len(self.shape_names):
            self.current_index = 0

        print(self.shape_names[self.current_index])
        self.load_shape()
        self.display()


    def display(self):
        if self.shape_points_mode == 'shape':
            if self.truth_predicted_mode == 'truth':
                display_shape(self.shape, self.face_truth)
                self.label_suffix = '_shape_groundtruth'
            else:
                display_shape(self.shape, self.face_predicted)
                self.label_suffix = '_shape_predicted'
        else:
            if self.truth_predicted_mode == 'truth':
                display_points(self.points, self.points_truth)
                self.label_suffix = '_points_groundtruth'
            else:
                display_points(self.points, self.points_predicted)
                self.label_suffix = '_points_predicted'


    def save_image(self):
        image_name = self.dataset_dir + '/' + self.points_category + '_feature/' + self.shape_names[self.current_index] + self.label_suffix + '.jpeg'
        occ_display.View.Dump(image_name)


def display_shape(shape, fmap):
    occ_display.EraseAll()
    ais = AIS_ColoredShape(shape)
    for f in fmap:
        ais.SetCustomColor(f, colors[fmap[f]])

    occ_display.Context.Display(ais.GetHandle())
#    occ_display.View_Iso()
    occ_display.FitAll()


def display_points(pts, labels):
    occ_display.EraseAll()

    print('number of points:', len(pts))
    feats = [None] * 14

    if len(labels) > 0:
        for i in range(len(pts)):
            feat_id = labels[i]
            if feats[feat_id] is None:
                feats[feat_id] = [pts[i]]
            else:
                feats[feat_id].append(pts[i])
    else:
        feats[0] = pts

    for i in range(len(feats)):
        feat = feats[i]
        if feat is None:
            continue
        print('number of feature ' + str(i) + ' points:', len(feat))
        n_points = len(feat)
        points_3d = Graphic3d_ArrayOfPoints(n_points)
        for pt in feat:
            points_3d.AddVertex(pt[0], pt[1], pt[2])

        point_cloud = AIS_PointCloud()
        point_cloud.SetPoints(points_3d)
        point_cloud.SetColor(colors[i])

        ais_context.Display(point_cloud)
#    occ_display.View_Iso()
    occ_display.FitAll()




#category_names = ['03001627','04099429','03790512','03797390']
#dataset_dir = 'F:/wjcao/datasets/ocnn'
#labels_subdir = category_names[0] + '_feature'
#octree_list_file_name = dataset_dir + category_names[0] + '_octree_names_shuffle.txt'


#def next_shape():
#    points_render.next_shape()


def save_all():
    for i in range(min(len(points_render.shape_names),50)):
        points_render.next_shape()
        save_image()
        points_render.display_predicted()
        save_image()


def switch_shape_points():
    if points_render.shape_points_mode == 'shape':
        points_render.shape_points_mode = 'points'
        points_render.display()
    else:
        points_render.shape_points_mode = 'shape'
        points_render.display()


def switch_truth_predicted():
    if points_render.truth_predicted_mode == 'predicted':
        points_render.truth_predicted_mode = 'truth'
        points_render.display()
    else:
        points_render.truth_predicted_mode = 'predicted'
        points_render.display()

def next_shape():
    asampler.next_sample()
       
def next_batch():
    asampler.next_batch()
    
    
if __name__ == '__main__':
    '''
    input:
        rootdir, dir containing shape dir, points dir, and batchname_list dir, 
        all must exist
        
        shape dir contains *.step file, *.face_truth file, *.face_predicted file 
        for each shape
        
        points dir contains *.points file, *.face_index file, *.points_predicted 
        file for each shape
        
        batchname_list dir contains octree_list.txt, from which names of shapes to 
        be rendered are extracted. 
        each shape has a unique name, and is associated with a step file, 
        a face_truth file, a points file, a face_index file, a points_predicted 
        file,all with the same name but different extensions

        batchname, which test result to render, used to find batchname_list dir
    '''
    occ_display, start_occ_display, add_menu, add_function_to_menu = init_display()
    ais_context = occ_display.GetContext()
    shape_path = '../../models/'
    
#    global points_render
#    points_render = SegShapeViewer(shape_path, ARGS.batchname)
    
    global asampler        
    asampler = ShapeViewer(shape_path, occ_display)   

    def save_image():
        image_name = shape_path + '/image.jpeg'
        occ_display.View.Dump(image_name)    
        
    add_menu('Display')
    add_function_to_menu('Display', switch_shape_points)
    add_function_to_menu('Display', switch_truth_predicted)
    add_function_to_menu('Display', save_image)
    add_function_to_menu('Display', save_all)
    add_menu('shape')
    add_function_to_menu('shape', next_shape)
    add_function_to_menu('shape', next_batch)
    add_function_to_menu('shape', save_image)
    start_occ_display()
