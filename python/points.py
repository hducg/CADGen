# -*- coding: utf-8 -*-
"""
Created on Wed May 22 12:15:37 2019

@author: 2624224
"""
import numpy as np
import os
import pickle

from OCC.Graphic3d import Graphic3d_ArrayOfPoints
from OCC.AIS import AIS_PointCloud
from OCC.Display.SimpleGui import init_display

import tables
import point_cloud
import shape

class LabeledPoints:
    def __init__(self):
        self.num_feats = 0
        self.face_index = []
    
    def convert(self, a_shape):
#        res = point_cloud.resolution_from_shape(a_shape.shape)
#        pricdnt('resolution', res)
        res = 10.0 / 64
        label_map = {face: a_shape.face_truth[a_shape.face_ids[face]] for face in a_shape.face_ids}
        self.points, _, _, self.labels, self.face_index = point_cloud.point_cloud_from_labeled_shape(a_shape.shape, label_map, a_shape.face_ids, res)
    
    def load(self, pts_path, pts_name):
        self.points = np.loadtxt(os.path.join(pts_path, pts_name + '.pts'))
        self.labels = np.loadtxt(os.path.join(pts_path, pts_name + '.seg')).astype('int64')
        self.num_feats = len(np.unique(self.labels))
    
    def save(self, pts_path, pts_name):        
        np.savetxt(os.path.join(pts_path, pts_name + '.pts'), self.points)
        
        if len(self.labels) > 0:
            np.savetxt(os.path.join(pts_path, pts_name + '.seg'), self.labels, fmt='%u')
            
        if len(self.face_index) > 0:
            face_index_path = os.path.join(pts_path, pts_name + '.face_index')
            with open(face_index_path, 'wb') as file:
                pickle.dump(self.face_index, file)
        
                
    def display(self, occ_display):
        occ_display.EraseAll()
    
        feats = {}
    
        if len(self.labels) > 0:
            for i in range(len(self.points)):
                feat_id = self.labels[i]
                if feat_id not in feats:
                    feats[feat_id] = [self.points[i]]
                else:
                    feats[feat_id].append(self.points[i])
        else:
            feats[0] = self.points
    
        for i in feats:
            feat = feats[i]
            print('feature #' + str(i) + ' points:', len(feat))
            n_points = len(feat)
            points_3d = Graphic3d_ArrayOfPoints(n_points)
            for pt in feat:
                points_3d.AddVertex(pt[0], pt[1], pt[2])
    
            point_cloud = AIS_PointCloud()
            point_cloud.SetPoints(points_3d)
            point_cloud.SetColor(tables.colors[i])
    
            occ_display.GetContext().Display(point_cloud)
        occ_display.View_Iso()
        occ_display.FitAll()                 
        

if __name__ == '__main__':
    occ_display, start_occ_display, add_menu, add_function_to_menu = init_display()
    rootdir = '../../dataset/machining_feature/'
#    the_shape = shape.LabeledShape()
#    shape_path = '../data/'
#    shape_name = '4-17-18-18-19-23-1'
#    the_shape.load(shape_path, shape_name)
    
    the_pts = LabeledPoints()
#    the_pts.convert(the_shape)
    pts_path = rootdir + 'points/'
    pts_name = '7-10-23-24'
    the_pts.load(pts_path, pts_name)
    the_pts.display(occ_display)
    start_occ_display()
    