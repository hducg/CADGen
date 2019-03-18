# -*- coding: utf-8 -*-
"""
Created on Sun Mar 10 16:48:08 2019

@author: 2624224
"""
from OCC.Display.SimpleGui import init_display
from OCC.Graphic3d import Graphic3d_ArrayOfPoints
from OCC.AIS import AIS_PointCloud

import generate_machining_dataset as gmd

import file_utils

occ_display, start_occ_display, add_menu, add_function_to_menu = init_display()
ais_context = occ_display.GetContext().GetObject()


def display_points(pts):
    occ_display.EraseAll()

    print('number of points:', len(pts))

    points_3d = Graphic3d_ArrayOfPoints(len(pts))
    for pt in pts:
        points_3d.AddVertex(pt[0], pt[1], pt[2])

    point_cloud = AIS_PointCloud()
    point_cloud.SetPoints(points_3d.GetHandle())

    ais_context.Display(point_cloud.GetHandle())
    occ_display.View_Iso()
    occ_display.FitAll()

    
if __name__ == '__main__':
#    pts = gmd.points_file_from_stl_path('F:/wjcao/datasets/Machining-feature-dataset-master/stl/0_Oring/0_1.STL')
    pts, normals, features, labels = file_utils.upgraded_point_cloud_from_file('F:/wjcao/datasets/Machining-feature-dataset-master/points/0_Oring/0_1.points')
    display_points(pts)
    start_occ_display()    