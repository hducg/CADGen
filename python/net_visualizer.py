# -*- coding: utf-8 -*-
"""
Created on Fri Mar 15 11:29:04 2019

@author: 2624224
"""
import argparse
import math
import pickle
import numpy as np

import OCC.BRepPrimAPI
import OCC.BRepBuilderAPI
import OCC.gp
import OCC.Display.SimpleGui
import OCC.AIS
import OCC.Geom
import OCC.Quantity
import OCC.GC


def generate_filter_shape():
    shape_list = []
    center_list = []
    for x in range(3):
        for y in range(3):
            for z in range(3):
                pt1 = OCC.gp.gp_Pnt(x, y, z)
                pt2 = OCC.gp.gp_Pnt(x + 1, y + 1, z + 1)
                shape_list.append(OCC.BRepPrimAPI.BRepPrimAPI_MakeBox(pt1, pt2).Solid())
                pt = OCC.gp.gp_Pnt(x + 0.5, y + 0.5, z + 0.5)
                center_list.append(pt)

    return shape_list, center_list


if __name__ == '__main__':
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument('--filterpath',
                        '-f',
                        type=str,
                        required=True)
    ARGS = PARSER.parse_args()
    
    box_list, center_list = generate_filter_shape()
    OCC_DISPLAY, START_OCC_DISPLAY, ADD_MENU, _ = OCC.Display.SimpleGui.init_display()
    OCC_DISPLAY.EraseAll()

    for shape in box_list:
        ais = OCC.AIS.AIS_ColoredShape(shape)
        ais.SetDisplayMode(OCC.AIS.AIS_WireFrame)
        OCC_DISPLAY.Context.Display(ais.GetHandle())

    with open(ARGS.filterpath, 'rb') as file:
        filters = pickle.load(file)
    print(filters.shape)
    print(filters[7])
    feat = filters[7].reshape(27,3)

    for i in range(27):
        pt = center_list[i]
        direction = OCC.gp.gp_Vec(float(feat[i][0]), float(feat[i][1]), float(feat[i][2]))
        endp = OCC.gp.gp_Pnt(pt.XYZ())
        endp.Translate(direction)
        edge = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(OCC.GC.GC_MakeSegment(pt, endp).Value()).Edge()
        OCC_DISPLAY.DisplayShape(edge)

        circ = OCC.gp.gp_Circ(OCC.gp.gp_Ax2(pt, OCC.gp.gp_Dir(direction)), direction.Magnitude())
        edge = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circ, 0., 2*math.pi).Edge()
        wire = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire(edge).Wire()
        face = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeFace(wire).Face()
        OCC_DISPLAY.DisplayShape(face)

    OCC_DISPLAY.View_Iso()
    OCC_DISPLAY.FitAll()

    START_OCC_DISPLAY()
    