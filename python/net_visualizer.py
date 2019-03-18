# -*- coding: utf-8 -*-
"""
Created on Fri Mar 15 11:29:04 2019

@author: 2624224
"""
import math

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
    box_list, center_list = generate_filter_shape()
    OCC_DISPLAY, START_OCC_DISPLAY, ADD_MENU, _ = OCC.Display.SimpleGui.init_display()
    OCC_DISPLAY.EraseAll()

    for shape in box_list:
        ais = OCC.AIS.AIS_ColoredShape(shape)
        ais.SetDisplayMode(OCC.AIS.AIS_WireFrame)
        OCC_DISPLAY.Context.Display(ais.GetHandle())

    normals = [[0.00991668, 0.02121447, -0.01957863], [0.01145461, 0.02974107, -0.00290425],
               [0.02170685, 0.02844858, -0.02558133], [0.02131673, 0.04355109, -0.01335364],
               [0.01226219, 0.01813491, 0.00613484], [0.04510777, 0.05335061, -0.06061369],
               [0.05890357, -0.02430928, 0.02942025], [0.0491891, -0.02333533, 0.00958339],
               [0.03981413, -0.06421408, -0.01231594], [0.00208728, 0.00049695, 0.02604783],
               [-0.03145519, -0.02269633, -0.00328037], [0.00815452, 0.0010067, 0.01621923],
               [-0.01266365, -0.02410725, -0.03711325], [-0.19958942, -0.02344204, -0.0099872],
               [-0.08614887, -0.01160532, 0.09720282], [-0.01106545, -0.05320733, 0.00355071],
               [-0.04177443, 0.04718323, 0.00197159], [0.00998389, -0.01981858, 0.00731814],
               [0.01735839, 0.03347654, 0.01181057], [0.03830191, -0.03223777, -0.0127057],
               [0.01040506, -0.01836227, 0.01899686], [0.02035885, -0.0136759, -0.02745979],
               [0.1468002, -0.02026927, -0.05697852], [0.09266672, 0.00851517, 0.1102615,],
               [0.00101798, -0.05933225, -0.0032459], [0.0179624, 0.02979583, 0.00779],
               [0.01518726, 0.03383517, 0.0312128]]

    for i in range(27):
        pt = center_list[i]
        direction = OCC.gp.gp_Vec(normals[i][0], normals[i][1], normals[i][2])
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
    