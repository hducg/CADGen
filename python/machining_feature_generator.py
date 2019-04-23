# _*_ coding: utf_8 _*_
"""
Created on Tue Mar 19 11:20:39 2019

@author: 2624224
"""

import sys
import random
import math
import logging

import numpy as np

from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeVertex, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace
from OCC.BRepFeat import BRepFeat_MakePrism
from OCC.Display.SimpleGui import init_display
from OCC.Display.OCCViewer import rgb_color
from OCC.AIS import AIS_ColoredShape
from OCC.gp import gp_Circ, gp_Ax2, gp_Pnt, gp_Dir
from OCC.TopoDS import TopoDS_Face
from OCC.BRep import BRep_Tool
from OCC.TopLoc import TopLoc_Location
from OCC.GC import GC_MakeSegment, GC_MakeArcOfCircle
from OCC.Geom import Geom_Circle
from OCC.BRepMesh import BRepMesh_IncrementalMesh
from OCC.TopoDS import TopoDS_Vertex
from OCC.TopAbs import TopAbs_VERTEX, TopAbs_REVERSED
from OCC.TopExp import TopExp_Explorer, topexp
from OCC.BRepFilletAPI import BRepFilletAPI_MakeFillet


import occ_utils
import shape_factory
import geom_utils

import OCCUtils.edge
import OCCUtils.face

def triangulation_from_face(face):
    aLoc = TopLoc_Location()
    aTriangulation = BRep_Tool().Triangulation(face, aLoc).GetObject()
    aTrsf = aLoc.Transformation()

    aNodes = aTriangulation.Nodes()
    aTriangles = aTriangulation.Triangles()

    pts = []
    for i in range(1, aTriangulation.NbNodes() + 1):
        pt = aNodes.Value(i)
        pt.Transform(aTrsf)
        pts.append([pt.X(),pt.Y(),pt.Z()])

    triangles = []
    vt_map = {}
    et_map = {}
    for i in range(1, aTriangulation.NbTriangles() + 1):
        n1, n2, n3 = aTriangles.Value(i).Get()
        pids = [n1 - 1, n2 - 1, n3 - 1]
        pids.sort()
        triangles.append((pids[0], pids[1], pids[2]))

        for pid in pids:
            if pid in vt_map:
                vt_map[pid].append(i -1)
            else:
                vt_map[pid] = [i-1]

        edges = [(pids[0], pids[1]), (pids[0], pids[2]), (pids[1], pids[2])]
        for edge in edges:
            if edge in et_map:
                et_map[edge].append(i - 1)
            else:
                et_map[edge] = [i - 1]

    return pts, triangles, vt_map, et_map

def triangles_from_faces(faces):
    tri_list = []
    for face in faces:
        pts, triangles, vt_map, et_map = triangulation_from_face(face)
        for tri in triangles:
            tri_list.append((pts[tri[0]], pts[tri[1]], pts[tri[2]]))

    return tri_list


#==============================================================================
# sketch face generator    
#============================================================================== 
    

def face_circle(ref_pnts):
    dir_w = ref_pnts[2] - ref_pnts[1]
    dir_h = ref_pnts[0] - ref_pnts[1]    
    width = np.linalg.norm(dir_w)
    height = np.linalg.norm(dir_h)
    
    try:
        assert width > 2.0 and height > 2.0, 'width or height too small'
    except AssertionError as error:
        logging.exception('error caught')
        print(width, height)
        return None
    
    dir_w = dir_w / width
    dir_h = dir_h / height
    normal = np.cross(dir_w, dir_h)
    
    radius = random.uniform(1.0, min(width / 2, height / 2))
    
    offset_w = random.uniform(radius, width - radius)
    offset_h = random.uniform(radius, height - radius)
    center = ref_pnts[1] + dir_w * offset_w + dir_h * offset_h

    circ = gp_Circ(gp_Ax2(gp_Pnt(center[0], center[1], center[2]), occ_utils.as_occ(normal, gp_Dir)), radius)
    edge = BRepBuilderAPI_MakeEdge(circ, 0., 2*math.pi).Edge()
    outer_wire = BRepBuilderAPI_MakeWire(edge).Wire()

    face_maker = BRepBuilderAPI_MakeFace(outer_wire)

    return face_maker.Face()

    
def face_circle_1(ref_pnts):
    # sample radius
    edge_dir = ref_pnts[2] - ref_pnts[1]
    width = np.linalg.norm(edge_dir)
    edge_dir = edge_dir / width
    height = np.linalg.norm(ref_pnts[0] - ref_pnts[1])

    try:
        assert width > 2.0 and height > 1.0, 'width or height too small'
    except AssertionError as error:
        print('face_circle_1', error)
        print(width, height)
        return None
        
    radius = random.uniform(1.0, min(width / 2, height))    
    arc_offset = random.uniform(0.0, width - 2 * radius)
    pnt1 = ref_pnts[1] + edge_dir * arc_offset
    pnt2 = pnt1 + edge_dir * 2 * radius
    center = pnt1 + edge_dir * radius
    
    pnt1 = occ_utils.as_occ(pnt1, gp_Pnt)
    pnt2 = occ_utils.as_occ(pnt2, gp_Pnt)
    center = occ_utils.as_occ(center, gp_Pnt)
    
    edge1 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pnt1, pnt2).Value()).Edge()

    normal = occ_utils.as_occ(np.cross(edge_dir, ref_pnts[0] - ref_pnts[1]), gp_Dir)
    circ = gp_Circ(gp_Ax2(center, normal), radius)
    edge2 = BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(circ, pnt2, pnt1, True).Value()).Edge()

    wire_maker = BRepBuilderAPI_MakeWire(edge1, edge2)
    face_maker = BRepBuilderAPI_MakeFace(wire_maker.Wire())    
    return face_maker.Face()

    
def face_circle_2(ref_pnts):
    vec0 = ref_pnts[0] - ref_pnts[1]
    vec2 = ref_pnts[2] - ref_pnts[1]
    width = np.linalg.norm(vec2)
    height = np.linalg.norm(vec0)

    try:
        assert width > 1.0 and height > 1.0, 'width or height too small'
    except AssertionError as error:
        print('face_circle_2', error)
        print(width, height)
        return None
        
    radius = random.uniform(0.5, 0.5 * min(width, height))
    vec0 = vec0 / np.linalg.norm(vec0)
    vec2 = vec2 / np.linalg.norm(vec2)
    
    pt0 = occ_utils.as_occ(ref_pnts[1] + vec0 * radius, gp_Pnt)
    pt1 = occ_utils.as_occ(ref_pnts[1], gp_Pnt)
    pt2 = occ_utils.as_occ(ref_pnts[1] + vec2 * radius, gp_Pnt)
    
    normal = occ_utils.as_occ(np.cross(vec2, vec0), gp_Dir)
    cir = gp_Circ(gp_Ax2(pt1, normal), radius)    
    seg_maker = [GC_MakeSegment(pt0, pt1), GC_MakeSegment(pt1, pt2), GC_MakeArcOfCircle(cir, pt2, pt0, True)]
    wire_maker = BRepBuilderAPI_MakeWire()
    for sm in seg_maker:
        if sm.IsDone:
            edge = BRepBuilderAPI_MakeEdge(sm.Value()).Edge()
            wire_maker.Add(edge)
        else:
            return None
            
    face_maker = BRepBuilderAPI_MakeFace(wire_maker.Wire())

    return face_maker.Face()

    
def face_circular_end_rect(ref_pnts):
    
    return None

    
def face_open_circular_end_rect_v(ref_pnts):
    dir_w = ref_pnts[2] - ref_pnts[1]
    dir_h = ref_pnts[0] - ref_pnts[1]
    width = np.linalg.norm(dir_w)
    height = np.linalg.norm(dir_h)

    try:
        assert width > 1.0 and height > 1.0, 'width or height too small'
    except AssertionError as error:
        print('face_circle_2', error)
        print(width, height)
        return None

    dir_w = dir_w / width
    dir_h = dir_h / height
    normal = np.cross(dir_w, dir_h)

    rect_w = random.uniform(1.0, min(width, height))        
    rect_h = random.uniform(rect_w / 2 + 0.5, height)
    offset = random.uniform(0.0, width - rect_w)
    pt1 = ref_pnts[1] + dir_w * offset
    pt2 = pt1 + dir_w * rect_w
    pt3 = pt2 + dir_h * (rect_h - rect_w / 2)
    pt4 = pt1 + dir_h * (rect_h - rect_w / 2)
    
    center = pt4 + dir_w * rect_w / 2
    circ = gp_Circ(gp_Ax2(gp_Pnt(center[0], center[1], center[2]), occ_utils.as_occ(normal, gp_Dir)), rect_w / 2)
    pt1 = occ_utils.as_occ(pt1, gp_Pnt)
    pt2 = occ_utils.as_occ(pt2, gp_Pnt)
    pt3 = occ_utils.as_occ(pt3, gp_Pnt)
    pt4 = occ_utils.as_occ(pt4, gp_Pnt)    
    seg_maker = [GC_MakeSegment(pt1, pt2), GC_MakeSegment(pt2, pt3), GC_MakeArcOfCircle(circ, pt3, pt4, True), GC_MakeSegment(pt4, pt1)]
    wire_maker = BRepBuilderAPI_MakeWire()
    for sm in seg_maker:
        edge = BRepBuilderAPI_MakeEdge(sm.Value()).Edge()
        wire_maker.Add(edge)
            
    face_maker = BRepBuilderAPI_MakeFace(wire_maker.Wire())
    
    return face_maker.Face()

    
def face_open_circular_end_rect_h(ref_pnts):
    dir_w = ref_pnts[2] - ref_pnts[1]
    dir_h = ref_pnts[0] - ref_pnts[1]
    width = np.linalg.norm(dir_w)
    height = np.linalg.norm(dir_h)

    try:
        assert width > 2.0 and height > 0.5, 'width or height too small'
    except AssertionError as error:
        print('face_circle_2', error)
        print(width, height)
        return None

    dir_w = dir_w / width
    dir_h = dir_h / height
    normal = np.cross(dir_w, dir_h)

    rect_h = random.uniform(0.5, min(height, width / 2))    
    rect_w = random.uniform(2 * rect_h + 1.0, width)        
    offset = random.uniform(0.0, width - rect_w)
    pt1 = ref_pnts[1] + dir_w * offset
    pt2 = pt1 + dir_w * rect_w
    pt3 = pt2 - dir_w * rect_h + dir_h * rect_h
    pt4 = pt1 + dir_w * rect_h + dir_h * rect_h
    
    center1 = pt1 + dir_w * rect_h
    center2 = pt2 - dir_w * rect_h
    circ1 = gp_Circ(gp_Ax2(occ_utils.as_occ(center1, gp_Pnt), occ_utils.as_occ(normal, gp_Dir)), rect_h)
    circ2 = gp_Circ(gp_Ax2(occ_utils.as_occ(center2, gp_Pnt), occ_utils.as_occ(normal, gp_Dir)), rect_h)
    pt1 = occ_utils.as_occ(pt1, gp_Pnt)
    pt2 = occ_utils.as_occ(pt2, gp_Pnt)
    pt3 = occ_utils.as_occ(pt3, gp_Pnt)
    pt4 = occ_utils.as_occ(pt4, gp_Pnt)
    
    seg_maker = [GC_MakeSegment(pt1, pt2), GC_MakeArcOfCircle(circ2, pt2, pt3, True), GC_MakeSegment(pt3, pt4), GC_MakeArcOfCircle(circ1, pt4, pt1, True)]
    wire_maker = BRepBuilderAPI_MakeWire()
    for sm in seg_maker:
        edge = BRepBuilderAPI_MakeEdge(sm.Value()).Edge()
        wire_maker.Add(edge)
            
    face_maker = BRepBuilderAPI_MakeFace(wire_maker.Wire())
    
    return face_maker.Face()

    
def face_hexagon(ref_pnts):
    dir_w = ref_pnts[2] - ref_pnts[1]
    dir_h = ref_pnts[0] - ref_pnts[1]    
    width = np.linalg.norm(dir_w)
    height = np.linalg.norm(dir_h)
    
    try:
        assert width > 2.0 and height > 2.0, 'width or height too small'
    except AssertionError as error:
        print('face_hexagon', error)
        print(width, height)
        return None
    
    dir_w = dir_w / width
    dir_h = dir_h / height
    normal = occ_utils.as_occ(np.cross(dir_w, dir_h), gp_Dir)

    radius = random.uniform(1.0, min(width / 2, height / 2))
    
    offset_w = random.uniform(radius, width - radius)
    offset_h = random.uniform(radius, height - radius)
    center = ref_pnts[1] + dir_w * offset_w + dir_h * offset_h
    
    circ = Geom_Circle(gp_Ax2(gp_Pnt(center[0], center[1], center[2]), normal), radius)

    ang1 = random.uniform(0.0, math.pi / 3)
    pt1 = occ_utils.as_list(circ.Value(ang1))

    ang2 = ang1 + math.pi / 3
    pt2 = occ_utils.as_list(circ.Value(ang2))

    ang3 = ang2 + math.pi / 3
    pt3 = occ_utils.as_list(circ.Value(ang3))

    ang4 = ang3 + math.pi / 3
    pt4 = occ_utils.as_list(circ.Value(ang4))

    ang5 = ang4 + math.pi / 3
    pt5 = occ_utils.as_list(circ.Value(ang5))

    ang6 = ang5 + math.pi / 3
    pt6 = occ_utils.as_list(circ.Value(ang6))

    return occ_utils.face_polygon([pt1, pt2, pt3, pt4, pt5, pt6])

    
def face_oring(ref_pnts):
    dir_w = ref_pnts[2] - ref_pnts[1]
    dir_h = ref_pnts[0] - ref_pnts[1]    
    width = np.linalg.norm(dir_w)
    height = np.linalg.norm(dir_h)
    
    try:
        assert width > 2.0 and height > 2.0, 'width or height too small'
    except AssertionError as error:
        print('face_oring', error)
        print(width, height)
        return None
    
    dir_w = dir_w / width
    dir_h = dir_h / height
    normal = occ_utils.as_occ(np.cross(dir_w, dir_h), gp_Dir)

    outer_r = random.uniform(1.0, min(width / 2, height / 2))
    
    offset_w = random.uniform(outer_r, width - outer_r)
    offset_h = random.uniform(outer_r, height - outer_r)
    center = ref_pnts[1] + dir_w * offset_w + dir_h * offset_h
    
    inner_r= random.uniform(outer_r / 3, outer_r - 0.2)

    circ = gp_Circ(gp_Ax2(gp_Pnt(center[0], center[1], center[2]), normal), outer_r)
    edge = BRepBuilderAPI_MakeEdge(circ, 0., 2*math.pi).Edge()
    outer_wire = BRepBuilderAPI_MakeWire(edge).Wire()

    normal.Reverse()
    circ = gp_Circ(gp_Ax2(gp_Pnt(center[0], center[1], center[2]), normal), inner_r)
    edge = BRepBuilderAPI_MakeEdge(circ, 0., 2*math.pi).Edge()
    inner_wire = BRepBuilderAPI_MakeWire(edge).Wire()

    face_maker = BRepBuilderAPI_MakeFace(outer_wire)
    face_maker.Add(inner_wire)

    return face_maker.Face()

    
def face_pentagon(ref_pnts):
    dir_w = ref_pnts[2] - ref_pnts[1]
    dir_l = ref_pnts[0] - ref_pnts[1]
    dir_r = ref_pnts[3] - ref_pnts[2]    

    normal = np.cross(dir_w, dir_l)    
    edge_dir = np.cross(normal, dir_w)
    edge_dir = edge_dir / np.linalg.norm(edge_dir)

    width = np.linalg.norm(dir_w)    
    height = min(np.dot(dir_l, edge_dir), np.dot(dir_r, edge_dir))
    
    try:
        assert width > 2.0 and height > 2.5, 'width or height too small'
    except AssertionError as error:
        print('face_pentagon', error)
        print(width, height)
        return None       

    offset1 = random.uniform(1.0, height - 2.0)
    offset2 = random.uniform(offset1 + 0.5, height - 1.0)
    
    dir_l = dir_l / np.linalg.norm(dir_l)
    dir_r = dir_r / np.linalg.norm(dir_r)
    
    pt0 = ref_pnts[1] +  dir_l * np.dot(offset1 * edge_dir, dir_l)
    pt1 = ref_pnts[1]
    pt2 = ref_pnts[2]
    pt3 = ref_pnts[2] + dir_r * np.dot(offset1 * edge_dir, dir_r)
    pt4 = (pt1 + pt2) / 2 + edge_dir * offset2

    return occ_utils.face_polygon([pt0, pt1, pt2, pt3, pt4])
    
    
def face_quad(ref_pnts):
    dir_w = ref_pnts[2] - ref_pnts[1]
    dir_l = ref_pnts[0] - ref_pnts[1]
    dir_r = ref_pnts[3] - ref_pnts[2]    

    normal = np.cross(dir_w, dir_l)    
    edge_dir = np.cross(normal, dir_w)
    edge_dir = edge_dir / np.linalg.norm(edge_dir)

    width = np.linalg.norm(dir_w)    
    height = min(np.dot(dir_l, edge_dir), np.dot(dir_r, edge_dir))
    
    try:
        assert width > 2.0 and height > 2.0, 'width or height too small'
    except AssertionError as error:
        print('face_quad', error)
        print(width, height)
        return None       

    offset1 = random.uniform(1.0, height - 1.0)
    offset2 = random.uniform(1.0, height - 1.0)
    
    dir_l = dir_l / np.linalg.norm(dir_l)
    dir_r = dir_r / np.linalg.norm(dir_r)
    
    pt0 = ref_pnts[1] +  dir_l * np.dot(offset1 * edge_dir, dir_l)
    pt1 = ref_pnts[1]
    pt2 = ref_pnts[2]
    pt3 = ref_pnts[2] + dir_r * np.dot(offset2 * edge_dir, dir_r)

    return occ_utils.face_polygon([pt0, pt1, pt2, pt3])

    
def face_rectangle(ref_pnts):
    dir_w = ref_pnts[2] - ref_pnts[1]
    dir_h = ref_pnts[0] - ref_pnts[1]    
    width = np.linalg.norm(dir_w)
    height = np.linalg.norm(dir_h)
    
    try:
        assert width > 1.0 and height > 1.0, 'width or height too small'
    except AssertionError as error:
        print('face_rectangle', error)
        print(width, height)
        return None
    
    dir_w = dir_w / width
    dir_h = dir_h / height
    
    rect_w = random.uniform(1.0, min(width, height))
    rect_h = random.uniform(1.0, min(width, height))
    offset_w = random.uniform(0.0, width - rect_w)
    offset_h = random.uniform(0.0, height - rect_h)
    
    pt1 = ref_pnts[1] + dir_w * offset_w + dir_h * offset_h
    pt2 = pt1 + rect_w * dir_w
    pt3 = pt2 + rect_h * dir_h
    pt4 = pt1 + rect_h * dir_h
    
    return occ_utils.face_polygon([pt1, pt2, pt3, pt4])
    
    
def face_rect_1(ref_pnts):
    edge_dir = ref_pnts[2] - ref_pnts[1]
    width = np.linalg.norm(edge_dir)
    edge_dir = edge_dir / width
    height = np.linalg.norm(ref_pnts[0] - ref_pnts[1])
    if width < 1.0 or height < 1.0:
        return None
    
    rect_w = random.uniform(1.0, width)
    rect_h = random.uniform(1.0, height)
    offset = random.uniform(0.0, width - rect_w)
    
    pnt1 = ref_pnts[1] + edge_dir * offset
    pnt2 = pnt1 + edge_dir * rect_w
    normal = np.cross(edge_dir, ref_pnts[0] - ref_pnts[1])
    edge_normal = np.cross(normal, edge_dir)
    edge_normal = edge_normal / np.linalg.norm(edge_normal)
    pnt3 = pnt2 + edge_normal * rect_h
    pnt4 = pnt1 + edge_normal * rect_h

    return occ_utils.face_polygon([pnt1, pnt2, pnt3, pnt4])

    
def face_rect_2(ref_pnts):
    vec0 = ref_pnts[0] - ref_pnts[1]
    vec2 = ref_pnts[2] - ref_pnts[1]
    if np.linalg.norm(vec0) < 0.000001 or np.linalg.norm(vec2) < 0.000001:
        print('zero length edges')
        print(ref_pnts)
        return None

    ratio0 = random.uniform(0.5 / np.linalg.norm(vec0), 1.0)
    ratio2 = random.uniform(0.5 / np.linalg.norm(vec2), 1.0)

    pt0 = ref_pnts[1] + vec0 * ratio0
    pt1 = ref_pnts[1]
    pt2 = ref_pnts[1] + vec2 * ratio2
    pt3 = ref_pnts[1] + vec0 * ratio0 + vec2 * ratio2        

    return occ_utils.face_polygon([pt0, pt1, pt2, pt3])

    
def face_rect_3(ref_pnts):
    vec1 = ref_pnts[0] - ref_pnts[1]
    vec2 = ref_pnts[3] - ref_pnts[2]

    ratio = random.uniform(0.5 / np.linalg.norm(vec1), 1.0)

    pt0 = ref_pnts[1] + vec1 * ratio
    pt1 = ref_pnts[1]
    pt2 = ref_pnts[2]
    pt3 = ref_pnts[2] + vec2 * ratio

    return occ_utils.face_polygon([pt0, pt1, pt2, pt3])

    
def face_triangle(ref_pnts):
    dir_w = ref_pnts[2] - ref_pnts[1]
    dir_h = ref_pnts[0] - ref_pnts[1]    
    width = np.linalg.norm(dir_w)
    height = np.linalg.norm(dir_h)
    
    try:
        assert width > 2.0 and height > 2.0, 'width or height too small'
    except AssertionError as error:
        print('face_triangle', error)
        print(width, height)
        return None
    
    dir_w = dir_w / width
    dir_h = dir_h / height
    normal = occ_utils.as_occ(np.cross(dir_w, dir_h), gp_Dir)
    
    radius = random.uniform(1.0, min(width / 2, height / 2))
    
    offset_w = random.uniform(radius, width - radius)
    offset_h = random.uniform(radius, height - radius)
    center = ref_pnts[1] + dir_w * offset_w + dir_h * offset_h

    circ = Geom_Circle(gp_Ax2(gp_Pnt(center[0], center[1], center[2]), normal), radius)
    
    ang1 = random.uniform(0.0, 2 * math.pi / 3)
    pt1 = occ_utils.as_list(circ.Value(ang1))

    ang2 = ang1 + random.uniform(2 * math.pi / 3 - math.pi / 9, 2 * math.pi / 3 + math.pi / 9)
    if ang2 > 2 * math.pi:
        ang2 = ang2 - 2 * math.pi
    pt2 = occ_utils.as_list(circ.Value(ang2))

    ang3 = ang2 + random.uniform(2 * math.pi / 3 - math.pi / 9, 2 * math.pi / 3 + math.pi / 9)
    if ang3 > 2 * math.pi:
        ang3 = ang3 - 2 * math.pi
    pt3 = occ_utils.as_list(circ.Value(ang3))

    return occ_utils.face_polygon([pt1, pt2, pt3])

    
def face_triangle_1(ref_pnts):
    edge_dir = ref_pnts[2] - ref_pnts[1]
    width = np.linalg.norm(edge_dir)
    edge_dir = edge_dir / width
    height = np.linalg.norm(ref_pnts[0] - ref_pnts[1])
    if width < 1.0 or height < 1.0:
        return None
    
    tri_w = random.uniform(1.0, width)
    tri_h = random.uniform(1.0, height)
    offset = random.uniform(0.0, width - tri_w)
    
    pnt1 = ref_pnts[1] + edge_dir * offset
    pnt2 = pnt1 + edge_dir * tri_w
    normal = np.cross(edge_dir, ref_pnts[0] - ref_pnts[1])
    edge_normal = np.cross(normal, edge_dir)
    edge_normal = edge_normal / np.linalg.norm(edge_normal)
    pnt3 = pnt1 + edge_dir * tri_w / 2 + edge_normal * tri_h

    return occ_utils.face_polygon([pnt1, pnt2, pnt3])

    
def face_triangle_2(ref_pnts):
    vec0 = ref_pnts[0] - ref_pnts[1]
    vec2 = ref_pnts[2] - ref_pnts[1]
    if np.linalg.norm(vec0) < 0.000001 or np.linalg.norm(vec2) < 0.000001:
        print('zero length edges')
        print(ref_pnts)
        return None

    ratio0 = random.uniform(0.5 / np.linalg.norm(vec0), 1.0)
    ratio2 = random.uniform(0.5 / np.linalg.norm(vec2), 1.0)

    pt0 = ref_pnts[1] + vec0 * ratio0
    pt1 = ref_pnts[1]
    pt2 = ref_pnts[1] + vec2 * ratio2       

    return occ_utils.face_polygon([pt0, pt1, pt2])

          
#==============================================================================
# sketch bound sampler    
#==============================================================================
def face_filter(stock, label_map, num_edge):
    result = []
    for face in occ_utils.list_face(stock):
        if label_map[face] != FEAT_NAMES.index('stock'):
            continue
        
#        f_type = occ_utils.type_face(face)        
#        if f_type != 'plane':
#            continue
        
        # planar face
        if num_edge == 0:
            result.append(face)
            continue
        
        face_util = OCCUtils.face.Face(face)
        normal = occ_utils.as_list(occ_utils.normal_to_face_center(face))
        for wire in face_util.topo.wires():
            edges = [edge for edge in OCCUtils.face.WireExplorer(wire).ordered_edges()]
            if len(edges) < 4:
                continue
            
            good_edge = []            
            for edge in edges:
                if occ_utils.type_edge(edge) != 'line':
                    good_edge.append(False)
                    continue
                
                face_adjacent = occ_utils.face_adjacent(stock, face, edge)
                assert face_adjacent is not None
                if label_map[face_adjacent] != FEAT_NAMES.index('stock'):
                    good_edge.append(False)
                    continue
                
#                if occ_utils.type_face(face_adjacent) != 'plane':
#                    good_edge.append(False)
#                    continue
                                  
                adj_normal = occ_utils.as_list(occ_utils.normal_to_face_center(face_adjacent))
                pnt1 = np.array(occ_utils.as_list(topexp.FirstVertex(edge, True)))
                pnt2 = np.array(occ_utils.as_list(topexp.LastVertex(edge, True)))
                edge_dir = pnt2 - pnt1
                if np.dot(np.cross(normal, adj_normal), edge_dir) < 1e-6:
                    good_edge.append(False)
                    continue
                
                good_edge.append(True)
                
            for i in range(len(edges)):
                j = (i + 1) % len(edges)
                k = (i + 2) % len(edges)
                               
                if not good_edge[i]:
                    continue
                
                if num_edge == 1: 
                    result.append([face, edge])
                    continue
                
                if not good_edge[j]:
                    continue
                
                pnt = np.array(occ_utils.as_list(topexp.FirstVertex(edges[i], True)))
                pntj = np.array(occ_utils.as_list(topexp.FirstVertex(edges[j], True)))
                pntk = np.array(occ_utils.as_list(topexp.FirstVertex(edges[k], True)))
                if not geom_utils.point_in_polygon(pnt, np.array([pntj, pntk]), False, normal):
                    continue
                
                if num_edge == 2:                       
                    result.append([face, edges[i], edges[j]])
                    continue
                
                if not good_edge[k]:
                    continue
                
                pnt = np.array(occ_utils.as_list(topexp.LastVertex(edges[k], True)))
                if not geom_utils.point_in_polygon(pnt, np.array([pntj, pntk]), False, normal):
                    continue
                
                if num_edge == 3:
                    result.append([face, edges[i], edges[j], edges[k]])

    return result

    
def sample_points_inside_face(face):
    tri_pnts, triangles, _, et_map = triangulation_from_face(face)
    sample_points = []

    # sketch on 3 edges
    for edge in et_map:
        if len(et_map[edge]) > 1:
            pt1 = np.asarray(tri_pnts[edge[0]])
            pt2 = np.asarray(tri_pnts[edge[1]])
            sample_points.append(((pt1 + pt2) / 2).tolist())

    for tri in triangles:
        pt1 = np.asarray(tri_pnts[tri[0]])
        pt2 = np.asarray(tri_pnts[tri[1]])
        pt3 = np.asarray(tri_pnts[tri[2]])
        sample_points.append(((pt1 + pt2 + pt3) / 3).tolist())

    return sample_points

    
def bound_inner(shape, label_map):
    fe_list = face_filter(shape, label_map, 0)
    bounds = []
    for face in fe_list:
        normal = np.array(occ_utils.as_list(occ_utils.normal_to_face_center(face)))
        sample_pnts = np.array(sample_points_inside_face(face))
        edges = occ_utils.list_edge(face)    
        for apnt in sample_pnts:
            dist, pnt = occ_utils.dist_point_to_edges(apnt, edges)        
            if dist >= 2.0:
                dir_w = apnt - np.array(pnt)
                dir_w = dir_w / np.linalg.norm(dir_w)
                dir_h = np.cross(dir_w, normal)
                half_len = dist * math.sqrt(2) / 2
                pnt0 = apnt - dir_w * half_len - dir_h * half_len 
                pnt1 = apnt - dir_w * half_len + dir_h * half_len
                pnt2 = apnt + dir_w * half_len + dir_h * half_len
                pnt3 = apnt + dir_w * half_len - dir_h * half_len
                bounds.append([pnt0, pnt1, pnt2, pnt3, -normal])
                
    return bounds
   
    
def bound_1(shape, label_map):
    fe_list = face_filter(shape, label_map, 1)
    
    bounds = []
    for item in fe_list:
        face = item[0]
        edge = item[1]
        pts, triangles, vt_map, et_map = triangulation_from_face(face)        
        segs = []
        for et in et_map:
            if len(et_map[et]) == 1:
                segs.append([pts[et[0]], pts[et[1]]])              
        pts = np.asarray(pts)
        normal = np.array(occ_utils.as_list(occ_utils.normal_to_face_center(face)))

        edge_util = OCCUtils.edge.Edge(edge)
        if edge_util.length() < 3.0:
            continue
                
        pnt1 = np.array(occ_utils.as_list(topexp.FirstVertex(edge, True)))
        pnt2 = np.array(occ_utils.as_list(topexp.LastVertex(edge, True)))
                        
        edge_dir = pnt2 - pnt1
        edge_len = np.linalg.norm(edge_dir)
        
        edge_dir = edge_dir / edge_len
        pnt1 = pnt1 + edge_dir * 0.5
        pnt2 = pnt2 - edge_dir * 0.5
        edge_len -= 1.0
                    
        edge_normal = np.cross(normal, edge_dir)
        edge_normal = edge_normal / np.linalg.norm(edge_normal)
        sample_pnts = [pnt1 + t * edge_dir for t in [0.0, edge_len * 0.3, edge_len * 0.6]]
        for pnt in sample_pnts:
            intersects = geom_utils.ray_segment_set_intersect(pnt, edge_normal, segs)                
            intersects.sort()
            intersects = intersects[1:]
            
            if len(intersects) == 0:
                continue
             
            edge_len = np.linalg.norm(pnt2 - pnt)
            bound = geom_utils.search_rect_inside_bound_2(pnt, edge_normal * intersects[0], edge_dir * edge_len, pts)
            if bound is not None:                    
                bound = np.append(bound, [-normal], axis=0)
                bounds.append(bound) 
                
    return bounds


def bound_2(shape, label_map):
    fe_list = face_filter(shape, label_map, 2)
    bounds = []
    for item in fe_list:
        face = item[0]
        edge1 = item[1]
        edge2 = item[2]

        pts, triangles, vt_map, et_map = triangulation_from_face(face)
        pts = np.asarray(pts)
                
        pnt0 = np.array(occ_utils.as_list(topexp.FirstVertex(edge1, True)))
        pnt1 = np.array(occ_utils.as_list(topexp.FirstVertex(edge2, True)))        
        pnt2 = np.array(occ_utils.as_list(topexp.LastVertex(edge2, True)))        
           
        vec0 = pnt0 - pnt1
        vec2 = pnt2 - pnt1
        
        bound = geom_utils.search_rect_inside_bound_2(pnt1, vec0, vec2, pts)
        if bound is not None:
            normal = np.array(occ_utils.as_list(occ_utils.normal_to_face_center(face)))
            bound = np.append(bound, [-normal], axis=0)
            bounds.append(bound)
    
    return bounds

    
def bound_3(shape, label_map):
    '''
    output:
        bounds: [np.array[[float,float,float]*4]]
    '''
    bounds = []
    fe_list = face_filter(shape, label_map, 3)
    
    for item in fe_list:
        face = item[0]
        edge1 = item[1]
        edge2 = item[2]
        edge3 = item[3]

        pts, triangles, vt_map, et_map = triangulation_from_face(face)
        pts = np.asarray(pts)

        pnt0 = np.array(occ_utils.as_list(topexp.FirstVertex(edge1, True)))
        pnt1 = np.array(occ_utils.as_list(topexp.FirstVertex(edge2, True)))
        pnt2 = np.array(occ_utils.as_list(topexp.FirstVertex(edge3, True)))
        pnt3 = np.array(occ_utils.as_list(topexp.LastVertex(edge3, True)))

        vec1 = pnt0 - pnt1
        vec2 = pnt3 - pnt2
        bound = geom_utils.search_rect_inside_bound_3(pnt1, pnt2, vec1, vec2, pts)
        if bound is not None:
            normal = np.array(occ_utils.as_list(occ_utils.normal_to_face_center(face)))
            bound = np.append(bound, [-normal], axis=0)            
            bounds.append(bound)

    return bounds

    
#==============================================================================
#     feature depth sampler
#==============================================================================
    
    
def depth_blind(bound, triangles):
    points = geom_utils.points_inside_rect(bound[0], bound[1], bound[2], bound[3], 1.0)
    depths = []
    for pnt in points:
        dpt = geom_utils.ray_triangle_set_intersect(pnt, bound[4], triangles)
        if dpt > 0:
            depths.append(dpt)
    
    if len(depths) > 0:
        depth = min(depths)
    else:
        depth = 10.0
    
    if depth < 2.0:
        return float('-inf')
        
    return random.uniform(1.0, depth - 1.0)

def depth_through(bound, triangles):
    return 10.0
    
SKETCH_BOUND_SAMPLER = {'Oring': bound_inner, 'through_hole': bound_inner,
                      'blind_hole': bound_inner, 'triangular_passage': bound_inner,
                      'rectangular_passage': bound_inner, 'circular_through_slot': bound_1,
                      'triangular_through_slot': bound_1, 'rectangular_through_slot': bound_1,
                      'rectangular_blind_slot': bound_1, 'triangular_pocket': bound_inner,
                      'rectangular_pocket': bound_inner, 'circular_end_pocket': bound_inner,
                      'triangular_blind_step': bound_2, 'circular_blind_step': bound_2,
                      'rectangular_blind_step': bound_2, 'rectangular_through_step': bound_3,
                      '2sides_through_step': bound_3, 'slanted_through_step': bound_3,
                      'v_circular_end_blind_slot': bound_1, 'h_circular_end_blind_slot': bound_1,
                      '6sides_passage': bound_inner, '6sides_pocket': bound_inner}

SKETCH_GENERATOR = {'Oring': face_oring, 'through_hole': face_circle,
                 'blind_hole': face_circle, 'triangular_passage': face_triangle,
                 'rectangular_passage': face_rectangle, 'circular_through_slot': face_circle_1,
                 'triangular_through_slot': face_triangle_1, 'rectangular_through_slot': face_rect_1,
                 'rectangular_blind_slot': face_rect_1, 'triangular_pocket': face_triangle,
                 'rectangular_pocket': face_rectangle, 'circular_end_pocket': face_circular_end_rect,
                 'triangular_blind_step': face_triangle_2, 'circular_blind_step': face_circle_2,
                 'rectangular_blind_step': face_rect_2, 'rectangular_through_step': face_rect_3,
                 '2sides_through_step': face_pentagon, 'slanted_through_step': face_quad,
                 'v_circular_end_blind_slot': face_open_circular_end_rect_v, 'h_circular_end_blind_slot': face_open_circular_end_rect_h,
                 '6sides_passage': face_hexagon, '6sides_pocket': face_hexagon}
                 
FEAT_DEPTH_SAMPLER = {'Oring': depth_blind, 'through_hole': depth_through,
                      'blind_hole': depth_blind, 'triangular_passage': depth_through,
                      'rectangular_passage': depth_through, 'circular_through_slot': depth_through,
                      'triangular_through_slot': depth_through, 'rectangular_through_slot': depth_through,
                      'rectangular_blind_slot': depth_blind, 'triangular_pocket': depth_blind,
                      'rectangular_pocket': depth_blind, 'circular_end_pocket': depth_blind,
                      'triangular_blind_step': depth_blind, 'circular_blind_step': depth_blind,
                      'rectangular_blind_step': depth_blind, 'rectangular_through_step': depth_blind,
                      '2sides_through_step': depth_blind, 'slanted_through_step': depth_blind,
                      'v_circular_end_blind_slot': depth_blind, 'h_circular_end_blind_slot': depth_blind,
                      '6sides_passage': depth_through, '6sides_pocket': depth_blind}
                 
FEAT_NAMES = ['Oring', 'through_hole', 'blind_hole', 'triangular_passage',
              'triangular_pocket', 'rectangular_passage', 'rectangular_pocket',
              '6sides_passage', '6sides_pocket', 'circular_end_pocket',         #10
              'triangular_through_slot', 'rectangular_through_slot',
              'circular_through_slot', 'rectangular_blind_slot', 
              'v_circular_end_blind_slot', 'h_circular_end_blind_slot',         #6
              'triangular_blind_step', 'circular_blind_step', 'rectangular_blind_step', #3
              'rectangular_through_step', '2sides_through_step', 'slanted_through_step', #3
              'chamfer', 'round', 'stock']

              
def apply_feature(old_shape, old_labels, feat_type, feat_face, depth_dir):
    feature_maker = BRepFeat_MakePrism()
    feature_maker.Init(old_shape, feat_face, TopoDS_Face(), occ_utils.as_occ(depth_dir, gp_Dir), False, False)
    feature_maker.Build()

    feature_maker.Perform(np.linalg.norm(depth_dir))
    shape = feature_maker.Shape()

    fmap = shape_factory.map_face_before_and_after_feat(old_shape, feature_maker)
    new_labels = shape_factory.map_from_shape_and_name(fmap, old_labels, shape, FEAT_NAMES.index(feat_type))
    return shape, new_labels


def triangulate_shape(shape):
    linear_deflection = 0.1
    angular_deflection = 0.5
    mesh = BRepMesh_IncrementalMesh(shape, linear_deflection, False, angular_deflection, True)
    mesh.Perform()
    assert mesh.IsDone()

    
def add_chamfer_round(stock, label_map):
    fillet_maker = BRepFilletAPI_MakeFillet(stock)
    edges = occ_utils.list_edge(stock)
    # to do: select edge
    
    fillet_maker.Add(1.0, random.choice(edges))
   
    shape = fillet_maker.Shape()
    fmap = shape_factory.map_face_before_and_after_feat(stock, fillet_maker)
    label_map = shape_factory.map_from_shape_and_name(fmap, label_map, shape, FEAT_NAMES.index('round'))   
    
    return shape, label_map

    
def shape_from_machining_feature():
    stock = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape()
    label_map = shape_factory.map_from_name(stock, FEAT_NAMES.index('stock'))
    triangulate_shape(stock)

    num_feats = random.randint(3, 5)
    bounds = []
    feat_cnt = 0
    while True:
        triangulate_shape(stock)
        # step 1: sample feature arameters [type, width, depth]
        feat_type = random.choice(FEAT_NAMES[:-3])
        print(feat_type)
        bounds = SKETCH_BOUND_SAMPLER[feat_type](stock, label_map)
        
        feat_face = None
        faces = occ_utils.list_face(stock)
        triangles = triangles_from_faces(faces)
        random.shuffle(bounds)
        for bound in bounds:
            depth = FEAT_DEPTH_SAMPLER[feat_type](bound, triangles)
            if depth < 0:
                continue
            
            # step 3: create feature face            
            feat_face = SKETCH_GENERATOR[feat_type](bound)
            if feat_face is not None:
                break

        # step 4: apply feature operation
        if feat_face is None:
            continue
        
        print(feat_cnt, feat_type)
        feat_cnt += 1
        stock, label_map = apply_feature(stock, label_map, feat_type, feat_face, bound[4] * depth)
        if feat_cnt == num_feats:
            break
    
#    stock, label_map = add_chamfer_round(stock, label_map)

    return stock, label_map, bounds


if __name__ == '__main__':

    OCC_DISPLAY, START_OCC_DISPLAY, ADD_MENU, ADD_FUNCTION_TO_MENU = init_display()
    OCC_DISPLAY.EraseAll()

    colors = []
    rgb_list = np.array(np.meshgrid([0.9, 0.6, 0.3], [0.9, 0.6, 0.3], [0.9, 0.6, 0.3])).T.reshape(-1,3)
    for rgb in rgb_list:
        colors.append(rgb_color(rgb[0], rgb[1], rgb[2]))

    random.seed()
    SHAPE, FMAP, bounds = shape_from_machining_feature()
#    for bound in bounds:
#        for i in range(4):
#            j = (i+1)%4
#            pnt1 = occ_utils.as_occ(bound[i], gp_Pnt)
#            pnt2 = occ_utils.as_occ(bound[j], gp_Pnt)
#            if np.linalg.norm(bound[i] - bound[j]) < 0.000001:
#                print('bound edge has zero length')
#                continue
#            try:
#                seg_maker = GC_MakeSegment(pnt1, pnt2)
#                edge = BRepBuilderAPI_MakeEdge(seg_maker.Value()).Edge()
#                OCC_DISPLAY.DisplayShape(edge)
#            except RuntimeError as error:
#                print('main', error)
#                print(bound[i], bound[j])

    AIS = AIS_ColoredShape(SHAPE)
    for a_face in FMAP:
        AIS.SetCustomColor(a_face, colors[FMAP[a_face]])

    OCC_DISPLAY.Context.Display(AIS.GetHandle())
    OCC_DISPLAY.View_Iso()
    OCC_DISPLAY.FitAll()

    START_OCC_DISPLAY()