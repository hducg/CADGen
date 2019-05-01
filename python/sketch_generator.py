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
