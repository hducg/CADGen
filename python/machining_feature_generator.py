# _*_ coding: utf_8 _*_
"""
Created on Tue Mar 19 11:20:39 2019

@author: 2624224
"""

import sys
import random
import math

import numpy as np

from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace
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

sys.path.append('../CADGen/python')
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
                et_map[edge].append( i -1)
            else:
                et_map[edge] = [i-1]

    return pts, triangles, vt_map, et_map

def triangles_from_faces(faces):
    tri_list = []
    for face in faces:
        pts, triangles, vt_map, et_map = triangulation_from_face(face)
        for tri in triangles:
            tri_list.append((pts[tri[0]], pts[tri[1]], pts[tri[2]]))

    return tri_list


def face_oring(ref_pnts, normal):
    center = ref_pnts[0]
    radius = np.linalg.norm(np.asarray(ref_pnts[0]) - np.asarray(ref_pnts[1]))

    outer_r = random.uniform(1, radius - 0.5)
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


def face_circle(ref_pnts, normal):
    center = ref_pnts[0]
    radius = np.linalg.norm(np.asarray(ref_pnts[0]) - np.asarray(ref_pnts[1]))

    radius = random.uniform(1.0, radius - 0.5)

    circ = gp_Circ(gp_Ax2(gp_Pnt(center[0], center[1], center[2]), normal), radius)
    edge = BRepBuilderAPI_MakeEdge(circ, 0., 2*math.pi).Edge()
    outer_wire = BRepBuilderAPI_MakeWire(edge).Wire()

    face_maker = BRepBuilderAPI_MakeFace(outer_wire)

    return face_maker.Face()


def face_triangle(ref_pnts, normal):
    center = ref_pnts[0]
    radius = np.linalg.norm(np.asarray(ref_pnts[0]) - np.asarray(ref_pnts[1]))

    radius = random.uniform(1.0, radius - 0.5)
    circ = Geom_Circle(gp_Ax2(gp_Pnt(center[0], center[1], center[2]), normal), radius)

    ang1 = random.uniform(0.0, 2 * math.pi / 3)
    pt1 = circ.Value(ang1)

    ang2 = ang1 + random.uniform(2 * math.pi / 3 - math.pi / 9, 2 * math.pi / 3 + math.pi / 9)
    if ang2 > 2 * math.pi:
        ang2 = ang2 - 2 * math.pi
    pt2 = circ.Value(ang2)

    ang3 = ang2 + random.uniform(2 * math.pi / 3 - math.pi / 9, 2 * math.pi / 3 + math.pi / 9)
    if ang3 > 2 * math.pi:
        ang3 = ang3 - 2 * math.pi
    pt3 = circ.Value(ang3)

    ed1 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt1, pt2).Value()).Edge()
    ed2 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt2, pt3).Value()).Edge()
    ed3 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt3, pt1).Value()).Edge()

    the_wire = BRepBuilderAPI_MakeWire(ed1, ed2, ed3).Wire()
    face_maker = BRepBuilderAPI_MakeFace(the_wire)

    return face_maker.Face()


def face_rectangle(ref_pnts, normal):
    center = ref_pnts[0]
    radius = np.linalg.norm(np.asarray(ref_pnts[0]) - np.asarray(ref_pnts[1]))

    radius = random.uniform(1.0, radius - 0.5)
    circ = Geom_Circle(gp_Ax2(gp_Pnt(center[0], center[1], center[2]), normal), radius)

    ang1 = random.uniform(0.0, math.pi / 2)
    pt1 = circ.Value(ang1)

    ang2 = ang1 + random.uniform(math.pi / 4, 3 * math.pi / 4)
    pt2 = circ.Value(ang2)

    ang3 = ang1 + math.pi
    pt3 = circ.Value(ang3)

    ang4 = ang2 + math.pi
    if ang4 > 2 * math.pi:
        ang4 -= 2 * math.pi
    pt4 = circ.Value(ang4)

    ed1 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt1, pt2).Value()).Edge()
    ed2 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt2, pt3).Value()).Edge()
    ed3 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt3, pt4).Value()).Edge()
    ed4 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt4, pt1).Value()).Edge()
    the_wire = BRepBuilderAPI_MakeWire(ed1, ed2, ed3, ed4).Wire()
    face_maker = BRepBuilderAPI_MakeFace(the_wire)

    return face_maker.Face()


def face_hexagon(ref_pnts, normal):
    center = ref_pnts[0]
    radius = np.linalg.norm(np.asarray(ref_pnts[0]) - np.asarray(ref_pnts[1]))

    radius = random.uniform(1.0, radius - 0.5)
    circ = Geom_Circle(gp_Ax2(gp_Pnt(center[0], center[1], center[2]), normal), radius)

    ang1 = random.uniform(0.0, math.pi / 3)
    pt1 = circ.Value(ang1)

    ang2 = ang1 + math.pi / 3
    pt2 = circ.Value(ang2)

    ang3 = ang2 + math.pi / 3
    pt3 = circ.Value(ang3)

    ang4 = ang3 + math.pi / 3
    pt4 = circ.Value(ang4)

    ang5 = ang4 + math.pi / 3
    pt5 = circ.Value(ang5)

    ang6 = ang5 + math.pi / 3
    pt6 = circ.Value(ang6)

    wire_maker = BRepBuilderAPI_MakeWire()
    pnts = [pt1, pt2, pt3, pt4, pt5, pt6, pt1]
    for i in range(len(pnts) - 1):
        edge = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pnts[i], pnts[i+1]).Value()).Edge()
        wire_maker.Add(edge)

    face_maker = BRepBuilderAPI_MakeFace(wire_maker.Wire())

    return face_maker.Face()


def face_circular_end_rect(ref_pnts, normal):

    return None

def face_open_circle_1(ref_pnts, normal):
    return None

def face_circle_2(ref_pnts):
    vec0 = ref_pnts[0] - ref_pnts[1]
    vec2 = ref_pnts[2] - ref_pnts[1]
    if np.linalg.norm(vec0) < 0.000001 or np.linalg.norm(vec2) < 0.000001:
        print('zero length edges')
        print(ref_pnts)
        return None

    radius = random.uniform(0.5, 0.5 * min(np.linalg.norm(vec0), np.linalg.norm(vec2)))
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

    
def face_open_triangle_1(ref_pnts, normal):
    return None

def face_triangle_2(ref_pnts):
    vec0 = ref_pnts[0] - ref_pnts[1]
    vec2 = ref_pnts[2] - ref_pnts[1]
    if np.linalg.norm(vec0) < 0.000001 or np.linalg.norm(vec2) < 0.000001:
        print('zero length edges')
        print(ref_pnts)
        return None

    ratio0 = random.uniform(0.5 / np.linalg.norm(vec0), 0.5)
    ratio2 = random.uniform(0.5 / np.linalg.norm(vec2), 0.5)

    pt0 = occ_utils.as_occ(ref_pnts[1] + vec0 * ratio0, gp_Pnt)
    pt1 = occ_utils.as_occ(ref_pnts[1], gp_Pnt)
    pt2 = occ_utils.as_occ(ref_pnts[1] + vec2 * ratio2, gp_Pnt)
        
    seg_maker = [GC_MakeSegment(pt0, pt1), GC_MakeSegment(pt1, pt2), GC_MakeSegment(pt2, pt0)]
    wire_maker = BRepBuilderAPI_MakeWire()
    for sm in seg_maker:
        if sm.IsDone:
            edge = BRepBuilderAPI_MakeEdge(sm.Value()).Edge()
            wire_maker.Add(edge)
        else:
            return None
            
    face_maker = BRepBuilderAPI_MakeFace(wire_maker.Wire())

    return face_maker.Face()

    
def face_open_rect_1(ref_pnts, normal):
    return None

def face_rect_2(ref_pnts):
    vec0 = ref_pnts[0] - ref_pnts[1]
    vec2 = ref_pnts[2] - ref_pnts[1]
    if np.linalg.norm(vec0) < 0.000001 or np.linalg.norm(vec2) < 0.000001:
        print('zero length edges')
        print(ref_pnts)
        return None

    ratio0 = random.uniform(0.5 / np.linalg.norm(vec0), 0.5)
    ratio2 = random.uniform(0.5 / np.linalg.norm(vec2), 0.5)

    pt0 = occ_utils.as_occ(ref_pnts[1] + vec0 * ratio0, gp_Pnt)
    pt1 = occ_utils.as_occ(ref_pnts[1], gp_Pnt)
    pt2 = occ_utils.as_occ(ref_pnts[1] + vec2 * ratio2, gp_Pnt)
    pt3 = occ_utils.as_occ(ref_pnts[1] + vec0 * ratio0 + vec2 * ratio2, gp_Pnt)
        
    seg_maker = [GC_MakeSegment(pt0, pt1), GC_MakeSegment(pt1, pt2), 
                 GC_MakeSegment(pt2, pt3), GC_MakeSegment(pt3, pt0)]
    wire_maker = BRepBuilderAPI_MakeWire()
    for sm in seg_maker:
        if sm.IsDone:
            edge = BRepBuilderAPI_MakeEdge(sm.Value()).Edge()
            wire_maker.Add(edge)
        else:
            return None
            
    face_maker = BRepBuilderAPI_MakeFace(wire_maker.Wire())

    return face_maker.Face()

def face_rect_3(ref_pnts):
    vec1 = ref_pnts[0] - ref_pnts[1]
    vec2 = ref_pnts[3] - ref_pnts[2]

    ratio = random.uniform(0.5 / np.linalg.norm(vec1), 0.5)

    pt0 = occ_utils.as_occ(ref_pnts[1] + vec1 * ratio, gp_Pnt)
    pt1 = occ_utils.as_occ(ref_pnts[1], gp_Pnt)
    pt2 = occ_utils.as_occ(ref_pnts[2], gp_Pnt)
    pt3 = occ_utils.as_occ(ref_pnts[2] + vec2 * ratio, gp_Pnt)

    ed1 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt0, pt1).Value()).Edge()
    ed2 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt1, pt2).Value()).Edge()
    ed3 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt2, pt3).Value()).Edge()
    ed4 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt3, pt0).Value()).Edge()
    the_wire = BRepBuilderAPI_MakeWire(ed1, ed2, ed3, ed4).Wire()
    face_maker = BRepBuilderAPI_MakeFace(the_wire)

    return face_maker.Face()

    
def face_open_pentagon(ref_pnts):
    vec1 = ref_pnts[0] - ref_pnts[1]
    vec2 = ref_pnts[3] - ref_pnts[2]

    ratio = random.uniform(0.5 / np.linalg.norm(vec1), 0.5)

    pt0 = occ_utils.as_occ(ref_pnts[1] + vec1 * ratio, gp_Pnt)
    pt1 = occ_utils.as_occ(ref_pnts[1], gp_Pnt)
    pt2 = occ_utils.as_occ(ref_pnts[2], gp_Pnt)
    pt3 = occ_utils.as_occ(ref_pnts[2] + vec2 * ratio, gp_Pnt)

    ed1 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt0, pt1).Value()).Edge()
    ed2 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt1, pt2).Value()).Edge()
    ed3 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt2, pt3).Value()).Edge()
    ed4 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt3, pt0).Value()).Edge()
    the_wire = BRepBuilderAPI_MakeWire(ed1, ed2, ed3, ed4).Wire()
    face_maker = BRepBuilderAPI_MakeFace(the_wire)

    return face_maker.Face()


def face_open_quad(ref_pnts):
    vec1 = ref_pnts[0] - ref_pnts[1]
    vec2 = ref_pnts[3] - ref_pnts[2]

    ratio1 = random.uniform(0.5 / np.linalg.norm(vec1), 0.5)
    ratio2 = random.uniform(0.5 / np.linalg.norm(vec2), 0.5)
    while abs(ratio1 - ratio2) < 0.000001:
        ratio2 = random.uniform(0.5 / np.linalg.norm(vec2), 0.5)
    
    pt0 = occ_utils.as_occ(ref_pnts[1] + vec1 * ratio1, gp_Pnt)
    pt1 = occ_utils.as_occ(ref_pnts[1], gp_Pnt)
    pt2 = occ_utils.as_occ(ref_pnts[2], gp_Pnt)
    pt3 = occ_utils.as_occ(ref_pnts[2] + vec2 * ratio2, gp_Pnt)

    ed1 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt0, pt1).Value()).Edge()
    ed2 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt1, pt2).Value()).Edge()
    ed3 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt2, pt3).Value()).Edge()
    ed4 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt3, pt0).Value()).Edge()
    the_wire = BRepBuilderAPI_MakeWire(ed1, ed2, ed3, ed4).Wire()
    face_maker = BRepBuilderAPI_MakeFace(the_wire)

    return face_maker.Face()    


def face_open_circular_end_rect_v(ref_pnts, normal):
    return None

def face_open_circular_end_rect_h(ref_pnts, normal):
    return None

def bound_face(f_list):
    
    return None

def bound_1(f_list):
    bounds = []
    for face in f_list:
        pts, triangles, vt_map, et_map = triangulation_from_face(face)
        pts = np.asarray(pts)
        normal = np.array(occ_utils.as_list(occ_utils.normal_to_face_center(face)))        
        f_util = OCCUtils.face.Face(face)
        edges = f_util.edges()
        for edge in edges:
            if occ_utils.type_edge(edge) != 'line':
                continue
            edge_util = OCCUtils.edge.Edge(edge)
            if edge_util.length() < 2.0:
                continue
            
            pnt1 = occ_utils.as_list(edge_util.first_vertex())
            pnt2 = occ_utils.as_list(edge_util.last_vertex())
            # make sure pnt1-->pnt2 in the right direction of the loop 
            if edge_util.Orientation() == TopAbs_REVERSED:
                pnt = pnt2
                pnt2 = pnt1
                pnt1 = pnt
            edge_dir = pnt2 - pnt1
            edge_len = np.linalg.norm(edge_dir)
            edge_dir = edge_dir / edge_len
            pnt1 = pnt1 + edge_dir * 0.5
            pnt2 = pnt2 - edge_dir * 0.5
            edge_len -= 1.0
            
            edge_normal = np.cross(normal, edge_dir)
            
            sample_pnts = [pnt1 + t * edge_dir for t in [0.0, edge_len * 0.25, edge_len * 0.5, edge_len * 0.75]]
            for pnt in sample_pnts:
                bounds.append(geom_utils.search_rect_inside_bound_2(pnt, edge_normal, edge_dir, pts))
            
    return bounds


def bound_2(f_list):
    bounds = []
    for face in f_list:
        pts, triangles, vt_map, et_map = triangulation_from_face(face)
        pts = np.asarray(pts)
        normal = np.array(occ_utils.as_list(occ_utils.normal_to_face_center(face)))
        f_topo = OCCUtils.Topology.Topo(face)
        verts = [vert for vert in f_topo.vertices()]
        for vert in verts:
            edges = occ_utils.edges_at_vertex(vert, face)
            v_list = []
            for edge in edges:
                if occ_utils.type_edge(edge) == 'line':
                    v_topo = OCCUtils.Topology.Topo(edge)
                    for ve in v_topo.vertices():
                        if not vert.IsSame(ve):
                            v_list.append(ve)
            if len(v_list) < 2:                
                continue
            
            pnt1 = np.array(occ_utils.as_list(vert))
            pnt0 = np.array(occ_utils.as_list(v_list[0]))
            pnt2 = np.array(occ_utils.as_list(v_list[1]))
            
            if np.dot(np.cross(pnt1 - pnt0, pnt2 - pnt1 ), normal) < 0:
                pnt = pnt0
                pnt0 = pnt2
                pnt2 = pnt                
                   
            vec0 = pnt0 - pnt1
            vec2 = pnt2 - pnt1
            if not geom_utils.point_in_polygon(pnt0, np.array([pnt1, pnt2]), False, normal):
                print('concave vertex')
                continue
                   
            bounds.append(geom_utils.search_rect_inside_bound_2(pnt1, vec0, vec2, pts))
    return bounds

def bound_3(f_list):
    '''
    output:
        bounds: [np.array[[float,float,float]*4]]
    '''
    bounds = []

    for face in f_list:
        face_util = OCCUtils.face.Face(face)
        edges = face_util.edges()
        if len(edges) < 4:
            continue

        pts, triangles, vt_map, et_map = triangulation_from_face(face)
        pts = np.asarray(pts)
        for wire in face_util.topo.wires():
            edges = [edge for edge in OCCUtils.face.WireExplorer(wire).ordered_edges()]
            is_line = []
            for an_edge in edges:
                is_line.append(occ_utils.type_edge(an_edge) == 'line')

            for i in range(len(edges)):
                if not is_line[i]:
                    continue
                j = (i + 1) % len(edges)
                if not is_line[j]:
                    continue
                k = (i + 2) % len(edges)
                if not is_line[k]:
                    continue

                eei = OCCUtils.edge.Edge(edges[i])
                eek = OCCUtils.edge.Edge(edges[k])
                vert1 = eei.common_vertex(edges[j])
                verts = [eei.first_vertex(), eei.last_vertex()]
                verts.remove(vert1)
                vert0 = verts[0]
                vert2 = eek.common_vertex(edges[j])
                verts = [eek.first_vertex(), eek.last_vertex()]
                verts.remove(vert2)
                vert3 = verts[0]
                verts = [vert0, vert1, vert2, vert3]
                verts = np.asarray([occ_utils.as_list(vert) for vert in verts])

                vec1 = verts[0] - verts[1]
                vec2 = verts[3] - verts[2]

                bounds.append(geom_utils.search_rect_inside_bound_3(verts[1], verts[2], vec1, vec2, pts))

    return bounds

FEAT_BOUND_SAMPLER = {'Oring': bound_face, 'through_hole': bound_face,
                      'blind_hole': bound_face, 'triangular_passage': bound_face,
                      'rectangular_passage': bound_face, 'circular_through_slot': bound_1,
                      'triangular_through_slot': bound_1, 'rectangular_through_slot': bound_1,
                      'rectangular_blind_slot': bound_1, 'triangular_pocket': bound_face,
                      'rectangular_pocket': bound_face, 'circular_end_pocket': bound_face,
                      'triangular_blind_step': bound_2, 'circular_blind_step': bound_2,
                      'rectangular_blind_step': bound_2, 'rectangular_through_step': bound_3,
                      '2sides_through_step': bound_3, 'slanted_through_step': bound_3,
                      'v_circular_end_blind_slot': bound_1, 'h_circular_end_blind_slot': bound_1,
                      '6sides_passage': bound_face, '6sides_pocket': bound_face}

FACE_GENERATOR = {'Oring': face_oring, 'through_hole': face_circle,
                 'blind_hole': face_circle, 'triangular_passage': face_triangle,
                 'rectangular_passage': face_rectangle, 'circular_through_slot': face_open_circle_1,
                 'triangular_through_slot': face_open_triangle_1, 'rectangular_through_slot': face_open_rect_1,
                 'rectangular_blind_slot': face_open_rect_1, 'triangular_pocket': face_triangle,
                 'rectangular_pocket': face_rectangle, 'circular_end_pocket': face_circular_end_rect,
                 'triangular_blind_step': face_triangle_2, 'circular_blind_step': face_circle_2,
                 'rectangular_blind_step': face_rect_2, 'rectangular_through_step': face_rect_3,
                 '2sides_through_step': face_open_pentagon, 'slanted_through_step': face_open_quad,
                 'v_circular_end_blind_slot': face_open_circular_end_rect_v, 'h_circular_end_blind_slot': face_open_circular_end_rect_h,
                 '6sides_passage': face_hexagon, '6sides_pocket': face_hexagon}
                 
FEAT_NAMES = ['Oring', 'through_hole', 'blind_hole', 'triangular_passage',
              'triangular_pocket', 'rectangular_passage', 'rectangular_pocket',
              '6sides_passage', '6sides_pocket', 'circular_end_pocket',         #10
              'triangular_through_slot', 'rectangular_through_slot',
              'rectangular_blind_slot', 'circular_through_slot',
              'v_circular_end_blind_slot', 'h_circular_end_blind_slot',         #6
              'triangular_blind_step', 'circular_blind_step', 'rectangular_blind_step', #3
              'rectangular_through_step', '2sides_through_step', 'slanted_through_step', #3
              'chamfer', 'round', 'stock']

def apply_feature(old_shape, old_labels, feat_type, feat_face, depth):
    direction = occ_utils.normal_to_face_center(feat_face)
    direction.Reverse()

    feature_maker = BRepFeat_MakePrism()
    feature_maker.Init(old_shape, feat_face, TopoDS_Face(), direction, False, False)
    feature_maker.Build()

    feature_maker.Perform(depth)
    shape = feature_maker.Shape()

    fmap = shape_factory.map_face_before_and_after_feat(old_shape, feature_maker)
    new_labels = shape_factory.map_from_shape_and_name(fmap, old_labels, shape, FEAT_NAMES.index(feat_type))
    return shape, new_labels




SKETCH_INSIDE_FACE = ['Oring', 'through_hole', 'blind_hole', 'triangular_passage',
                      'triangular_pocket', 'rectangular_passage', 'rectangular_pocket',
                      '6sides_passage', '6sides_pocket', 'circular_end_pocket']

SKETCH_ON_EDGE = ['triangular_through_slot', 'rectangular_through_slot',
                  'rectangular_blind_slot', 'circular_through_slot',
                  'v_circular_end_blind_slot', 'h_circular_end_blind_slot']

SKETCH_ON_VERTEX = ['triangular_blind_step', 'circular_blind_step', 'rectangular_blind_step']

SKETCH_ON_EDGES = ['rectangular_through_step', '2sides_through_step', 'slanted_through_step']



def points_filter_by_dist(sample_points, face):
    dists = []
    nearest_pnts = []

    edges = occ_utils.list_edge(face)

    for apnt in sample_points:
        dist, pnt = occ_utils.dist_point_to_edges(apnt, edges)
        dists.append(dist)
        nearest_pnts.append(pnt)

    nearest_pnts = np.asarray(nearest_pnts)
    dists = np.asarray(dists)
    idx = dists > 1.5
    sample_points = sample_points[idx]
    dists = dists[idx]
    nearest_pnts = nearest_pnts[idx]

    return sample_points, dists, nearest_pnts

def sample_point_param(sample_points, dists, nearest_pnts, ray_dir, tri_list):
    while len(sample_points) > 0:
        pnt_idx = random.choice(range(len(sample_points)))

    depths = []
    for pnt in sample_points:
        depth = geom_utils.ray_triangle_set_intersect(pnt, ray_dir, tri_list)
        depths.append(depth)

    depths = np.asarray(depths)
    idx = depths > 2.0
    sample_points = sample_points[idx]
    dists = dists[idx]
    nearest_pnts = nearest_pnts[idx]
    depths = depths[idx]
    idx = random.choice(range(len(sample_points)))
    min_depth = depths[idx]
    max_depth = 10.0
    if feat_type in ['Oring', 'blind_hole', 'triangular_pocket', 'rectangular_pocket',
                     '6sides_pocket', 'circular_end_pocket', 'rectangular_blind_slot',
                     'triangular_blind_step', 'circular_blind_step', 'rectangular_blind_step',
                     'rectangular_through_step', '2sides_through_step', 'slanted_through_step',
                     'v_circular_end_blind_slot', 'h_circular_end_blind_slot']:
        depth = random.uniform(1.0, min_depth - 1.0)
    else:
        depth = max_depth

    return pnt_idx, radius, depth



def filter_points_by_param(sample_points, face, tri_list, sketch_extrema, depth):
    # condition 3: be distant from opposit faces
    normal = occ_utils.normal_to_face_center(face)
    ray_dir = (-np.asarray(occ_utils.list_from_occ(normal))).tolist()
    idx, depth = sample_point_param(sample_points, dists, nearest_pnts, ray_dir, tri_list)
    if idx == None:
        candidate_faces.remove(face)

    return None


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

def sample_points_on_edge(edge):
    sample_pnts = []
    edge_util = OCCUtils.edge.Edge(edge)
    n_pnts = int(edge_util.length() / 1.0) + 1
    if n_pnts < 3:
        return sample_pnts

    pnt_list = edge_util.divide_by_number_of_points(n_pnts)
    if pnt_list == None:
        return sample_pnts

    pnt_list.sort()
    for i in range(1, len(pnt_list) - 1):
        sample_pnts.append(occ_utils.list_from_occ(pnt))

    return sample_pnts

def sample_points_from_face(face, feat_type):
    sample_points = []
    pin_edges = []
    if feat_type in SKETCH_INSIDE_FACE:
        # sketch inside face
        sample_points = sample_points_inside_face(face)
    elif feat_type in SKETCH_ON_EDGE:
        # sketch on one edge
        edges = occ_utils.list_edge(face)
        for edge in edges:
            if occ_utils.is_line(edge):
                edge_pnts = sample_points_on_edge(edge)
                sample_points += edge_pnts
                pin_edges += [edge] * len(edge_pnts)
    elif feat_type in SKETCH_ON_VERTEX:
        # sketch on one vertex
        sample_points = []
    else:
        sample_points = []

    sample_points, dists, nearest_pnts = points_filter_by_dist(sample_points, face)
    return np.asarray(sample_points), dists, nearest_pnts


def search_sketch_face(label_map, feat_type, sketch_extrema, depth):
    candidate_faces = label_map.keys()
    tri_list = triangles_from_faces()

    while len(candidate_faces) > 0:
        face = random.choice(candidate_faces)
        # only create sketches on some feature faces
        if label_map[face] not in [FEAT_NAMES.index('stock')]:
            candidate_faces.remove(face)
            continue

        # only consider planar faces
        if not occ_utils.is_planar(face):
            candidate_faces.remove(face)
            continue

        # sample face
        sample_points = sample_points_from_face(face, feat_type)

        ref_points = filter_points_by_param(sample_points, face, tri_list, sketch_extrema, depth)
        if ref_points == None:
            candidate_faces.remove(face)
            continue

        normal = occ_utils.normal_to_face_center(face)
        return ref_points, normal

    return None, None

BLIND_FEAT = ['Oring', 'blind_hole', 'triangular_pocket', 'rectangular_pocket',
              '6sides_pocket', 'circular_end_pocket', 'rectangular_blind_slot',
              'triangular_blind_step', 'circular_blind_step', 'rectangular_blind_step',
              'rectangular_through_step', '2sides_through_step', 'slanted_through_step',
              'v_circular_end_blind_slot', 'h_circular_end_blind_slot']

def sample_feature_param(ref_extrema, ref_depth):
    feat_type = random.choice(FEAT_NAMES[:9])
    sketch_extrema = random.uniform(1.0, ref_extrema - 1.0)

    if feat_type in BLIND_FEAT:
        depth = random.uniform(1.0, ref_depth - 1.0)
    else:
        depth = ref_depth

    return feat_type, sketch_extrema, depth

def triangulate_shape(shape):
    linear_deflection = 0.1
    angular_deflection = 0.5
    mesh = BRepMesh_IncrementalMesh(shape, linear_deflection, False, angular_deflection, True)
    mesh.Perform()
    assert mesh.IsDone()


def face_filter(face_map, label_list, type_list):
    result = []
    for face in face_map:
        f_label = face_map[face]
        f_type = occ_utils.type_face(face)
        if f_label in label_list and f_type in type_list:
            result.append(face)

    return result

    
def shape_from_machining_feature():
    stock = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape()
    label_map = shape_factory.map_from_name(stock, FEAT_NAMES.index('stock'))
    triangulate_shape(stock)

    num_feats = random.randint(2, 2)
    for i in range(num_feats):
        triangulate_shape(stock)
        # step 1: sample feature arameters [type, width, depth]
        feat_type = random.choice(FEAT_NAMES[16:20])       
        f_list = face_filter(label_map, [FEAT_NAMES.index('stock')], ['plane'])
        bounds = FEAT_BOUND_SAMPLER[feat_type](f_list)
        feat_face = None
        random.shuffle(bounds)
        for bound in bounds:
            # step 3: create feature face            
            feat_face = FACE_GENERATOR[feat_type](bound)
            if feat_face is not None:
                break

        # step 4: apply feature operation
        if feat_face is None:
            continue
        depth = 10.0
        print(i, feat_type)
        stock, label_map = apply_feature(stock, label_map, feat_type, feat_face, depth)

    return stock, label_map, bounds


if __name__ == '__main__':

    OCC_DISPLAY, START_OCC_DISPLAY, ADD_MENU, ADD_FUNCTION_TO_MENU = init_display()
    OCC_DISPLAY.EraseAll()

    colors = []
    rgb_list = np.array(np.meshgrid([0.3, 0.6, 0.9], [0.9, 0.3, 0.6], [0.6, 0.9, 0.3])).T.reshape(-1,3)
    for rgb in rgb_list:
        colors.append(rgb_color(rgb[0], rgb[1], rgb[2]))

    random.seed()
    SHAPE, FMAP, bounds = shape_from_machining_feature()
    for bound in bounds:
        for i in range(4):
            j = (i+1)%4
            pnt1 = occ_utils.as_occ(bound[i], gp_Pnt)
            pnt2 = occ_utils.as_occ(bound[j], gp_Pnt)
            if np.linalg.norm(bound[i] - bound[j]) < 0.000001:
                continue
            seg_maker = GC_MakeSegment(pnt1, pnt2)
            if seg_maker.IsDone():
                edge = BRepBuilderAPI_MakeEdge(seg_maker.Value()).Edge()
                OCC_DISPLAY.DisplayShape(edge)
            else:
                print(bound[i], bound[j])

    AIS = AIS_ColoredShape(SHAPE)
    for a_face in FMAP:
        AIS.SetCustomColor(a_face, colors[FMAP[a_face]])

    OCC_DISPLAY.Context.Display(AIS.GetHandle())
    OCC_DISPLAY.View_Iso()
    OCC_DISPLAY.FitAll()

    START_OCC_DISPLAY()