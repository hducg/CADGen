# _*_ coding: utf_8 _*_
"""
Created on Tue Mar 19 11:20:39 2019

@author: 2624224
"""

import random
import logging
import numpy as np

from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace
from OCC.Core.BRepFeat import BRepFeat_MakePrism
from OCC.Display.SimpleGui import init_display
from OCC.Display.OCCViewer import rgb_color
from OCC.Core.AIS import AIS_ColoredShape
from OCC.Core.gp import gp_Circ, gp_Ax2, gp_Pnt, gp_Dir, gp_Vec
from OCC.Core.TopoDS import TopoDS_Face
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.GC import GC_MakeSegment, GC_MakeArcOfCircle
from OCC.Core.Geom import Geom_Circle
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.TopoDS import TopoDS_Vertex
from OCC.Core.TopAbs import TopAbs_VERTEX, TopAbs_REVERSED
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.BRepFilletAPI import BRepFilletAPI_MakeFillet, BRepFilletAPI_MakeChamfer
from OCC.Extend.TopologyUtils import TopologyExplorer

import occ_utils
import shape_factory
import geom_utils

import OCCUtils.edge
import OCCUtils.face
import sketch

MAX_TRY = 6
CLEARANCE = 1.0
LEN_MIN = 2.0

THE_BOUND = None
THE_POINTS = []

def triangulation_from_face(face):
    aLoc = TopLoc_Location()
    aTriangulation = BRep_Tool().Triangulation(face, aLoc)
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
# sketch bound sampler
#==============================================================================
def face_filter(stock, label_map, num_edge):
    result = []
    faces = occ_utils.list_face(stock)
    for face in faces:
        try:
            if label_map[face] != FEAT_NAMES.index('stock'):
                continue
        except KeyError:
            print('keyError')
            print(face)
            print(faces)
            return []
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
                try:
                    if label_map[face_adjacent] != FEAT_NAMES.index('stock'):
                        good_edge.append(False)
                        continue
                except KeyError:
                    print('KeyError')
                    print(face_adjacent)
                    print(faces)
                    return []

                good_edge.append(True)

            for i in range(len(edges)):
                j = (i + 1) % len(edges)
                k = (i + 2) % len(edges)

                if not good_edge[i]:
                    continue

                if num_edge == 1:
                    result.append([face, edges[i]])
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
            e_dir = pt1 - pt2
            e_len = np.linalg.norm(e_dir)
            if e_len < LEN_MIN + 2 * CLEARANCE:
                continue

            sample_points.append((0.5 * pt1 + 0.5 * pt2).tolist())

    for tri in triangles:
        pt1 = np.asarray(tri_pnts[tri[0]])
        pt2 = np.asarray(tri_pnts[tri[1]])
        pt3 = np.asarray(tri_pnts[tri[2]])
        if geom_utils.outer_radius_triangle(pt1, pt2, pt3) < LEN_MIN + 2 * CLEARANCE:
            continue

        sample_points.append(((pt1 + pt2 + pt3) / 3).tolist())
        sample_points.append((0.2 * pt1 + 0.4 * pt2 + 0.4 * pt3).tolist())
        sample_points.append((0.4 * pt1 + 0.2 * pt2 + 0.4 * pt3).tolist())
        sample_points.append((0.4 * pt1 + 0.4 * pt2 + 0.2 * pt3).tolist())

    return sample_points


def rect_size(rect):
    dir_w = rect[2] - rect[1]
    dir_h = rect[0] - rect[1]
    width = np.linalg.norm(dir_w)
    height = np.linalg.norm(dir_h)

    return width, height


def bound_inner(shape, label_map):
    global THE_POINTS
    fe_list = face_filter(shape, label_map, 0)
    bounds = []
    for face in fe_list:
        normal = np.array(occ_utils.as_list(occ_utils.normal_to_face_center(face)))
        sample_pnts = np.array(sample_points_inside_face(face))
        edges = occ_utils.list_edge(face)
        for apnt in sample_pnts:
            THE_POINTS.append(apnt)
            dist, pnt = occ_utils.dist_point_to_edges(apnt, edges)            
            if dist >= LEN_MIN / 2 + CLEARANCE:
                dir_w = apnt - np.array(pnt)
                dir_w = dir_w / np.linalg.norm(dir_w)
                dir_h = np.cross(dir_w, normal)

                pnt0 = apnt - dir_w * (dist - CLEARANCE) - dir_h * dist
                pnt1 = apnt - dir_w * (dist - CLEARANCE) + dir_h * dist
                pnt2 = apnt + dir_w * dist + dir_h * dist
                pnt3 = apnt + dir_w * dist - dir_h * dist
                bounds.append([pnt0, pnt1, pnt2, pnt3, -normal])

    return bounds


def shifter_inner(bound):
    dir_w = bound[2] - bound[1]
    dir_h = bound[0] - bound[1]
    old_w = np.linalg.norm(dir_w)
    old_h = np.linalg.norm(dir_h)
    dir_w = dir_w / old_w
    dir_h = dir_h / old_h

    scale_w = random.uniform(0.1, 1.0)
    scale_h = random.uniform(0.1, 1.0)
    new_w = max(LEN_MIN, old_w * scale_w)
    new_h = max(LEN_MIN, old_h * scale_h)

    offset_w = random.uniform(0.0, old_w - new_w)
    offset_h = random.uniform(0.0, old_h - new_h)

    bound[1] = bound[1] + offset_w * dir_w + offset_h * dir_h
    bound[0] = bound[1] + new_h * dir_h
    bound[2] = bound[1] + new_w * dir_w
    bound[3] = bound[0] + new_w * dir_w

    return bound


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

        pnt1 = np.array(occ_utils.as_list(topexp.FirstVertex(edge, True)))
        pnt2 = np.array(occ_utils.as_list(topexp.LastVertex(edge, True)))

        edge_dir = pnt2 - pnt1
        edge_len = np.linalg.norm(edge_dir)
        if edge_len < 4.0:
            continue

        edge_dir = edge_dir / edge_len
        pnt1 = pnt1 + edge_dir
        pnt2 = pnt2 - edge_dir
        edge_len -= 2.0

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
                w, h = rect_size(bound)
                if w >= LEN_MIN + 2 * CLEARANCE and h >= LEN_MIN + CLEARANCE:
                    bound = shrink_bound_1(bound)
                    bound = np.append(bound, [-normal], axis=0)
                    bounds.append(bound)

    return bounds


def shifter_1(bound):
    dir_w = bound[2] - bound[1]
    dir_h = bound[0] - bound[1]
    old_w = np.linalg.norm(dir_w)
    old_h = np.linalg.norm(dir_h)
    dir_w = dir_w / old_w
    dir_h = dir_h / old_h

    scale_w = random.uniform(0.1, 1.0)
    scale_h = random.uniform(0.1, 1.0)
    new_w = max(LEN_MIN, old_w * scale_w)
    new_h = max(LEN_MIN, old_h * scale_h)

    offset_w = random.uniform(0.0, old_w - new_w)

    bound[1] = bound[1] + offset_w * dir_w
    bound[0] = bound[1] + new_h * dir_h
    bound[2] = bound[1] + new_w * dir_w
    bound[3] = bound[0] + new_w * dir_w

    return bound


def shrink_bound_1(bound):
    dir_w = bound[2] - bound[1]
    dir_h = bound[0] - bound[1]
    old_w = np.linalg.norm(dir_w)
    old_h = np.linalg.norm(dir_h)
    dir_w = CLEARANCE * dir_w / old_w
    dir_h = CLEARANCE * dir_h / old_h

    bound[0] = bound[0] - dir_h
    bound[3] = bound[3] - dir_h

    return bound

def shrink_bound_2(bound):
    dir_w = bound[2] - bound[1]
    dir_h = bound[0] - bound[1]
    old_w = np.linalg.norm(dir_w)
    old_h = np.linalg.norm(dir_h)
    dir_w = CLEARANCE * dir_w / old_w
    dir_h = CLEARANCE * dir_h / old_h

    bound[0] = bound[0] - dir_h
    bound[2] = bound[2] - dir_w
    bound[3] = bound[3] - dir_h - dir_w

    return bound

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
            w, h = rect_size(bound)
            if w >= LEN_MIN + CLEARANCE and h >= LEN_MIN + CLEARANCE:
                normal = np.array(occ_utils.as_list(occ_utils.normal_to_face_center(face)))
                bound = np.append(bound, [-normal], axis=0)
                shrink_bound_2(bound)
                bounds.append(bound)

    return bounds


def shifter_2(bound):
    dir_w = bound[2] - bound[1]
    dir_h = bound[0] - bound[1]
    old_w = np.linalg.norm(dir_w)
    old_h = np.linalg.norm(dir_h)
    dir_w = dir_w / old_w
    dir_h = dir_h / old_h

    scale_w = random.uniform(0.1, 1.0)
    scale_h = random.uniform(0.1, 1.0)
    new_w = max(LEN_MIN, old_w * scale_w)
    new_h = max(LEN_MIN, old_h * scale_h)

    bound[0] = bound[1] + new_h * dir_h
    bound[2] = bound[1] + new_w * dir_w
    bound[3] = bound[0] + new_w * dir_w

    return bound


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
            w, h = rect_size(bound)
            if w >= LEN_MIN and h >= LEN_MIN + CLEARANCE:
                normal = np.array(occ_utils.as_list(occ_utils.normal_to_face_center(face)))
                bound = shrink_bound_1(bound)
                bound = np.append(bound, [-normal], axis=0)
                bounds.append(bound)

    return bounds


def shifter_3(bound):
    dir_h = bound[0] - bound[1]
    old_h = np.linalg.norm(dir_h)
    dir_h = dir_h / old_h

    scale_h = random.uniform(0.1, 1.0)
    new_h = max(LEN_MIN, old_h * scale_h)

    bound[0] = bound[1] + new_h * dir_h
    bound[3] = bound[2] + new_h * dir_h

    return bound
#==============================================================================
#     feature depth sampler
#==============================================================================
def depth_min_max(bound, triangles, thres):
    points = geom_utils.points_inside_rect(bound[0], bound[1], bound[2], bound[3], 0.2)
    global THE_POINTS
    THE_POINTS = points
    depths = []
    for pnt in points:
        dpt = geom_utils.ray_triangle_set_intersect(pnt, bound[4], triangles)
        if dpt < 0.0:
            continue
        if dpt + 1e-6 < thres:
            return float('-inf'), float('-inf')
        depths.append(dpt)

    depths.sort()
    return min(depths), max(depths)


def depth_blind(bound, triangles):
    d_min, _ = depth_min_max(bound, triangles, LEN_MIN + CLEARANCE)
    if d_min < 0:
        return float('-inf')

    return random.uniform(LEN_MIN, d_min - CLEARANCE)


def depth_through_slot(bound, triangles):
    d_min, _ = depth_min_max(bound, triangles, 10.0)
    if d_min < 0:
#        logging.warning('no valid through slot depth')
        return float('-inf')
    return 10.0


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

BOUND_SHIFTER = {'Oring': shifter_inner, 'through_hole': shifter_inner,
                      'blind_hole': shifter_inner, 'triangular_passage': shifter_inner,
                      'rectangular_passage': shifter_inner, 'circular_through_slot': shifter_1,
                      'triangular_through_slot': shifter_1, 'rectangular_through_slot': shifter_1,
                      'rectangular_blind_slot': shifter_1, 'triangular_pocket': shifter_inner,
                      'rectangular_pocket': shifter_inner, 'circular_end_pocket': shifter_inner,
                      'triangular_blind_step': shifter_2, 'circular_blind_step': shifter_2,
                      'rectangular_blind_step': shifter_2, 'rectangular_through_step': shifter_3,
                      '2sides_through_step': shifter_3, 'slanted_through_step': shifter_3,
                      'v_circular_end_blind_slot': shifter_1, 'h_circular_end_blind_slot': shifter_1,
                      '6sides_passage': shifter_inner, '6sides_pocket': shifter_inner}

SKETCH_GENERATOR = {'Oring': sketch.face_oring, 'through_hole': sketch.face_circle,
                 'blind_hole': sketch.face_circle, 'triangular_passage': sketch.face_triangle,
                 'rectangular_passage': sketch.face_rect, 'circular_through_slot': sketch.face_circle_1,
                 'triangular_through_slot': sketch.face_triangle_1, 'rectangular_through_slot': sketch.face_rect,
                 'rectangular_blind_slot': sketch.face_rect, 'triangular_pocket': sketch.face_triangle,
                 'rectangular_pocket': sketch.face_rect, 'circular_end_pocket': sketch.face_circular_end_rect,
                 'triangular_blind_step': sketch.face_triangle_2, 'circular_blind_step': sketch.face_circle_2,
                 'rectangular_blind_step': sketch.face_rect, 'rectangular_through_step': sketch.face_rect,
                 '2sides_through_step': sketch.face_pentagon, 'slanted_through_step': sketch.face_quad,
                 'v_circular_end_blind_slot': sketch.face_open_circular_end_rect_v, 
                 'h_circular_end_blind_slot': sketch.face_open_circular_end_rect_h,
                 '6sides_passage': sketch.face_hexagon, '6sides_pocket': sketch.face_hexagon}

FEAT_DEPTH_SAMPLER = {'Oring': depth_blind, 'through_hole': depth_through,
                      'blind_hole': depth_blind, 'triangular_passage': depth_through,
                      'rectangular_passage': depth_through, 'circular_through_slot': depth_through_slot,
                      'triangular_through_slot': depth_through_slot, 'rectangular_through_slot': depth_through_slot,
                      'rectangular_blind_slot': depth_blind, 'triangular_pocket': depth_blind,
                      'rectangular_pocket': depth_blind, 'circular_end_pocket': depth_blind,
                      'triangular_blind_step': depth_blind, 'circular_blind_step': depth_blind,
                      'rectangular_blind_step': depth_blind, 'rectangular_through_step': depth_blind,
                      '2sides_through_step': depth_blind, 'slanted_through_step': depth_blind,
                      'v_circular_end_blind_slot': depth_blind, 'h_circular_end_blind_slot': depth_blind,
                      '6sides_passage': depth_through, '6sides_pocket': depth_blind}

FEAT_NAMES = ['chamfer', 'through_hole', 'triangular_passage', 'rectangular_passage', # 3
              '6sides_passage', 'triangular_through_slot', 'rectangular_through_slot', # 6
              'circular_through_slot', 'rectangular_through_step',
              '2sides_through_step', 'slanted_through_step', # 10 through features
              'Oring', 'blind_hole', 'triangular_pocket', 'rectangular_pocket',
              '6sides_pocket', 'circular_end_pocket', 'rectangular_blind_slot',
              'v_circular_end_blind_slot', 'h_circular_end_blind_slot',         #6
              'triangular_blind_step', 'circular_blind_step', 'rectangular_blind_step', #3
              'round', 'stock']


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


def add_chamfer(stock, label_map):
    fillet_maker = BRepFilletAPI_MakeChamfer(stock)
    edges = occ_utils.list_edge(stock)
    # to do: select edge
    topo_exp = TopologyExplorer(stock)
    while True:
        edge = random.choice(edges)
        is_stock_edge = True
        for face in topo_exp.faces_from_edge(edge):
            if label_map[face] != FEAT_NAMES.index('stock'):
                is_stock_edge = False
        if is_stock_edge:
            break

    topo_exp = TopologyExplorer(stock)
    face = next(topo_exp.faces_from_edge(edge))
    depth = random.uniform(1.0, 4.0)
    fillet_maker.Add(depth, edge, face)

    shape = fillet_maker.Shape()

    fmap = shape_factory.map_face_before_and_after_feat(stock, fillet_maker)
    label_map = shape_factory.map_from_shape_and_name(fmap, label_map, shape, FEAT_NAMES.index('chamfer'))

    return shape, label_map


def add_round(stock, label_map):
    fillet_maker = BRepFilletAPI_MakeFillet(stock)
    edges = occ_utils.list_edge(stock)

    # to do: select edge
    radius = random.uniform(0.5, 1.0)

    try_cnt = 0
    while try_cnt < len(edges):
        edge = random.choice(edges)
        e_util = OCCUtils.edge.Edge(edge)

        if e_util.length() < LEN_MIN:
            continue
        fillet_maker.Add(radius, edge)
        try:
            shape = fillet_maker.Shape()
        except RuntimeError:
            shape = stock
            try_cnt += 1
            continue

        fmap = shape_factory.map_face_before_and_after_feat(stock, fillet_maker)
        label_map = shape_factory.map_from_shape_and_name(fmap, label_map, shape, FEAT_NAMES.index('round'))
        break

    return shape, label_map


def add_sketch(stock, label_map, feat_type):
    # 1. max bounds
#    OCC_DISPLAY.DisplayShape(stock)
    global THE_BOUND
#    logging.info('1. sketch bound')
    bounds = SKETCH_BOUND_SAMPLER[feat_type](stock, label_map)
    if len(bounds) < 1:
#        logging.warning('no sketch plane found')
        return stock, label_map

    feat_face = None
    faces = occ_utils.list_face(stock)
    triangles = triangles_from_faces(faces)
    random.shuffle(bounds)
    depth = float('-inf')
#        display_segments([bound_max[0], bound_max[1], bound_max[2], bound_max[3], bound_max[0]])
#    logging.info('2. feature bound')
    try_cnt = 0
    while try_cnt < len(bounds):
        bound_max = random.choice(bounds)
        # 2. sample bound
        bound = BOUND_SHIFTER[feat_type](bound_max)
        THE_BOUND = bound

        # 4. sample depth
        depth = FEAT_DEPTH_SAMPLER[feat_type](bound, triangles)
        if depth <= 0:
           try_cnt += 1
           continue

        # 5. create sketch
#        logging.info('3. generate sketch')
        feat_face = SKETCH_GENERATOR[feat_type](bound)
        try_cnt = len(bounds)

    # 6. apply feature
    if feat_face is None:
        if try_cnt == len(bounds):
            bw = np.linalg.norm(bound[2] - bound[1])
            bh = np.linalg.norm(bound[0] - bound[1])
            print('feature bound failed', bw, bh)
        else:
            print('failed create sketch')
        return stock, label_map

#    logging.info('4. apply feature', depth)
    stock, label_map = apply_feature(stock, label_map, feat_type, feat_face, bound[4] * depth)
    return stock, label_map


def shape_from_directive(combo):
    global THE_POINTS
    try_cnt = 0
    while True:
        stock = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape()
        label_map = shape_factory.map_from_name(stock, FEAT_NAMES.index('stock'))

        for fid in combo:
#            logging.info('')
#            logging.info(FEAT_NAMES[fid])
            THE_POINTS = []
            triangulate_shape(stock)
            if fid == FEAT_NAMES.index('chamfer'):
                stock, label_map = add_chamfer(stock, label_map)
            elif fid == FEAT_NAMES.index('round'):
                stock, label_map = add_round(stock, label_map)
            else:
                stock, label_map = add_sketch(stock, label_map, FEAT_NAMES[fid])

        if stock is not None:
            break

        try_cnt += 1
#        logging.warning(combo, 'tried', try_cnt, 'times')
        if try_cnt > len(combo):
#            logging.warning('tried too many times, no solution found')
            stock = None
            label_map = None
            break

    return stock, label_map


#OCC_DISPLAY, START_OCC_DISPLAY, ADD_MENU, ADD_FUNCTION_TO_MENU = init_display()
#if __name__ == '__main__':
#    shape_path = '../../models/'
#    combo = (8,11,11,12)
#    print(combo)
#    shape, label_map = shape_from_directive(combo)
#    asampler = viewer.ShapeViewer(shape_path, OCC_DISPLAY)
#    asampler.load_shape(shape, label_map)
#    asampler.display_shape()
#    viewer.SegmentsViewer([THE_BOUND[0], THE_BOUND[1], THE_BOUND[2], THE_BOUND[3], THE_BOUND[0]], OCC_DISPLAY)
#    viewer.PointsViewer(THE_POINTS, OCC_DISPLAY)
##    if shape is not None:
##        occ_utils.labeled_shape_to_file(shape, label_map, shape_path, '0-18-1')
#    print('***done***')
#    START_OCC_DISPLAY()