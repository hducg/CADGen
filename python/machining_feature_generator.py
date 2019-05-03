# _*_ coding: utf_8 _*_
"""
Created on Tue Mar 19 11:20:39 2019

@author: 2624224
"""

import sys
import random
import math
import logging
import os
import pickle
import glob

import numpy as np

from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace
from OCC.Core.BRepFeat import BRepFeat_MakePrism
from OCC.Display.SimpleGui import init_display
from OCC.Display.OCCViewer import rgb_color
from OCC.Core.AIS import AIS_ColoredShape
from OCC.Core.gp import gp_Circ, gp_Ax2, gp_Pnt, gp_Dir
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
import sketch_generator

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

SKETCH_GENERATOR = {'Oring': sketch_generator.face_oring, 'through_hole': sketch_generator.face_circle,
                 'blind_hole': sketch_generator.face_circle, 'triangular_passage': sketch_generator.face_triangle,
                 'rectangular_passage': sketch_generator.face_rectangle, 'circular_through_slot': sketch_generator.face_circle_1,
                 'triangular_through_slot': sketch_generator.face_triangle_1, 'rectangular_through_slot': sketch_generator.face_rect_1,
                 'rectangular_blind_slot': sketch_generator.face_rect_1, 'triangular_pocket': sketch_generator.face_triangle,
                 'rectangular_pocket': sketch_generator.face_rectangle, 'circular_end_pocket': sketch_generator.face_circular_end_rect,
                 'triangular_blind_step': sketch_generator.face_triangle_2, 'circular_blind_step': sketch_generator.face_circle_2,
                 'rectangular_blind_step': sketch_generator.face_rect_2, 'rectangular_through_step': sketch_generator.face_rect_3,
                 '2sides_through_step': sketch_generator.face_pentagon, 'slanted_through_step': sketch_generator.face_quad,
                 'v_circular_end_blind_slot': sketch_generator.face_open_circular_end_rect_v, 'h_circular_end_blind_slot': sketch_generator.face_open_circular_end_rect_h,
                 '6sides_passage': sketch_generator.face_hexagon, '6sides_pocket': sketch_generator.face_hexagon}
                 
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
                 
FEAT_NAMES = ['through_hole', 'triangular_passage', 'rectangular_passage',
              '6sides_passage', 'triangular_through_slot', 'rectangular_through_slot',
              'circular_through_slot', 'rectangular_through_step',
              '2sides_through_step', 'slanted_through_step', # 10 through features
              'Oring', 'blind_hole', 'triangular_pocket', 'rectangular_pocket', 
              '6sides_pocket', 'circular_end_pocket', 'rectangular_blind_slot', 
              'v_circular_end_blind_slot', 'h_circular_end_blind_slot',         #6
              'triangular_blind_step', 'circular_blind_step', 'rectangular_blind_step', #3
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


def add_chamfer(stock, label_map):
    fillet_maker = BRepFilletAPI_MakeChamfer(stock)
    edges = occ_utils.list_edge(stock)
    # to do: select edge
    edge = random.choice(edges)
    topo_exp = TopologyExplorer(stock)
    face = next(topo_exp.faces_from_edge(edge))
    fillet_maker.Add(1.0, edge, face)
   
    shape = fillet_maker.Shape()

    fmap = shape_factory.map_face_before_and_after_feat(stock, fillet_maker)
    label_map = shape_factory.map_from_shape_and_name(fmap, label_map, shape, FEAT_NAMES.index('chamfer'))
            
    return shape, label_map

    
def add_round(stock, label_map):
    fillet_maker = BRepFilletAPI_MakeFillet(stock)
    edges = occ_utils.list_edge(stock)
    # to do: select edge
    radius = random.gauss(0.3, 0.1)
    if radius < 0.1:
        radius = 0.1
    if radius > 0.5:
        radius = 0.5    
    
    topo_exp = TopologyExplorer(stock)    
    for edge in edges:
        is_stock_edge = True
        for face in topo_exp.faces_from_edge(edge):
            if label_map[face] != FEAT_NAMES.index('stock'):
                is_stock_edge = False
        if is_stock_edge:
            fillet_maker.Add(radius, edge)
   
    try:
        shape = fillet_maker.Shape()
    except RuntimeError as error:
        print(error)
        print('round radius', radius)    
        return stock, label_map
    
    fmap = shape_factory.map_face_before_and_after_feat(stock, fillet_maker)
    label_map = shape_factory.map_from_shape_and_name(fmap, label_map, shape, FEAT_NAMES.index('round'))   
    
    return shape, label_map

    
def shape_from_machining_feature():
    stock = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape()
    label_map = shape_factory.map_from_name(stock, FEAT_NAMES.index('stock'))
    triangulate_shape(stock)
    stock, label_map = add_chamfer(stock, label_map)
    
    num_feats = [random.randint(1,2), random.randint(1,3)]
    print(num_feats)
    idx_split = [0, 10, -3]    

    for i in range(2):
        bounds = []
        feat_cnt = 0
        while True:
            triangulate_shape(stock)
            # step 1: sample feature arameters [type, width, depth]
            feat_type = random.choice(FEAT_NAMES[idx_split[i]:idx_split[i + 1]])            
            bounds = SKETCH_BOUND_SAMPLER[feat_type](stock, label_map)
            if len(bounds) < 1:
                break
            
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
            if feat_cnt == num_feats[i]:
                break
    
    stock, label_map = add_round(stock, label_map)

    return stock, label_map


class shape_sampler:
    def __init__(self, shape_path, occ_display):
        self.shape_path = shape_path
        names = [int(name.split(os.sep)[-1].split('.')[0]) for name in glob.glob(shape_path + '*.step')]
        if len(names) < 1:
            names.append(0)        
        names.sort()
        self.shape_cnt = names[-1]
        self.occ_display = occ_display
        random.seed()
        
    def display_shape(self):
        self.occ_display.EraseAll()
        AIS = AIS_ColoredShape(self.shape)
        for a_face in self.fmap:
            AIS.SetCustomColor(a_face, colors[self.fmap[a_face]])
    
        self.occ_display.Context.Display(AIS)
        self.occ_display.View_Iso()
        self.occ_display.FitAll()   
        
    def next_sample(self):
        self.shape, self.fmap = shape_from_machining_feature()
        self.shape_cnt += 1
        self.display_shape()                
        
    def save_shape(self):
        filename = os.path.join(shape_path + str(self.shape_cnt) + '.step')        
        id_map = {}
        fid = 0
        for face in self.fmap:
            id_map[face] = fid
            fid += 1
            
        occ_utils.shape_with_fid_to_step(filename , self.shape, id_map)
        
        filename = filename.replace('step', 'face_truth')
        with open(filename, 'wb') as file:
            pickle.dump(self.fmap, file)
 

    def save_image(self):
        image_name = os.path.join(self.shape_path, str(self.shape_cnt) + '.jpeg')
        self.occ_display.View.Dump(image_name)           

def next_shape():
    asampler.next_sample()

        
def save_shape():
    asampler.save_shape()


def save_image():
    asampler.save_image()


colors = []
rgb_list = np.array(np.meshgrid([0.9, 0.6, 0.3], [0.9, 0.6, 0.3], [0.9, 0.6, 0.3])).T.reshape(-1,3)
for rgb in rgb_list:
    colors.append(rgb_color(rgb[0], rgb[1], rgb[2]))
    
OCC_DISPLAY, START_OCC_DISPLAY, ADD_MENU, ADD_FUNCTION_TO_MENU = init_display()            
shape_path = '../../models/'        
asampler = shape_sampler(shape_path, OCC_DISPLAY)

if __name__ == '__main__':    
    ADD_MENU('menu')
    ADD_FUNCTION_TO_MENU('menu', next_shape)
    ADD_FUNCTION_TO_MENU('menu', save_shape)
    ADD_FUNCTION_TO_MENU('menu', save_image)
            
    START_OCC_DISPLAY()