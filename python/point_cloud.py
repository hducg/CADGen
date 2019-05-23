# -*- coding: utf-8 -*-
"""
Created on Fri Sep 28 15:37:43 2018

@author: 2624224
"""

##Copyright 2010-2017 Thomas Paviot (tpaviot@gmail.com)
##
##This file is part of pythonOCC.
##
##pythonOCC is free software: you can redistribute it and/or modify
##it under the terms of the GNU Lesser General Public License as published by
##the Free Software Foundation, either version 3 of the License, or
##(at your option) any later version.
##
##pythonOCC is distributed in the hope that it will be useful,
##but WITHOUT ANY WARRANTY; without even the implied warranty of
##MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##GNU Lesser General Public License for more details.
##
##You should have received a copy of the GNU Lesser General Public License
##along with pythonOCC.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function

import numpy as np

##########################
# OCC library
##########################
from OCC.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCC.BRepMesh import BRepMesh_IncrementalMesh
from OCC.BRep import BRep_Tool, BRep_Tool_Surface
from OCC.TopLoc import TopLoc_Location
from OCC.GeomLProp import GeomLProp_SLProps
from OCC.TopAbs import TopAbs_REVERSED

#==============================================================================
# local library
#==============================================================================
import occ_utils

#logging.basicConfig(level=logging.INFO)
#
#logger = logging.getLogger(__name__)

#from OCCUtils.edge import Edge
#def points_from_edge_interior(edge, resolution):
#    # compute edge length and number of sample points
#    edge_len = GCPnts_AbscissaPoint().Length(BRepAdaptor_Curve(edge))
#    N = int(edge_len / resolution)
#    edge_util = Edge(edge)
#    edge_util.divide_by_number_of_points(N)
    #



def uvs_from_interpolation(uv1, uv2, uv3, Nv, Nh):
    '''
    input
        uv1:    [float,float]
        uv2:    [float,float]
        uv3:    [float,float]
        N1:     int
        N2:     int
    output
        uvs:    [[float,float]]
    '''
    uvs = []
    for i in range(Nv + 1):#[0, Nv]
        t1 = i / Nv#[0, 1]
        uv12 = np.multiply(uv2, t1) + np.multiply(uv1, 1 - t1)#[uv1, uv2]
        uv13 = np.multiply(uv3, t1) + np.multiply(uv1, 1 - t1)#[uv1, uv3]
        N = int(Nh * t1) #[1, N2 + 1]
        if N == 0:
            N = 1
        for j in range(N + 1):#[0, 1] [0, N2]
            t2 = j / N #[0, 1]
            uv = np.multiply(uv12, t2) + np.multiply(uv13, 1 - t2)#uv12 * t2 + uv13 * (1 - t2)
            uvs.append(uv)

    return uvs



def points_sample_from_triangle(t):
    '''
    input
        t:          ([[float,float,float],[float,float,float],[float,float,float]],
                      [[float,float],[float,float],[float,float]],
                      TopoDS_Face,
                      float)
    output
        samples:        [[float,float,float]]
        normals:    [[float,float,float]]
        f:          TopoDS_Face
    '''
    samples = []
    normals = []

    p = np.array([np.array(t[0][0]), np.array(t[0][1]), np.array(t[0][2])])
    l = np.array([np.sum(np.square(p[1] - p[2])), np.sum(np.square(p[0] - p[2])), np.sum(np.square(p[0] - p[1]))])
    li = np.argsort(l)
    l = l[li]

    resolution = t[3]
    Nv = int(np.sqrt(l[2]) / resolution)
    if Nv == 0:
        Nv = 1
    Nh = int(np.sqrt(l[0]) / resolution)
    if Nh == 0:
        Nh =1

    uv0 = t[1][li[0]]
    uv1 = t[1][li[1]]
    uv2 = t[1][li[2]]

    uvs = uvs_from_interpolation(uv0,uv1,uv2,Nv,Nh)

    f = t[2]
    surf = BRep_Tool_Surface(f)
    for uv in uvs:
        pt = BRepAdaptor_Surface(f).Value(uv[0],uv[1])
        samples.append([pt.X(),pt.Y(),pt.Z()])
        #   the normal
        D = GeomLProp_SLProps(surf,uv[0],uv[1],1,0.01).Normal()
        if f.Orientation() == TopAbs_REVERSED:
            D.Reverse()
        normals.append([D.X(),D.Y(),D.Z()])

    return (samples, normals, f)



def triangles_from_shape(shape, resolution):
    '''
    input
        shape: TopoDS_Shape
        resolution: float
    output
        traingles: [([],[],TopoDS_Face,float)]
    '''
    tri_pts, tri_uvs, tri_facets, tri_faces = occ_utils.triangulation_from_shape(shape)
#    print('number of traingulation nodes:', len(tri_pts))
    triangles = []
    for i in range(len(tri_facets)):
        t_idx = tri_facets[i]
        pts = []
        uv= []
        for idx in t_idx:
            pts.append(tri_pts[idx])
            uv.append(tri_uvs[idx])
        f= tri_faces[i]
        triangles.append((pts,uv,f,resolution))

    return triangles



def point_cloud_from_labeled_shape(shape, label_map, id_map, resolution=0.1):
    '''
    input
        shape:          TopoDS_Shape
        label_map:      {TopoDS_Face: int}
        id_map:         {TopoDS_Face: int}
        resolution: float
    output
        cloud_pts:      [[float,float,float]]
        cloud_normals:  [[float,float,float]]
        cloud_segs:     [int]
        face_ids:       [int]
    '''
    #    print('point_cloud_from_labeled_shape')
    cloud_pts = []
    cloud_normals = []
    cloud_feats = []
    cloud_segs = []
    face_ids = []
    triangles = triangles_from_shape(shape, resolution)
    print(len(triangles), 'triangles')
    for t in triangles:
        r = points_sample_from_triangle(t)
        cloud_pts += r[0]
        cloud_normals += r[1]
        cloud_segs += [label_map[r[2]]] * len(r[0])
        face_ids += [id_map[r[2]]] * len(r[0])
    print(len(cloud_pts), 'points')
    return cloud_pts, cloud_normals, cloud_feats, cloud_segs, face_ids


def resolution_from_shape(shape):
    '''
    input
        shape:          TopoDS_Shape
    output
        resolution:     float
    '''
    xmin, ymin, zmin, xmax, ymax, zmax, xlen, ylen, zlen = occ_utils.get_boundingbox(shape, use_mesh = False)
    resolution = max(xlen, ylen, zlen) / 128

    return resolution

