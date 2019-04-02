# -*- coding: utf-8 -*-
"""
Created on Thu Oct 25 11:43:39 2018

@author: 2624224
"""
import random
import os
import sys
import math

from OCC.TopExp import TopExp_Explorer
from OCC.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_EDGE, TopAbs_VERTEX
from OCC.TopoDS import topods, TopoDS_Shape, TopoDS_Vertex, TopoDS_Edge, TopoDS_Face
from OCC.Bnd import Bnd_Box
from OCC.BRepMesh import BRepMesh_IncrementalMesh
from OCC.BRepBndLib import brepbndlib_Add
from OCC.BRepTools import breptools_UVBounds
from OCC.IntTools import IntTools_FClass2d
from OCC.gp import gp_Pnt2d, gp_Pnt, gp_Dir, gp_Vec
from OCC.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCC.BRep import BRep_Tool_Surface, BRep_Tool, BRep_Tool_Curve
from OCC.GeomLProp import GeomLProp_SLProps
from OCC.TDocStd import Handle_TDocStd_Document
from OCC.XCAFApp import XCAFApp_Application
from OCC.TCollection import TCollection_ExtendedString
from OCC.XCAFDoc import XCAFDoc_DocumentTool_ShapeTool, XCAFDoc_DocumentTool_ColorTool
from OCC.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCC.TCollection import TCollection_HAsciiString
from OCC.STEPConstruct import stepconstruct_FindEntity
from OCC.StepRepr import Handle_StepRepr_RepresentationItem
from OCC.TopLoc import TopLoc_Location
from OCC.StlAPI import StlAPI_Reader
from OCC.GeomLib import GeomLib_IsPlanarSurface
from OCC.BRepExtrema import BRepExtrema_ExtPC, BRepExtrema_DistShapeShape
from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeVertex


SURFACE_TYPE = ['plane', 'cylinder', 'cone', 'sphere', 'torus', 'bezier', 'bspline', 'revolution', 'extrusion', 'offset', 'other']
CURVE_TYPE = ['line', 'circle', 'ellipse', 'hyperbola', 'parabola', 'bezier', 'bspline', 'offset', 'other']

import OCCUtils.edge
import OCCUtils.face
import OCCUtils.Topology


def list_face(shape):
    '''
    input
        shape: TopoDS_Shape
    output
        fset: {TopoDS_Face}
    '''
    fset = set()
    exp = TopExp_Explorer(shape,TopAbs_FACE)
    while exp.More():
        s = exp.Current()
        exp.Next()
        face = topods.Face(s)
        fset.add(face)

    return list(fset)


def list_edge(shape):
    '''
    input
        shape: TopoDS_Shape
    output
        eset: {TopoDS_Edge}
    '''
    eset = set()
    exp = TopExp_Explorer(shape,TopAbs_EDGE)
    while exp.More():
        s = exp.Current()
        exp.Next()
        e = topods.Edge(s)
        eset.add(e)
#        print(face)

    return list(eset)


def edges_at_vertex(vert, face):
    f_topo = OCCUtils.Topology.Topo(face)
    v_edges = [edge for edge in f_topo.edges_from_vertex(vert)]
    return v_edges
     
def list_verts_ordered(face):
    f_util = OCCUtils.face.Face(face)
    w_util = OCCUtils.Topology.WireExplorer(next(f_util.topo.wires()))
    verts = [vert for vert in w_util.ordered_vertices()]
    return verts
    

def as_list(occ_obj):
    if type(occ_obj) not in [TopoDS_Vertex, gp_Pnt, gp_Dir, gp_Vec]:
        return None
        
    if type(occ_obj) is TopoDS_Vertex:
        occ_obj = BRep_Tool.Pnt(occ_obj)
    
    return list(occ_obj.Coord())
    
    
def as_occ(pnt, occ_type):
    if occ_type not in [TopoDS_Vertex, gp_Pnt, gp_Dir, gp_Vec]:
        return None
     
    if occ_type is TopoDS_Vertex:
        return BRepBuilderAPI_MakeVertex(gp_Pnt(pnt[0], pnt[1], pnt[2])).Vertex()
    else:
        return occ_type(pnt[0], pnt[1], pnt[2])
    
    
def type_face(face):
    if type(face) is not TopoDS_Face:
        print(face, 'not face')
        return None
        
    surf_adaptor = BRepAdaptor_Surface(face)        
    return SURFACE_TYPE[surf_adaptor.GetType()]

    
def type_edge(the_edge):
    if type(the_edge) is not TopoDS_Edge:
        return None
        
    curve_adaptor = BRepAdaptor_Curve(the_edge)    
    return CURVE_TYPE[curve_adaptor.GetType()]

    
def get_boundingbox(shape, tol=1e-6, use_mesh=True):
    """ return the bounding box of the TopoDS_Shape `shape`
    Parameters
    ----------
    shape : TopoDS_Shape or a subclass such as TopoDS_Face
        the shape to compute the bounding box from
    tol: float
        tolerance of the computed boundingbox
    use_mesh : bool
        a flag that tells whether or not the shape has first to be meshed before the bbox
        computation. This produces more accurate results
    """
    bbox = Bnd_Box()
    bbox.SetGap(tol)
    if use_mesh:
        mesh = BRepMesh_IncrementalMesh()
        mesh.SetParallel(True)
        mesh.SetShape(shape)
        mesh.Perform()
        assert mesh.IsDone()
    brepbndlib_Add(shape, bbox, use_mesh)

    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    return xmin, ymin, zmin, xmax, ymax, zmax, xmax-xmin, ymax-ymin, zmax-zmin


'''
input
    face: TopoDS_Face
output
    P: gp_Pnt
    D: gp_Dir
'''
def sample_point(face):
    #    randomly choose a point from F
    u_min, u_max, v_min, v_max = breptools_UVBounds(face)
    u = random.uniform(u_min, u_max)
    v = random.uniform(v_min, v_max)

    itool = IntTools_FClass2d(face, 1e-6)
    while itool.Perform(gp_Pnt2d(u,v)) != 0:
        print('outside')
        u = random.uniform(u_min, u_max)
        v = random.uniform(v_min, v_max)

    P = BRepAdaptor_Surface(face).Value(u, v)

#   the normal
    surf = BRep_Tool_Surface(face)
    D = GeomLProp_SLProps(surf,u,v,1,0.01).Normal()
    if face.Orientation() == TopAbs_REVERSED:
        D.Reverse()

    return P, D

def shape_with_fid_to_step(filename, shape, id_map):
    '''
    input
        filename
        shape
        id_map： {TopoDS_Face: int}
    output
    '''
#    print('shape_with_fid_to_step')
#    fset = set_face(shape)
    writer = STEPControl_Writer()
    writer.Transfer(shape, STEPControl_AsIs)

    finderp = writer.WS().GetObject().TransferWriter().GetObject().FinderProcess()

    fset = list_face(shape)

    loc = TopLoc_Location()
    for face in fset:
        item = stepconstruct_FindEntity(finderp, face, loc)
        if item.IsNull():
            print(face)
            continue
        item.GetObject().SetName(TCollection_HAsciiString(str(id_map[face])).GetHandle())

    writer.Write(filename)


def shape_with_fid_from_step(filename):
    '''
    input
    output
        shape:      TopoDS_Shape
        id_map:  {TopoDS_Face: int}
    '''
#    print('shape_with_fid_from_step')
    if not os.path.exists(filename):
        print(filename, ' not exists')
        return

    reader = STEPControl_Reader()
    reader.ReadFile(filename)
    reader.TransferRoots()
    shape = reader.OneShape()

    treader = reader.WS().GetObject().TransferReader().GetObject()

    id_map = {}
    fset = list_face(shape)
    # read the face names
    for face in fset:
        item = treader.EntityFromShapeResult(face, 1)
        if item.IsNull():
            print(face)
            continue
        item = Handle_StepRepr_RepresentationItem.DownCast(item).GetObject()
        name = item.Name().GetObject().ToCString()
        if name:
            nameid = int(name)
            id_map[face] = nameid

    return shape, id_map


def shape_from_stl(filename):
    if not os.path.isfile(filename):
        raise FileNotFoundError("%s not found." % filename)

    stl_reader = StlAPI_Reader()
    the_shape = TopoDS_Shape()
    stl_reader.Read(the_shape, filename)

    if the_shape.IsNull():
        raise AssertionError("Shape is null.")

    return the_shape

def normal_to_face_center(face):
    u_min, u_max, v_min, v_max = breptools_UVBounds(face)
    u_mid = (u_min + u_max) / 2.
    v_mid = (v_min + v_max) / 2.

    surf = BRep_Tool_Surface(face)
    normal = GeomLProp_SLProps(surf,u_mid, v_mid,1,0.01).Normal()  
    if face.Orientation() == TopAbs_REVERSED:
        normal.Reverse()
    
    return normal


       
def points_from_edge(edge):
    vset = []
    exp = TopExp_Explorer(edge, TopAbs_VERTEX)
    while exp.More():
        s = exp.Current()
        exp.Next()
        vert = topods.Vertex(s)
        vset.append(as_list(vert))

    return list(vset)
        
def dist_point_to_edge(pnt, edge):    
    assert pnt is not None
    
    vert_maker = BRepBuilderAPI_MakeVertex(gp_Pnt(pnt[0], pnt[1], pnt[2]))
    dss = BRepExtrema_DistShapeShape(vert_maker.Vertex(), edge)
    if not dss.IsDone():
        print('BRepExtrema_ExtPC not done')
        return None, None

    if dss.NbSolution() < 1:
        print('no nearest points found')
        return None, None
    return dss.Value(), as_list(dss.PointOnShape2(1))

    
def dist_point_to_edges(the_pnt, edges):
    min_d = sys.float_info.max
    nearest_pnt = None
    for edge in edges:
        dist, pnt = dist_point_to_edge(the_pnt, edge)
        if dist is None:
            continue
        if dist < min_d:
            min_d = dist
            nearest_pnt = pnt     
    return min_d, nearest_pnt
    
'''
input
    shape:          TopoDS_Shape
output
    pts:            [[float,float,float]]
    uvs:            [[float,float]]
    triangles:   [[int,int,int]]
    triangle_faces: [TopoDS_Face]    
'''    
def triangulation_from_shape(shape):
    linear_deflection = 0.1
    angular_deflection = 0.5
    mesh = BRepMesh_IncrementalMesh(shape, linear_deflection, False, angular_deflection, True)
    mesh.Perform()
    assert mesh.IsDone()
    
    pts = []
    uvs = []
    triangles = []
    triangle_faces = []
    faces = list_face(shape)
    offset = 0
    for f in faces:
        aLoc = TopLoc_Location()
        aTriangulation = BRep_Tool().Triangulation(f, aLoc).GetObject()
        aTrsf = aLoc.Transformation()
        aOrient = f.Orientation()

        aNodes = aTriangulation.Nodes()
        aUVNodes = aTriangulation.UVNodes()
        aTriangles = aTriangulation.Triangles()
        
        for i in range(1, aTriangulation.NbNodes() + 1):
            pt = aNodes.Value(i)
            pt.Transform(aTrsf)
            pts.append([pt.X(),pt.Y(),pt.Z()])
            uv = aUVNodes.Value(i)
            uvs.append([uv.X(),uv.Y()])
        
        for i in range(1, aTriangulation.NbTriangles() + 1):
            n1, n2, n3 = aTriangles.Value(i).Get()
            n1 -= 1
            n2 -= 1
            n3 -= 1
            if aOrient == TopAbs_REVERSED:
                tmp = n1
                n1 = n2
                n2 = tmp
            n1 += offset
            n2 += offset
            n3 += offset
            triangles.append([n1, n2, n3])
            triangle_faces.append(f)
        offset += aTriangulation.NbNodes()

    return pts, uvs, triangles, triangle_faces

if __name__ == '__main__':
    print('occ_utils')
     