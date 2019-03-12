# -*- coding: utf-8 -*-
"""
Created on Thu Oct 25 11:43:39 2018

@author: 2624224
"""
import random
import os

from OCC.TopExp import TopExp_Explorer
from OCC.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_EDGE
from OCC.TopoDS import topods
from OCC.Bnd import Bnd_Box
from OCC.BRepMesh import BRepMesh_IncrementalMesh
from OCC.BRepBndLib import brepbndlib_Add
from OCC.BRepTools import breptools_UVBounds
from OCC.IntTools import IntTools_FClass2d
from OCC.gp import gp_Pnt2d
from OCC.BRepAdaptor import BRepAdaptor_Surface
from OCC.BRep import BRep_Tool_Surface
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


def tool_shape_color():
    h_doc = Handle_TDocStd_Document()
    assert(h_doc.IsNull())
    # Create the application
    app = XCAFApp_Application.GetApplication().GetObject()
    app.NewDocument(TCollection_ExtendedString("MDTV-CAF"), h_doc)
    # Get root assembly
    doc = h_doc.GetObject()
    h_shape_tool = XCAFDoc_DocumentTool_ShapeTool(doc.Main())
    l_Colors = XCAFDoc_DocumentTool_ColorTool(doc.Main())
    shape_tool = h_shape_tool.GetObject()
    color_tool = l_Colors.GetObject()
    
    return shape_tool, color_tool
    
'''
input
    shape: TopoDS_Shape
output
    fset: {TopoDS_Face}
'''
def set_face(shape):
    fset = set()
    exp = TopExp_Explorer(shape,TopAbs_FACE)
    while exp.More():
        s = exp.Current()       
        exp.Next()
        face = topods.Face(s) 
        fset.add(face)
    
    return fset


'''
input
    shape: TopoDS_Shape
output
    eset: {TopoDS_Edge}
'''
def set_edge(shape):
    eset = set()
    exp = TopExp_Explorer(shape,TopAbs_EDGE)
    while exp.More():
        s = exp.Current()       
        exp.Next()
        e = topods.Edge(s) 
        eset.add(e)
#        print(face)
    
    return eset


'''
'''
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

    fset = set_face(shape)

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
    fset = set_face(shape)
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
   