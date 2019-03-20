# -*- coding: utf-8 -*-
"""
Created on Mon Oct 22 15:57:43 2018

@author: 2624224
"""

from math import pi
import random

from OCC.BRepBuilderAPI import (BRepBuilderAPI_Transform, BRepBuilderAPI_MakeWire,
                                BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace)
from OCC.BRepFeat import BRepFeat_MakePrism
from OCC.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCC.gp import gp_Ax2, gp_Pnt, gp_Dir, gp_Ax1, gp_Trsf, gp_Vec, gp_OZ, gp_Circ
from OCC.TopAbs import TopAbs_REVERSED
from OCC.TopoDS import topods
from OCC.BRepTools import breptools_UVBounds
from OCC.BRep import BRep_Tool_Surface
from OCC.GeomLProp import GeomLProp_SLProps
from OCC.GC import GC_MakeArcOfCircle, GC_MakeSegment
from OCC.TopTools import TopTools_ListIteratorOfListOfShape
from OCC.AIS import AIS_ColoredShape
from OCC.Display.SimpleGui import init_display
from OCC.Display.OCCViewer import rgb_color

import occ_utils

DRAIN_R = 10.0
DRAIN_S = 0.5
DRAIN_T = 1.0
DRAIN_RCS = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))



def wire_circle():
    '''
        standard circle on XY plane, centered at origin, with radius 1

    output
        w:     TopoDS_Wire
    '''
    circ = gp_Circ(DRAIN_RCS, 1.0)
    edge = BRepBuilderAPI_MakeEdge(circ, 0., 2*pi).Edge()
    wire = BRepBuilderAPI_MakeWire(edge).Wire()

    return wire



def wire_triangle3():
    '''
    equal sided triangle, centered at origin
    output
        w:  TopoDS_Wire
    '''
    pt1 = gp_Pnt(-1, 0, 0)
    pt2 = gp_Pnt(-1, 0, 0)
    pt2.Rotate(gp_OZ(), 2*pi/3)
    pt3 = gp_Pnt(-1, 0, 0)
    pt3.Rotate(gp_OZ(), 4*pi/3)

    ed1 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt1, pt2).Value()).Edge()
    ed2 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt2, pt3).Value()).Edge()
    ed3 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt3, pt1).Value()).Edge()

    wire = BRepBuilderAPI_MakeWire(ed1, ed2, ed3).Wire()

    return wire



def wire_triangle2():
    '''
        isosceles triangle
    output
        w:  TopoDS_Wire
    '''
    ang = random.gauss(2*pi/3, pi/6)
    amin = pi / 3
    amax = 5 * pi / 6
    if ang > amax:
        ang = amax
    if ang < amin:
        ang = amin
    pt1 = gp_Pnt(-1, 0, 0)
    pt2 = gp_Pnt(-1, 0, 0)
    pt2.Rotate(gp_OZ(), ang)
    pt3 = gp_Pnt(-1, 0, 0)
    pt3.Rotate(gp_OZ(), -ang)

    ed1 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt1, pt2).Value()).Edge()
    ed2 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt2, pt3).Value()).Edge()
    ed3 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt3, pt1).Value()).Edge()

    wire = BRepBuilderAPI_MakeWire(ed1, ed2, ed3).Wire()

    return wire



def wire_rectangle():
    '''
    output
        w:  TopoDS_Wire
    '''
    pt1 = gp_Pnt(0, 1, 0)
    pt2 = gp_Pnt(-1, 0, 0)
    pt3 = gp_Pnt(0, -1, 0)
    pt4 = gp_Pnt(1, 0, 0)

    ed1 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt1, pt2).Value()).Edge()
    ed2 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt2, pt3).Value()).Edge()
    ed3 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt3, pt4).Value()).Edge()
    ed4 = BRepBuilderAPI_MakeEdge(GC_MakeSegment(pt4, pt1).Value()).Edge()

    wire = BRepBuilderAPI_MakeWire(ed1, ed2, ed3, ed4).Wire()

    return wire



def wire_sweep_circle(ct1, ct2):
    '''
    input
        c1:     gp_Pnt
        c2:     gp_Pnt
    output
        w:      TopoDS_Wire
    '''
    center = DRAIN_RCS.Location()
    vec = DRAIN_RCS.Direction()

    radius = center.Distance(ct1)

    pt1 = gp_Pnt(ct1.XYZ())
    pt2 = gp_Pnt(ct1.XYZ())
    pt3 = gp_Pnt(ct2.XYZ())
    pt4 = gp_Pnt(ct2.XYZ())

    vec1 = gp_Vec(ct1, center)
    vec1.Normalize()
    vec2 = gp_Vec(ct2, center)
    vec2.Normalize()

    pt1.Translate(vec1*DRAIN_S)
    pt2.Translate(-vec1*DRAIN_S)
    pt3.Translate(vec2*DRAIN_S)
    pt4.Translate(-vec2*DRAIN_S)

    cir1 = gp_Circ(gp_Ax2(ct1, vec), DRAIN_S)
    ed1 = BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(cir1, pt1, pt2, True).Value()).Edge()

    cir2 = gp_Circ(gp_Ax2(center, vec), radius + DRAIN_S)
    ed2 = BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(cir2, pt2, pt4, False).Value()).Edge()

    cir3 = gp_Circ(gp_Ax2(ct2, vec), DRAIN_S)
    ed3 = BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(cir3, pt4, pt3, True).Value()).Edge()

    cir4 = gp_Circ(gp_Ax2(center, vec), radius - DRAIN_S)
    ed4 = BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(cir4, pt1, pt3, False).Value()).Edge()

    wire = BRepBuilderAPI_MakeWire(ed1, ed2, ed3, ed4).Wire()

    return wire


# list of wire generation function
FLIST = [wire_circle, wire_rectangle, wire_triangle2, wire_sweep_circle]
SKETCH_TYPE = ['circle', 'rectangle', 'triangle2', 'sweep']
FEAT_TYPE = ['hole', 'blind', 'boss']
LABEL_INDEX = {'ohter':0, 'base':1, 'hole_triangle2':2, 'hole_rectangle':3, 'hole_circle':4,
               'hole_sweep':5, 'blind_triangle2':6, 'blind_rectangle':7, 'blind_circle':8,
               'blind_sweep':9, 'boss_triangle2':10, 'boss_rectangle':11, 'boss_circle':12,
               'boss_sweep':13}



def len_seq_natural(pos, pos_list):
    '''
        find the length of the natural sequence from pos, pos is an element of pos_list
    input
        pos:        int
        pos_list:   [int]
    output
        j - i:      int
    '''
    i = pos_list.index(pos)
    j = i + 1
    while j < len(pos_list):
        if pos_list[j] != pos_list[j - 1] + 1:
            break
        j += 1
    return j - i



def list_wire_combo(num_cell, ang, offset, radius):
    '''
    input
       nc:              int, number of cells to be combined
       ang:             float, angle between adjaent cells
       offset:          float, offset angle of start position
       ri:              float, radius of this ring
    output
        wlist:          {TopoDS_Wire: string}
        combo_name:     ''
    '''
    combo_name = ''
    pos_list = list(range(num_cell))
    wlist = {}
    pos_len_name = {}
    while len(pos_list) > 0:
#       1 choose a random location
        pos = random.choice(pos_list)

#       2 choose a random length
        len_seq = len_seq_natural(pos, pos_list)
        len_seq = random.randrange(1, len_seq + 1)

#       3 choose a random shape
        func = random.choice(FLIST)
#        print(pos_list, pos, l, fname[FLIST.index(func)])
        trsf_scale = gp_Trsf()
        trsf_scale.SetScale(DRAIN_RCS.Location(), DRAIN_S)
        trsf_trans = gp_Trsf()
        trans_vec = gp_Vec(DRAIN_RCS.XDirection()) * radius
        trsf_trans.SetTranslation(trans_vec)
        trsf_rotate = gp_Trsf()
        trsf_rotate.SetRotation(gp_Ax1(DRAIN_RCS.Location(), DRAIN_RCS.Direction()),
                                offset + pos * ang)
        if func == wire_sweep_circle and len_seq > 1:
            cir1 = DRAIN_RCS.Location()
            cir2 = DRAIN_RCS.Location()
            cir1.Translate(trans_vec)
            cir1.Rotate(gp_Ax1(DRAIN_RCS.Location(), DRAIN_RCS.Direction()), offset + pos * ang)
            cir2.Translate(trans_vec)
            cir2.Rotate(gp_Ax1(DRAIN_RCS.Location(), DRAIN_RCS.Direction()),
                        offset + (pos + len_seq -1) * ang)
            wire = wire_sweep_circle(cir1, cir2)
        elif func != wire_sweep_circle and len_seq == 1:
            wire = func()
            bresp_trsf = BRepBuilderAPI_Transform(wire, trsf_scale)
            wire = topods.Wire(bresp_trsf.Shape())
            bresp_trsf = BRepBuilderAPI_Transform(wire, trsf_trans)
            wire = topods.Wire(bresp_trsf.Shape())
            bresp_trsf = BRepBuilderAPI_Transform(wire, trsf_rotate)
            wire = topods.Wire(bresp_trsf.Shape())
        else:
            continue

        wname = SKETCH_TYPE[FLIST.index(func)]
        pos_len_name[pos] = (len_seq, wname)
        wlist[wire] = wname
        for pos in range(pos, pos + len_seq):
            pos_list.remove(pos)

    pos_len_name = sorted(pos_len_name.items(), key=lambda t: t[0])
    for pos in pos_len_name:
        combo_name += str(pos[1][0]) + '[' + pos[1][1] + ']'
    return wlist, combo_name



def list_wire_random():
    '''
    output
        wires:      {TopoDS_Wire:string}
        wire_name:  ''
    '''
    wire_name = ''
    #    number of rings
    numr = int((DRAIN_R/4/DRAIN_S-0.5))
    wires = {}

    for i in range(numr):
#        radius of ith ring
        radius = 3*DRAIN_S+i*4*DRAIN_S
#        number of cells per ring
        nump = int(1.5*pi+2*pi*i)
#        print('np:',np)

#        randomly choose the number of cells to combine
        combo_list = range(1, nump // 3 + 1)
        combo = random.choice(combo_list)
#        angle between two adjacent cells
        ang = 2 * pi / nump
#        randomly offset the start cell
        offset = random.gauss(ang / 2, ang / 2)
        if offset < 0.:
            offset = 0.
        if offset > ang:
            offset = ang
        wlist, combo_name = list_wire_combo(combo, ang, offset, radius)
        wires.update(wlist)
        wire_name += str(combo) + '(' + combo_name + ')'
        nump = nump // combo
#        print('combo',combo,'repeat',np)

        ang = 2*pi/nump
        for j in range(1, nump):
            trsf = gp_Trsf()
            trsf.SetRotation(gp_Ax1(DRAIN_RCS.Location(), DRAIN_RCS.Direction()), ang * j)
            for wire in wlist:
                wname = wlist[wire]
                bresp_trsf = BRepBuilderAPI_Transform(wire, trsf)
                wire = topods.Wire(bresp_trsf.Shape())
                wires[wire] = wname

    return wires, wire_name


def face_bottom(shape):
    '''
    input
        s: TopoDS_Shape
    output
        f: TopoDS_Face
    '''
    f_list = occ_utils.set_face(shape)
    face = None
    for face in f_list:
        normal = occ_utils.normal_to_face_center(face)
        if normal.IsEqual(DRAIN_RCS.Direction(), 0.01):
            break

    return face


def map_face_before_and_after_feat(base, feature_maker):
    '''
    input
        base: TopoDS_Shape
        feature_maker: BRepFeat_MakePrism
    output
        fmap: {TopoDS_Face:TopoDS_Face}
    '''

    fmap = {}
    base_faces = occ_utils.set_face(base)

    for face in base_faces:
        if feature_maker.IsDeleted(face):
            continue

        fmap[face] = []
        modified = feature_maker.Modified(face)
        if modified.IsEmpty():
            fmap[face].append(face)
            continue

        occ_it = TopTools_ListIteratorOfListOfShape(modified)
        while occ_it.More():
            fmap[face].append(occ_it.Value())
            occ_it.Next()

    return fmap



def map_from_name(shape, name):
    '''
    input
        shape: TopoDS_Shape
        name: string
    output
        name_map: {TopoDS_Face: string}
    '''
    name_map = {}
    faces = occ_utils.set_face(shape)

    for one_face in faces:
        name_map[one_face] = name

    return name_map



def map_from_shape_and_name(fmap, old_map, new_shape, new_name):
    '''
    input
        fmap: {TopoDS_Face: TopoDS_Face},
        old_map: {TopoDS_Face: int}
        new_shape: TopoDS_Shape
        new_name: string
    output
        new_map: {TopoDS_Face: int}
    '''
    new_map = {}
    new_faces = occ_utils.set_face(new_shape)
    for oldf in fmap:
        old_name = old_map[oldf]
        for samef in fmap[oldf]:
            new_map[samef] = old_name
            new_faces.remove(samef)

    for n_face in new_faces:
        new_map[n_face] = new_name

    return new_map



def shape_multiple_hole_feats(base, wlist):
    '''
        one face and one hole feature for each wire
    input
        base:       TopoDS_Shape
        wlist:      {TopoDS_Wire:string}
    output
        base:       TopoDS_Shape
        name_map:   {TopoDS_Face:int}
        ftype:      ''
    '''
    b_face = face_bottom(base)
    random.shuffle(FEAT_TYPE)
    ftype = random.choice(FEAT_TYPE)
    if ftype == 'hole':
        direction = DRAIN_RCS.Direction()
        fuse = False
        length = DRAIN_T
    elif ftype == 'blind':
        direction = DRAIN_RCS.Direction()
        fuse = False
        length = DRAIN_T / 2
    else:
        direction = -DRAIN_RCS.Direction()
        fuse = True
        length = DRAIN_T / 2

    base_map = map_from_name(base, LABEL_INDEX['base'])
    for wire in wlist:
        face_p = BRepBuilderAPI_MakeFace(wire).Face()
        feature_maker = BRepFeat_MakePrism()
        feature_maker.Init(base, face_p, b_face, direction, fuse, False)
        feature_maker.Build()

        feature_maker.Perform(length)
        shape = feature_maker.Shape()

        fmap = map_face_before_and_after_feat(base, feature_maker)
        name_map = map_from_shape_and_name(fmap, base_map, shape,
                                           LABEL_INDEX[ftype + '_' + wlist[wire]])

        base = shape
        base_map = name_map

    return base, base_map, ftype



def shape_base_drain():
    '''
    output
        s: TopoDS_Shape
    '''
    wire = wire_circle()

    trsf = gp_Trsf()
    trsf.SetScale(DRAIN_RCS.Location(), DRAIN_R)
    bresp_trsf = BRepBuilderAPI_Transform(wire, trsf)
    wire = topods.Wire(bresp_trsf.Shape())

    base_face = BRepBuilderAPI_MakeFace(wire).Face()
    shape = BRepPrimAPI_MakePrism(base_face, gp_Vec(0, 0, DRAIN_T)).Shape()

    return shape



def shape_drain():
    '''
    output
        shape:          TopoDS_Shape
        face_map:       {TopoDS_Face: int}
        id_map:         {TopoDS_Face: int}
        shape_name:     ''
    '''
#    print('shape_drain')
    random.seed()
#    step1, create the base
    base = shape_base_drain()

#    step2, create wires for holes
    wlist, wire_name = list_wire_random()

#    step3, add hole feature from wire
    shape, name_map, feat_name = shape_multiple_hole_feats(base, wlist)

    shape_name = feat_name + '-' + wire_name

    fid = 0
    fset = occ_utils.set_face(shape)
    id_map = {}
    for shape_face in fset:
        id_map[shape_face] = fid
        fid += 1

    return shape, name_map, id_map, shape_name



if __name__ == '__main__':
    print('model_factory.py')

    OCC_DISPLAY, START_OCC_DISPLAY, ADD_MENU, ADD_FUNCTION_TO_MENU = init_display()
    OCC_DISPLAY.EraseAll()

    COLORS = [rgb_color(0, 0, 0), rgb_color(0.75, 0.75, 0.75), rgb_color(1, 0, 0),
              rgb_color(1, 0.5, 0), rgb_color(0, 1, 1), rgb_color(1, 0, 1),
              rgb_color(1, 0.8, 0.8), rgb_color(1, 1, 0.8), rgb_color(0.8, 1, 1),
              rgb_color(1, 0.8, 1), rgb_color(0.4, 0, 0), rgb_color(0.4, 0.4, 0),
              rgb_color(0, 0.4, 0.4), rgb_color(0.4, 0, 0.4)]

    SHAPE, FMAP, ID_MAP, SHAPE_NAME = shape_drain()
    print(SHAPE_NAME)

    AIS = AIS_ColoredShape(SHAPE)
    for a_face in FMAP:
        AIS.SetCustomColor(a_face, COLORS[FMAP[a_face]])

    OCC_DISPLAY.Context.Display(AIS.GetHandle())
    OCC_DISPLAY.View_Iso()
    OCC_DISPLAY.FitAll()

    START_OCC_DISPLAY()
