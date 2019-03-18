# -*- coding: utf-8 -*-
"""
Created on Fri Mar 15 16:24:52 2019

@author: 2624224
"""

import argparse
import glob
from multiprocessing import Pool
import os

import numpy as np

#from stl import mesh
#
#import file_utils

def points_from_triangle(tri):
    pts = np.array([np.array(tri[0:3]), np.array(tri[3:6]), np.array(tri[6:9])])
    lens = np.array([np.sum(np.square(pts[1] - pts[2])), np.sum(np.square(pts[0] - pts[2])), np.sum(np.square(pts[0] - pts[1]))])
    idx = np.argsort(lens)
    lens = lens[idx]
    pts = pts[idx]

    resolution = 10.0 / 128
    nv = int(np.sqrt(lens[2]) / resolution)
    if nv == 0:
        nv = 1
    nh = int(np.sqrt(lens[0]) / resolution)
    if nh == 0:
        nh = 1

    samples = []
    for i in range(nv + 1):#[0, Nv]
        t1 = i / nv#[0, 1]
        pt1 = np.multiply(pts[1], t1) + np.multiply(pts[0], 1 - t1)#[0, 1]
        pt2 = np.multiply(pts[2], t1) + np.multiply(pts[0], 1 - t1)#[0, 2]
        nn = int(nh * t1) #[1, N2 + 1]
        if nn == 0:
            nn = 1
        for j in range(nn + 1):#[0, 1] [0, N2]
            t2 = j / nn #[0, 1]
            pt = np.multiply(pt1, t2) + np.multiply(pt2, 1 - t2)#uv12 * t2 + uv13 * (1 - t2)
            samples.append(pt.tolist())

    return samples


def points_from_stl_mesh(the_mesh):
    pts = []
    normals = []
    for i in range(len(the_mesh.points)):
        tri = the_mesh.points[i]
        nml = the_mesh.normals[i]
        samples = points_from_triangle(tri)
        pts += samples
        normals += [nml] * len(samples)

    return pts, normals


def points_file_from_stl_path(stl_path):
#    the_mesh = mesh.Mesh.from_file(stl_path)
#    pts, normals = points_from_stl_mesh(the_mesh)
    points_path = stl_path.split('.')[0].replace('stl', 'points') + '.points'    
#    file_utils.upgraded_point_cloud_to_file(points_path, pts, normals)
    return points_path


if __name__ is '__main__':
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument('--rootdir',
                        '-r',
                        type=str,
                        help='root dir containning the stl dir, points dir, \
                        octree dir, lmdb dir',
                        required=True)
    ARGS = PARSER.parse_args()

    if not os.path.exists(ARGS.rootdir + '/points'):
        os.mkdir(ARGS.rootdir + '/points')

    sub_dirs = [item.replace('stl', 'points') for item in glob.glob(ARGS.rootdir + '/stl/*')]
    for sub_dir in sub_dirs:
        if not os.path.exists(sub_dir):
            os.mkdir(sub_dir)

    stl_list = glob.glob(ARGS.rootdir + '/stl/*/*')
    print(stl_list[:8])
    result = Pool().map(points_file_from_stl_path, stl_list)
    print(result[:8])
