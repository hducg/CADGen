# -*- coding: utf-8 -*-
"""
Created on Fri Mar 15 16:24:52 2019

@author: 2624224
"""

import argparse
from multiprocessing import Pool
import glob
import os
import struct
import time

import numpy as np

#import file_utils
import stl

def upgraded_point_cloud_to_file(filename, pts, normals, features=[], labels=[]):
    '''
    write upgraded point cloud to file
    '''
#    print('upgraded_point_cloud_to_file')
        
    npt = len(pts)
    if len(normals) != npt and len(features) != npt:
        print('either normal or feature info is not correct')
        return npt

    with open(filename, 'wb') as file:
        magic_str = '_POINTS_1.0_\0\0\0\0'
        for i in range(16):
            file.write(struct.pack('c', magic_str[i].encode('ascii')))

        file.write(struct.pack('i', npt))

        content_flags = 0|1 # KPoint = 1
        channels = [0] * 8
        channels[0] = 3
        ptr_dis = [0] * 8
        ptr_dis[0] = 88
        if len(normals) > 0:
            content_flags |= 2  # KNormal = 2
            channels[1] = 3
        if len(features) > 0:
            content_flags |= 4  # KFeature = 4
            channels[2] = len(features[0])
        if len(labels) > 0:
            content_flags |= 8  # KLabel = 8
            channels[3] = 1
        for i in range(1, 5):
            ptr_dis[i] = ptr_dis[i-1] + 4 * npt * channels[i-1]

        file.write(struct.pack('i', content_flags))

        for i in range(8):
            file.write(struct.pack('i', channels[i]))

        for i in range(8):
            file.write(struct.pack('i', ptr_dis[i]))

        for point in pts:
            file.write(struct.pack('f', point[0]))
            file.write(struct.pack('f', point[1]))
            file.write(struct.pack('f', point[2]))

        for normal in normals:
            file.write(struct.pack('f', normal[0]))
            file.write(struct.pack('f', normal[1]))
            file.write(struct.pack('f', normal[2]))

        for feat in features:
            for item in feat:
                file.write(struct.pack('f', item))

        for label in labels:
            file.write(struct.pack('f', label))

    return npt
    
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
    points_path = stl_path.split('.')[0].replace('stl', 'points') + '.points'
   
    if not os.path.exists(points_path):
        the_mesh = stl.mesh.Mesh.from_file(stl_path)
        pts, normals = points_from_stl_mesh(the_mesh)    
        upgraded_point_cloud_to_file(points_path, pts, normals)


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
    start = time.time()
    Pool().map(points_file_from_stl_path, stl_list)
    end = time.time()
    print(end - start, 'seconds')   
    
