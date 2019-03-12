# -*- coding: utf-8 -*-
"""
Created on Fri Jan 18 12:17:14 2019

@author: 2624224
"""
import os
import struct

def point_cloud_from_file(filename, has_label=True):
    '''
    input
        filename:   ''
        has_label:  bool
    output
        pts:        [[float,float,float]]
        normals:    [[float,float,float]]
        segs：       [int]
    '''
    with open(filename, 'rb') as file:
        num = struct.unpack('i', file.read(4))[0]
        print(num)
        pts = []
        for _ in range(num):
            coord_x = struct.unpack('f', file.read(4))[0]
            coord_y = struct.unpack('f', file.read(4))[0]
            coord_z = struct.unpack('f', file.read(4))[0]
            pts.append([coord_x, coord_y, coord_z])

        normals = []
        for _ in range(num):
            coord_x = struct.unpack('f', file.read(4))[0]
            coord_y = struct.unpack('f', file.read(4))[0]
            coord_z = struct.unpack('f', file.read(4))[0]
            normals.append([coord_x, coord_y, coord_z])

        segs = []
        if has_label is True:
            for _ in range(num):
                seg = struct.unpack('i', file.read(4))[0]
                segs.append(seg)

    return pts, normals, segs



def point_cloud_to_file(filename, pts, normals, segs):
    '''
    input
        filename:   ''
        pts:        [[float,float,float]]
        normals:    [[float,float,float]]
        segs:       [int]
    '''
    with open(filename, 'wb') as file:
        num = len(pts)
        file.write(struct.pack('i', num))
        for point in pts:
            file.write(struct.pack('f', point[0]))
            file.write(struct.pack('f', point[1]))
            file.write(struct.pack('f', point[2]))

        for normal in normals:
            file.write(struct.pack('f', normal[0]))
            file.write(struct.pack('f', normal[1]))
            file.write(struct.pack('f', normal[2]))

        for seg in segs:
            file.write(struct.pack('i', seg))



def upgraded_point_cloud_from_file(filename):
    '''
    input
        filename: ''
    output
        pts:        [[float] * 3] * number of point
        normals:    [[float] * 3] * number of point
        features:   [[float] * number of channels] * number of point
        labels:     [int] * number of point
    '''
    with open(filename, 'rb') as file:
        magic_str_ = []
        for _ in range(16):
            magic_str_.append(struct.unpack('c', file.read(1))[0])

        npt = struct.unpack('i', file.read(4))[0]

        content_flags_ = struct.unpack('i', file.read(4))[0]

        channels_ = []
        for _ in range(8):
            channels_.append(struct.unpack('i', file.read(4))[0])

        ptr_dis_ = []
        for _ in range(8):
            ptr_dis_.append(struct.unpack('i', file.read(4))[0])

#        print(magic_str_)
#        print(npt)
#        print(content_flags_)
#        print(channels_)
#        print(ptr_dis_)

        pts = []
        for _ in range(npt):
            coord_x = struct.unpack('f', file.read(4))[0]
            coord_y = struct.unpack('f', file.read(4))[0]
            coord_z = struct.unpack('f', file.read(4))[0]
            pts.append([coord_x, coord_y, coord_z])

        normals = []
        for _ in range(npt):
            coord_x = struct.unpack('f', file.read(4))[0]
            coord_y = struct.unpack('f', file.read(4))[0]
            coord_z = struct.unpack('f', file.read(4))[0]
            normals.append([coord_x, coord_y, coord_z])

        features = []
        if content_flags_ & 4 != 0:
            cnum = channels_[2]
            for _ in range(npt):
                feat = []
                for _ in range(cnum):
                    feat.append(struct.unpack('f', file.read(4))[0])
                features.append(feat)

        labels = []
        if content_flags_ & 8 != 0:
            for _ in range(npt):
                label = struct.unpack('f', file.read(4))[0]
                labels.append(int(label))

    return pts, normals, features, labels


def upgraded_point_cloud_to_file(filename, pts, normals, features, labels):
    '''
    write upgraded point cloud to file
    '''
#    print('upgraded_point_cloud_to_file')
    if not os.path.exists(filename):
        print(filename, ' not exists')
        return
        
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


def labels_from_file(filename):
    '''
    input
        filename:   ''
    output
        labels:     [int] * number of lines
    '''
    labels = []
    with open(filename, 'r') as file:
        for line in file:
            labels.append(int(float(line)))

    return labels


def label_index_from_file(filename):
    '''
    read label index
    '''
    print('label_index_from_file')
    label_index = []
    with open(filename, 'rb') as file:
        npt = struct.unpack('i', file.read(4))[0]
#        print('npt', npt)
        for _ in range(npt):
            label_index.append(struct.unpack('i', file.read(4))[0])

    return label_index


def file_names_from_dir(where, pattern=''):
    '''
    '''
    names = []
    for name in os.listdir(where):
        if name.find(pattern) is not -1:
            names.append(name)
    
    return names
    
def file_paths_from_dir(where, pattern=''):
    '''
    '''
    return [where + '/' + name for name in file_names_from_dir(where, pattern)]
    

if __name__ == '__main__':
    FILE_NAME = 'D:/Weijuan/dataset/ModelNet40/ModelNet40/airplane/test/airplane_0627.points'
    PTS, NORMALS, SEGS = point_cloud_from_file(FILE_NAME, False)
    print(len(PTS))
