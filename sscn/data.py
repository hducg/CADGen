# Copyright 2016-present, Facebook, Inc.
# All rights reserved.
#
# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import torch, torch.utils.data
import glob
import random
import pickle


data_dir = '../../datasets/machining_feature/'

def init(resolution=50,sz=50*8+8,batchSize=16):
#    globals()['categ']=c
    globals()['resolution']=resolution
    globals()['batchSize']=batchSize
    globals()['spatialSize']=torch.LongTensor([sz]*3)
    globals()['nClassesTotal']=24
    
    pts_path_list = glob.glob(data_dir + 'points/*.pts')
    random.seed()
    random.shuffle(pts_path_list)
    
    num_train = int(len(pts_path_list) * 0.4)
    print(num_train, 'training samples')    
    with open(data_dir + 'train_list', 'wb') as file:
        pickle.dump(pts_path_list[: num_train], file)

    with open(data_dir + 'valid_list', 'wb') as file:
        pickle.dump(pts_path_list[num_train : 2 * num_train], file)

    with open(data_dir + 'test_list', 'wb') as file:
        pickle.dump(pts_path_list[2 * num_train :], file)
    
    
def load(xF):
    xl=np.loadtxt(xF[0])
    xl/= ((xl**2).sum(1).max()**0.5)
    y = np.loadtxt(xF[0][:-3]+'seg').astype('int64')
    return (xF[0], xl, y, np.random.randint(1e6))

def train():
    d=[]
    with open(data_dir + 'train_list', 'rb') as file:
        pts_path_list = pickle.load(file)
    
    for x in torch.utils.data.DataLoader(
        pts_path_list,
        collate_fn=lambda x: load(x),
        num_workers=12):
        d.append(x)

    def merge(tbl):
        xl_=[]
        xf_=[]
        y_=[]
        nPoints_=[]
        np_random=np.random.RandomState([x[-1] for x in tbl])
        for _, xl, y, idx in tbl:
            m=np.eye(3,dtype='float32')
            m[0,0]*=np_random.randint(0,2)*2-1
            m=np.dot(m,np.linalg.qr(np_random.randn(3,3))[0])
            xl=np.dot(xl,m)
            xl+=np_random.uniform(-1,1,(1,3)).astype('float32')
            xl=np.floor(resolution*(4+xl)).astype('int64')
            xf=np.ones((xl.shape[0],1)).astype('float32')
            xl_.append(xl)
            xf_.append(xf)
            y_.append(y)
            nPoints_.append(y.shape[0])
        xl_=[np.hstack([x,idx*np.ones((x.shape[0],1),dtype='int64')]) for idx,x in enumerate(xl_)]
        return {'x':  [torch.from_numpy(np.vstack(xl_)),torch.from_numpy(np.vstack(xf_))],
                'y':           torch.from_numpy(np.hstack(y_)),
                'xf':          [x[0] for x in tbl],
                'nPoints':     nPoints_}
    return torch.utils.data.DataLoader(d,batch_size=batchSize, collate_fn=merge, num_workers=10, shuffle=True)

def valid():
    d=[]
    with open(data_dir + 'train_list', 'rb') as file:
        pts_path_list = pickle.load(file)
    
    for x in torch.utils.data.DataLoader(
        pts_path_list,
        collate_fn=lambda x: load(x),
        num_workers=12):
        d.append(x)

    def merge(tbl):
        xl_=[]
        xf_=[]
        y_=[]
        nPoints_=[]
        np_random=np.random.RandomState([x[-1] for x in tbl])
        for _, xl, y, idx in tbl:
            m=np.eye(3,dtype='float32')
            m[0,0]*=np_random.randint(0,2)*2-1
            m=np.dot(m,np.linalg.qr(np_random.randn(3,3))[0])
            xl=np.dot(xl,m)
            xl+=np_random.uniform(-1,1,(1,3)).astype('float32')
            xl=np.floor(resolution*(4+xl)).astype('int64')
            xl_.append(xl)
            xf=np.ones((xl.shape[0],1)).astype('float32')
            xf_.append(xf)
            y_.append(y)
            nPoints_.append(y.shape[0])
        xl_=[np.hstack([x,idx*np.ones((x.shape[0],1),dtype='int64')]) for idx,x in enumerate(xl_)]
        return {'x':  [torch.from_numpy(np.vstack(xl_)),torch.from_numpy(np.vstack(xf_))],
                'y':           torch.from_numpy(np.hstack(y_)),
                'xf':          [x[0] for x in tbl],
                'nPoints':     nPoints_}
    return torch.utils.data.DataLoader(d,batch_size=batchSize, collate_fn=merge, num_workers=10, shuffle=True)
