# -*- coding: utf-8 -*-
"""
Created on Fri Feb 22 16:24:02 2019

@author: 2624224
"""
import os
import subprocess
import argparse

import file_utils
            
def generate_octree(ocnn_root, root_dir, depth, rot_num):
    '''
    '''
    octree_dir = root_dir + '/octree_' + depth + '_' + rot_num
    if not os.path.exists(octree_dir):
        os.mkdir(octree_dir)    

    points_dir = root_dir + '/points/'
    points_paths = [path + '\n' for path in file_utils.file_paths_from_dir(points_dir, '.points')]     
    points_list = root_dir + '/points_list.txt'
    with open(points_list, 'w') as file:
        file.writelines(points_paths)
        
    octree = ocnn_root + '/octree.exe'
    subprocess.check_call([octree, '--filenames', points_list, '--output_path', octree_dir,
                           '--depth', depth, '--rot_num', rot_num])
    

if __name__ == '__main__':
    '''
    input:
        ocnnroot, dir where octree.exe resides
        
        rootdir, dir containing points dir where *.points files reside
        
        depth, at what depth octrees should be generated
        
        num, how many views to generate for each octree, 1 for test, 12 for training
        
    output:
        octree dir for specified depth and num under rootdir
        
        *.octree files and *.label_index files in octree dir
        see ocnn codes for format of *.octree
        *.label_index contains id of octree leaf nodes for each point, in the same 
        order as in *.points file, id corresponds to entry number in *.label_predicted   
    '''
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument('--ocnnroot',
                        '-o',
                        type=str,
                        required=True)
    PARSER.add_argument('--rootdir',
                        '-r',
                        type=str,
                        help='root dir containning points dir',
                        required=True)
    PARSER.add_argument('--depth',
                        '-d',
                        type=int,
                        required=False,
                        default=6)
    
    PARSER.add_argument('--num',
                        '-n',
                        type=int,
                        required=False,
                        default=1)

    ARGS = PARSER.parse_args()
    
    generate_octree(ARGS.ocnnroot, ARGS.rootdir, str(ARGS.depth), str(ARGS.num))