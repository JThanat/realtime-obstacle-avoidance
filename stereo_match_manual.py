#!/usr/bin/env python

'''
Simple example of stereo image matching and point cloud generation.
Resulting .ply file cam be easily viewed using MeshLab ( http://meshlab.sourceforge.net/ )
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')


if __name__ == '__main__':
    print('loading images...')
    imgL = cv2.pyrDown( cv2.imread('./im0.png') )  # downscale images for faster processing
    imgR = cv2.pyrDown( cv2.imread('./im1.png') )

    window_size = 3
    min_disp = 38
    num_disp = 240
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = 16,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )

    print('computing disparity...')
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
    print(stereo)
    print(disp)

    # print('generating 3d point cloud...',)

    # cam0=[5806.559 0 1429.219; 0 5806.559 993.403; 0 0 1]
    # cam1=[5806.559 0 1543.51; 0 5806.559 993.403; 0 0 1]
    # doffs=114.291
    # baseline=174.019
    # width=2960
    # height=2016
    # ndisp=250
    # isint=0
    # vmin=38
    # vmax=222
    # dyavg=0
    # dymax=0

    w = 2960
    h = 2016
    f = 5806.559
    baseline = 174.019
    doffs=114.291
    cx = 1429.219
    cy = 993.403    
    # z = baseline * f / (d + doffs)

    # Q = np.float32([[1, 0, 0, -cx],
    #                 [0,-1, 0,  cy], # turn points 180 deg around x-axis,
    #                 [0, 0, 0,  -f], # so that y-axis looks up
    #                 [0, 0, 1,   baseline]])
    # points = cv2.reprojectImageTo3D(disp, Q)
    # colors = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)
    # mask = disp > disp.min()
    # out_points = points[mask]
    # out_colors = colors[mask]
    # out_fn = 'out.ply'
    # write_ply('out.ply', out_points, out_colors)
    # print('%s saved' % 'out.ply')

    cv2.imshow('left', imgL)
    cv2.imshow('disparity', (disp-min_disp)/num_disp)
    cv2.waitKey()
    cv2.destroyAllWindows()