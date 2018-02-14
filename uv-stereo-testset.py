# disparity from
# https://stackoverflow.com/questions/38504760/fast-calculation-of-v-disparity-with-opencv-function-calchist
# calcHist Example https://www.programcreek.com/python/example/70449/cv2.calcHist
from __future__ import print_function
from __future__ import division

import numpy as np
import os
import cv2
import math

from copy import copy

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

IMAGE_HEIGHT = 2016
IMAGE_WIDTH = 2960
BASE_DIR = './'


def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')


def calculate_vdisparity(disp_img, max_disparity, img_height, img_width, obstacle=None):
    # calculate v-disparity
    vhist_vis = np.zeros((img_height, max_disparity), np.float)
    for i in range(img_height):
        # [disp_img[i, ...]] make two dimesion arrays (just like orginal image but we just put only one row of the image here)
        # read more about calcHist at https://docs.opencv.org/2.4/modules/imgproc/doc/histograms.html
        # flatten the array to make it to one dimension array
        # divide all row with img_width value to normalize the histogram (make it less or equal to 1)
        vhist_vis[i, ...] = cv2.calcHist(images=[disp_img[i, ...]], channels=[0], mask=None, histSize=[max_disparity],
                                         ranges=[0, max_disparity]).flatten() / float(img_width)


    vhist_vis = np.array(vhist_vis * 255, np.uint8)
    # mask_threshold = max_disparity/10
    mask_threshold = 20

    vblack_mask = vhist_vis < mask_threshold
    vwhite_mask = vhist_vis >= mask_threshold

    vhist_vis[vblack_mask] = 0
    vhist_vis[vwhite_mask] = 255

    # Add houghman line extract
    lines = cv2.HoughLinesP(vhist_vis, 1, math.pi/180.0, 5, np.array([]), 40, 10)
    a,b,c = lines.shape
    tmp = np.zeros((img_height, max_disparity), np.float)
    for i in range(a):
        # line on x1 y1 and x2 y2
        # x is disparity and y is row
        # print("{} {} {} {}".format(lines[i][0][0], lines[i][0][1], lines[i][0][2], lines[i][0][3]))
        cv2.line(tmp, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (255, 0, 0), 1, cv2.LINE_AA)
        
        if obstacle is None:
            continue

        if lines[i][0][0] != 0:
            expect_disp = lines[i][0][0]
            # for j in range row y1 to row y2
            if expect_disp != 0:
                for j in range(lines[i][0][1], lines[i][0][3]+1):
                    for k in range(img_width):
                        if disp_img[j][k] == expect_disp:
                            obstacle[j][k] = 125



    vhist_vis = cv2.applyColorMap(vhist_vis, cv2.COLORMAP_JET)
    # return vhist_vis
    return tmp

def calculate_udisparity(disp_img, max_disparity, img_height, img_width, obstacle=None):
    # calculate u-disparity
    uhist_vis = np.zeros((max_disparity, img_width), np.float)
    for i in range(img_width):
        uhist_vis[..., i] = cv2.calcHist(images=[disp_img[..., i]], channels=[0], mask=None, histSize=[max_disparity],
                                         ranges=[0, max_disparity]).flatten() / float(img_height)

    uhist_vis = np.array(uhist_vis * 255, np.uint8)
    mask_threshold = 10
    ublack_mask = uhist_vis < mask_threshold
    uwhite_mask = uhist_vis >= mask_threshold

    uhist_vis[ublack_mask] = 0
    uhist_vis[uwhite_mask] = 255

    # Add houghman line extract
    lines = cv2.HoughLinesP(uhist_vis, 1, math.pi/180.0, 5, np.array([]), 40,10)
    a,b,c = lines.shape
    tmp = np.zeros((max_disparity, img_width), np.float)
    for i in range(a):
        cv2.line(tmp, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (255, 0, 0), 1, cv2.LINE_AA)
        # print("{} {} {} {}".format(lines[i][0][0], lines[i][0][1], lines[i][0][2], lines[i][0][3]))
        # x is column y is disparity
        if obstacle is None:
            continue
            
        if lines[i][0][1] == lines[i][0][3] and lines[i][0][1] != 0:
            expect_disp = lines[i][0][1]
            # from range column x1 to column x2
            for k in range(lines[i][0][0], lines[i][0][2]+1):
                for j in range(img_height):
                    if disp_img[j][k] == expect_disp:
                        obstacle[j][k] = 125

    uhist_vis = cv2.applyColorMap(uhist_vis, cv2.COLORMAP_JET)
    # return uhist_vis
    return tmp

def scale_disparity(disparity_map, h, w, max_disp, num_disp, scaling_factor):
    m = copy(disparity_map)
    for i in range(h):
        for j in range(w):
            # Invert the testset grayscale value and scale to 255 (Make the nearer point color brighter)
            # m[i][j] = (max_disp - m[i][j]) / num_disp * scaling_factor
            if m[i][j] != 0:
                m[i][j] = (max_disp - m[i][j]) / num_disp * scaling_factor
    return m

if __name__ == '__main__':
    print('loading images...')
    img_path = os.path.join(BASE_DIR, 'dataset/DATASET-CVC-02/CVC-02-CG/data')
    color_img_path = os.path.join(img_path, 'color')
    depth_img_path = os.path.join(img_path, 'depth')
    img_name = 'frame0005.png'
    imgR = cv2.imread(os.path.join(color_img_path, img_name))
    imgD = cv2.imread(os.path.join(depth_img_path, img_name))  
    #  = cv2.cvtColor(imgD, cv2.COLOR_BGR2GRAY)

    h, w, channel = imgD.shape
    print('h:{} w:{} channel:{}'.format(h,w,channel))
    disparity_map = np.zeros((h, w), np.uint8)
    for i in range(h):
        for j in range(w):
            disparity_map[i][j] = np.mean(imgD[i][j])
    

    max_disp = int(disparity_map.max())
    min_disp = int(disparity_map.min())
    num_disp = max_disp-min_disp

    scaling_factor = max_disp
    obs_u = np.zeros((h, w), np.uint8)
    obs_v = np.zeros((h, w), np.uint8)
    obs = np.zeros((h, w), np.uint8)

    disparity_map = scale_disparity(disparity_map, h, w, max_disp, num_disp, scaling_factor)

    print('calculate uv-disparity...')
    u_disparity = calculate_udisparity(disp_img=disparity_map, max_disparity=scaling_factor, img_height=h, img_width=w, obstacle=obs_u)
    v_disparity = calculate_vdisparity(disp_img=disparity_map, max_disparity=scaling_factor, img_height=h, img_width=w, obstacle=obs_v)

    obs = np.add(obs_u,obs_v)
    x,y = obs.shape

    # for i in range(x):
    #     for j in range(y):
    #         obs[i][j] = obs_u[i][j] + obs_v[i][j]

    scaled_disparity = cv2.applyColorMap(disparity_map, cv2.COLORMAP_JET)
    disparity_map_origin = cv2.applyColorMap(imgD, cv2.COLORMAP_JET)

    print('displaying images...')
    cv2.imshow('disparity', disparity_map)
    cv2.imshow('udisp', u_disparity)
    cv2.imshow('vdisp', v_disparity)
    cv2.imshow('original', imgR)
    cv2.imshow('original_disp', disparity_map_origin)
    cv2.imshow('obstacle_map', obs)
    cv2.imshow('obstacle_v',obs_v)
    cv2.imshow('obstacle_u',obs_u)
    cv2.waitKey(0)
    cv2.destroyAllWindows()