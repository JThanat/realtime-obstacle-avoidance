# disparity from
# https://stackoverflow.com/questions/38504760/fast-calculation-of-v-disparity-with-opencv-function-calchist
# calcHist Example https://www.programcreek.com/python/example/70449/cv2.calcHist
from __future__ import print_function
from __future__ import division

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

IMAGE_HEIGHT = 2016
IMAGE_WIDTH = 2960


def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')


def calculate_vdisparity(disp_img,max_disp, img_height):
    # calculate v-disparity
    vhist_vis = np.zeros((img_height, max_disp), np.float)
    for i in range(img_height):
        vhist_vis[i, ...] = cv2.calcHist(images=[disp_img[i, ...]], channels=[0], mask=None, histSize=[max_disp],
                                         ranges=[0, max_disp]).flatten() / float(img_height)

    vhist_vis = np.array(vhist_vis * 255, np.uint8)
    vblack_mask = vhist_vis < 5
    vhist_vis = cv2.applyColorMap(vhist_vis, cv2.COLORMAP_JET)
    vhist_vis[vblack_mask] = 0
    return vhist_vis

def calculate_udisparity(disp_img, max_disp, img_width):
    # calculate u-disparity
    uhist_vis = np.zeros((max_disp, img_width), np.float)
    for i in range(img_width):
        uhist_vis[..., i] = cv2.calcHist(images=[disp_img[..., i]], channels=[0], mask=None, histSize=[max_disp],
                                         ranges=[0, max_disp]).flatten() / float(img_width)

    uhist_vis = np.array(uhist_vis * 255, np.uint8)
    ublack_mask = uhist_vis < 5
    uhist_vis = cv2.applyColorMap(uhist_vis, cv2.COLORMAP_JET)
    uhist_vis[ublack_mask] = 0
    return uhist_vis

if __name__ == '__main__':
    print('loading images...')
    imgD = cv2.imread('./frame0002_disp.png')  
    imgR = cv2.imread('./frame0002.png')
    #  = cv2.cvtColor(imgD, cv2.COLOR_BGR2GRAY)

    h, w, channel = imgD.shape
    disparity_map = np.zeros((h, w), np.uint8)
    for i in range(h):
        for j in range(w):
            disparity_map[i][j] = np.mean(imgD[i][j])
    

    max_disp = int(disparity_map.max())
    for i in range(h):
        for j in range(w):
            disparity_map[i][j] = (max_disp - disparity_map[i][j]) / float(max_disp) * 255.0
    
    u_disparity = calculate_udisparity(disparity_map,255, w)
    v_disparity = calculate_vdisparity(disparity_map,255, h)

    disparity_map = cv2.applyColorMap(disparity_map, cv2.COLORMAP_JET)
    # disparity_map_origin = cv2.applyColorMap(imgD, cv2.COLORMAP_JET)
    cv2.imshow('disparity', disparity_map)
    cv2.imshow('udisp', u_disparity)
    cv2.imshow('vdisp', v_disparity)
    cv2.imshow('original', imgR)
    cv2.waitKey()
    cv2.destroyAllWindows()
