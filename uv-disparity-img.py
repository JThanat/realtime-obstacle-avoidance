# disparity from
# https://stackoverflow.com/questions/38504760/fast-calculation-of-v-disparity-with-opencv-function-calchist
# calcHist Example https://www.programcreek.com/python/example/70449/cv2.calcHist
from __future__ import print_function

import cv2
import numpy as np

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
                                         ranges=[0, max_disp]).flatten() / float(IMAGE_HEIGHT)

    vhist_vis = np.array(vhist_vis * 255, np.uint8)
    vblack_mask = vhist_vis < 5
    vhist_vis = cv2.applyColorMap(vhist_vis, cv2.COLORMAP_JET)
    vhist_vis[vblack_mask] = 0
    return vhist_vis

def calculate_udisparity(disp_img, max_disp, img_width):
    # calculate u-disparity
    uhist_vis = np.zeros((max_disp, img_width), np.float)
    for i in range(IMAGE_WIDTH):
        uhist_vis[..., i] = cv2.calcHist(images=[disp_img[..., i]], channels=[0], mask=None, histSize=[max_disp],
                                         ranges=[0, max_disp]).flatten() / float(IMAGE_WIDTH)

    uhist_vis = np.array(uhist_vis * 255, np.uint8)
    ublack_mask = uhist_vis < 5
    uhist_vis = cv2.applyColorMap(uhist_vis, cv2.COLORMAP_JET)
    uhist_vis[ublack_mask] = 0
    return uhist_vis

if __name__ == '__main__':
    print('loading images...')
    imgL = cv2.pyrDown(cv2.imread('./im0.png'))  # downscale images for faster processing
    imgR = cv2.pyrDown(cv2.imread('./im1.png'))

    window_size = 3
    min_disp = 38
    num_disp = 240
    stereo = cv2.StereoSGBM_create(minDisparity=min_disp,
                                   numDisparities=num_disp,
                                   blockSize=16,
                                   P1=8 * 3 * window_size ** 2,
                                   P2=32 * 3 * window_size ** 2,
                                   disp12MaxDiff=1,
                                   uniquenessRatio=10,
                                   speckleWindowSize=100,
                                   speckleRange=32
                                   )

    print('computing disparity...')
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
    disp_h, disp_w = disp.shape

    w = IMAGE_WIDTH
    h = IMAGE_HEIGHT
    f = 5806.559
    baseline = 174.019
    doffs = 114.291
    cx = 1429.219
    cy = 993.403

    uhist_vis = calculate_udisparity(disp_img=disp, max_disp=min_disp + num_disp, img_width=disp_w)
    vhist_vis = calculate_vdisparity(disp_img=disp, max_disp=min_disp + num_disp, img_height=disp_h)

    cv2.imshow('left', imgL)
    cv2.imshow('disparity', (disp - min_disp) / num_disp)
    cv2.imshow('vhist_vis', vhist_vis)
    cv2.imshow('uhist_vis', uhist_vis)
    cv2.waitKey()
    cv2.destroyAllWindows()


    # cv2.imwrite('v-disparity.png', vhist_vis)
    # cv2.imwrite('u-disparity.png', uhist_vis)

