# THE USE OF THIS SOURCE-CODE. USE AT YOUR OWN RISK.

import cv2
import numpy as np
import time

def draw_object(image, x, y, width=50, height=100):
    color = image[y, x]
    image[y-height:y, x-width//2:x+width//2] = color


IMAGE_HEIGHT = 600
IMAGE_WIDTH = 800

while True:

    max_disp = 200

    # create fake disparity
    image = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH), np.uint8)

    for c in range(IMAGE_HEIGHT)[::-1]:
        image[c, ...] = int(float(c) / IMAGE_HEIGHT * max_disp)

    draw_object(image, 275, 175)
    draw_object(image, 300, 200)
    draw_object(image, 100, 350)

    # calculate v-disparity
    vhist_vis = np.zeros((IMAGE_HEIGHT, max_disp), np.float)
    for i in range(IMAGE_HEIGHT):
        vhist_vis[i, ...] = cv2.calcHist(images=[image[i, ...]], channels=[0], mask=None, histSize=[max_disp],
                                         ranges=[0, max_disp]).flatten() / float(IMAGE_WIDTH)

    print(vhist_vis)
    vhist_vis = np.array(vhist_vis * 255, np.uint8)
    vblack_mask = vhist_vis < 5
    print(vblack_mask)
    vhist_vis = cv2.applyColorMap(vhist_vis, cv2.COLORMAP_JET)
    vhist_vis[vblack_mask] = 0

    # calculate u-disparity
    uhist_vis = np.zeros((max_disp, IMAGE_WIDTH), np.float)
    for i in range(IMAGE_WIDTH):
        uhist_vis[..., i] = cv2.calcHist(images=[image[..., i]], channels=[0], mask=None, histSize=[max_disp],
                                         ranges=[0, max_disp]).flatten() / float(IMAGE_HEIGHT)

    uhist_vis = np.array(uhist_vis * 255, np.uint8)
    ublack_mask = uhist_vis < 5
    uhist_vis = cv2.applyColorMap(uhist_vis, cv2.COLORMAP_JET)
    uhist_vis[ublack_mask] = 0


    image = cv2.applyColorMap(image, cv2.COLORMAP_JET)


    cv2.imshow('image', image)

    cv2.imshow('vhist_vis', vhist_vis)
    cv2.imshow('uhist_vis', uhist_vis)

    cv2.imwrite('disparity_image.png', image)
    cv2.imwrite('v-disparity.png', vhist_vis)
    cv2.imwrite('u-disparity.png', uhist_vis)


    if chr(cv2.waitKey(0)&255) == 'q':
        break