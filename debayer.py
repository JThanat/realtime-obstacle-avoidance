# scp -r ubuntu@192.168.30.50:/home/ubuntu/cameraDev/export_img /home/thanat/obstacle_avoidance/data/camera
# https://stackoverflow.com/questions/596216/formula-to-determine-brightness-of-rgb-color
import numpy as np
import cv2
import glob
import time
import os

if __name__ == '__main__':
    images = glob.glob('./data/camera/export_img/*.jpg')

    for fname in images:
        print(fname)
        img = cv2.imread(fname)
        img_name = fname.split('.')[1].split('/')[4]
        h, w, c = img.shape
        debayer_img = np.zeros((int(h/2), int(w/2)), np.uint8)
        # bayer pattern
        # g r 
        # b g
        start = time.time()
        for i in range(0,h,2):
            for j in range(0,w,2):
                g = (int(img[i][j][0]) + int(img[i+1][j+1][0]))/2
                r = int(img[i][j+1][0])
                b = int(img[i+1][j][0])
                # luma equation perceived option 2
                y = np.sqrt(0.299*r*r + 0.587*g*g + 0.114*b*b)
                # img[i][j] = y
                # img[i][j+1] = y
                # img[i+1][j] = y
                # img[i][j+1] = y
                debayer_img[int(i/2)][int(j/2)] = y
        end = time.time()
        print('time used: {}'.format(end-start))
        w_path = os.path.join('./calib_img', 'debayer-{}.jpg'.format(img_name))
        print('writing to {}'.format(w_path))
        rt = cv2.imwrite(w_path, debayer_img)
        print(rt)
    # cv2.destroyAllWindows()