# scp -r ubuntu@192.168.30.50:/home/ubuntu/cameraDev/export_img /home/thanat/obstacle_avoidance/data/camera
# https://stackoverflow.com/questions/596216/formula-to-determine-brightness-of-rgb-color
import numpy as np
import cv2

if __name__ == '__main__':
    img = cv2.imread('./data/camera/export_img/img-1.jpg')
    
    h, w, c = img.shape

    # bayer pattern
    # g r 
    # b g
    for i in range(0,h,2):
        for j in range(0,w,2):
            g = (int(img[i][j][0]) + int(img[i+1][j+1][0]))/2
            r = int(img[i][j+1][0])
            b = int(img[i+1][j][0])
            # luma equation perceived option 2
            y = np.sqrt(0.299*r*r + 0.587*g*g + 0.114*b*b)
            # print(y)
            img[i][j] = y
            img[i][j+1] = y
            img[i+1][j] = y
            img[i][j+1] = y
    img = cv2.resize(img, (int(w/2),int(h/2)))
    cv2.imshow('img',img)
    cv2.imwrite('intensity2.png', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
