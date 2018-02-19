#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>

#include "camera.h"
#include <linux/videodev2.h>
int main(int argc, char **argv)
{
		char* dev_name = "/dev/video0";
        cameraState* c1 = init_camera(dev_name, 1216, 920, 1, 3, 2);
        
        //v4l2ctrl thingies
        int control_val;
        control_val = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
        fprintf(stderr, "old value:%d\n", control_val);
        v4l2SetControl(c1, V4L2_CID_EXPOSURE, 10);
        control_val = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
        fprintf(stderr, "set value:%d\n", control_val);
        v4l2ResetControl(c1, V4L2_CID_EXPOSURE);
        control_val = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
        fprintf(stderr, "reset value:%d\n", control_val);
        void* hostBuffer = malloc(c1->width*c1->height*c1->bytePerPixel*5);
        //copyFrames(c1, hostBuffer);
        struct v4l2_buffer buff1;
        getBuffer(c1, &buff1);
        fwrite(buff1.m.userptr, c1->bytePerPixel, c1->width*c1->height, stdout);
        pushBuffer(c1, &buff1);
        fflush(stdout);
        //getBuffer()
        uninit_device(c1);
        return 0;
}
