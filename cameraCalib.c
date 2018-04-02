#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>

#include "camera.h"
#include <linux/videodev2.h>
int main(int argc, char **argv)
{
        char* dev_name = "/dev/video1";
        char* dev_name2 = "/dev/video0";
        cameraState* c1 = init_camera(dev_name, 2432, 1842, 1, 3, 2);
        cameraState* c2 = init_camera(dev_name2, 2432, 1842, 1,3, 2);
	//resolution
	//2432x1842
	//4896x3684
	
	

	//V4L2_CID_EXPOSURE
	//V4L2_CID_GAIN
	

        //v4l2ctrl thingies
        int control_val, control_val2;
        control_val = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
        fprintf(stderr, "old value:%d\n", control_val);
        v4l2SetControl(c1, V4L2_CID_EXPOSURE, 2.5);
        control_val = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
        fprintf(stderr, "set value:%d\n", control_val);
        v4l2ResetControl(c1, V4L2_CID_EXPOSURE);
        control_val = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
        fprintf(stderr, "reset value:%d\n", control_val);

	int conrtrol_val2 = v4l2GetControl(c2, V4L2_CID_EXPOSURE);
	fprintf(stderr, "old value:%d\n", control_val2);
        v4l2SetControl(c2, V4L2_CID_EXPOSURE, 2.5);
        control_val2 = v4l2GetControl(c2, V4L2_CID_EXPOSURE);
        fprintf(stderr, "set value:%d\n", control_val2);
        v4l2ResetControl(c2, V4L2_CID_EXPOSURE);
        control_val2 = v4l2GetControl(c2, V4L2_CID_EXPOSURE);
        fprintf(stderr, "reset value:%d\n", control_val2);

       



	void* hostBuffer = malloc(c1->width*c1->height*c1->bytePerPixel*5);
	void* hostBuffer2 = malloc(c2->width*c2->height*c2->bytePerPixel*5);
        
        struct v4l2_buffer buff1;
	struct v4l2_buffer buff2;
       	int k;
	for(k=0;k<35;k++){
                fprintf(stderr, "Waiting 3 secs before next image...")
                delay(3000)
                fprintf(stderr, "Starting Round: %d", k+1)
                char *leftName = (char*)malloc(11 * sizeof(char));
                char *rightName = (char*)malloc(11 * sizeof(char));
                sprintf(leftName, "%s%d.raw", "left", k+1);
                sprintf(rightName, "%s%d.raw", "left", k+1);

                getBuffer(c1, &buff1);
                getBuffer(c2, &buff2);

                FILE *f1 = fopen(leftName, "w");
                FILE *f2 = fopen(rightName, "w");

                fwrite(buff1.m.userptr, c1->bytePerPixel, c1->width*c1->height, f1);
                fwrite(buff2.m.userptr, c2->bytePerPixel, c2->width*c2->height, f2);

                pushBuffer(c1, &buff1);
                pushBuffer(c2, &buff2);

                fflush(stdout);
	}
        //getBuffer()
        uninit_device(c1);
	uninit_device(c2);
        return 0;
}
