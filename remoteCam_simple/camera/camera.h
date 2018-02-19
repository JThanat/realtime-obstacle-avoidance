struct buffer {
        void   *start;
        size_t  length;
};

typedef struct s_cameraState{
char             *dev_name;
int              fd;
struct buffer    *buffers;
unsigned int     n_buffers;
int              frame_count;
int width;
int height;
int bytePerPixel;
} cameraState;

cameraState* init_camera(char* dev_name, int w, int h, int frame_count, int num_buffers, int bytePerPixel);


void copyFrames(cameraState* cam, void* hostBuffer);	//copy frame(s) into buffer
void getBuffer(cameraState* cam, struct v4l2_buffer* buf);
int getBufferTimeOut(cameraState* cam, struct v4l2_buffer* buf, int timeout);
void pushBuffer(cameraState* cam, struct v4l2_buffer* buf);
void uninit_device(cameraState* cam);
int v4l2SetControl(cameraState* cam, int control, int value);
int v4l2GetControl (cameraState* cam, int control);
int v4l2ResetControl (cameraState* cam, int control);
