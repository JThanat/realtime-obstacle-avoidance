/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see https://linuxtv.org/docs.php for more information
 */

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include "camera.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

//parameterize it...
//const unsigned int bytePerPixel = 2;


/*
char             *dev_name;
int              fd = -1;
struct buffer    *buffers;
unsigned int     n_buffers;
int              frame_count = 5;
*/
static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
        int r;

        do {
                r = ioctl(fh, request, arg);
        } while (-1 == r && EINTR == errno);

        return r;
}

//From UVC capture 0.5
////////////////////////////////////////////////////////////////////////
static int
isv4l2Control (cameraState *cam, int control, struct v4l2_queryctrl *queryctrl)
{
  int err = 0;

  queryctrl->id = control;
  if ((err = ioctl (cam->fd, VIDIOC_QUERYCTRL, queryctrl)) < 0) {
    fprintf (stderr, "ioctl querycontrol error %d \n", errno);
  } else if (queryctrl->flags & V4L2_CTRL_FLAG_DISABLED) {
    fprintf (stderr, "control %s disabled \n", (char *) queryctrl->name);
  } else if (queryctrl->flags & V4L2_CTRL_TYPE_BOOLEAN) {
    return 1;
  } else if (queryctrl->type & V4L2_CTRL_TYPE_INTEGER) {
    return 0;
  } else {
    fprintf (stderr, "contol %s unsupported  \n", (char *) queryctrl->name);
  }
  return -1;
}

int v4l2SetControl(cameraState* cam, int control, int value)
{
  struct v4l2_control control_s;
  struct v4l2_queryctrl queryctrl;
  int min, max, step, val_def;
  int err;

  if (isv4l2Control (cam, control, &queryctrl) < 0)
	return -1;
  min = queryctrl.minimum;
  max = queryctrl.maximum;
  step = queryctrl.step;
  val_def = queryctrl.default_value;
  if ((value >= min) && (value <= max)) {
	control_s.id = control;
	control_s.value = value;
	if ((err = ioctl (cam->fd, VIDIOC_S_CTRL, &control_s)) < 0) {
	  fprintf (stderr, "ioctl set control error\n");
	  return -1;
	}
  }
  return 0;
}

int v4l2GetControl (cameraState* cam, int control)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_control control_s;
  int err;

  if (isv4l2Control (cam, control, &queryctrl) < 0)
    return -1;
  control_s.id = control;
  if ((err = ioctl (cam->fd, VIDIOC_G_CTRL, &control_s)) < 0) {
    fprintf (stderr, "ioctl get control error\n");
    return -1;
  }
  return control_s.value;
}

int v4l2ResetControl (cameraState* cam, int control)
{
  struct v4l2_control control_s;
  struct v4l2_queryctrl queryctrl;
  int val_def;
  int err;

  if (isv4l2Control (cam, control, &queryctrl) < 0)
    return -1;
  val_def = queryctrl.default_value;
  control_s.id = control;
  control_s.value = val_def;
  if ((err = ioctl (cam->fd, VIDIOC_S_CTRL, &control_s)) < 0) {
    fprintf (stderr, "ioctl reset control error\n");
    return -1;
  }

  return 0;
}
	
////////////////////////////////////////////////////////////////////////	


static int read_frame_to_buffer(cameraState* cam, void* target)
{
        struct v4l2_buffer buf;
        unsigned int i;

		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;

		if (-1 == xioctl(cam->fd, VIDIOC_DQBUF, &buf)) {
				switch (errno) {
				case EAGAIN:
						return 0;

				case EIO:
						/* Could ignore EIO, see spec. */

						/* fall through */

				default:
						errno_exit("VIDIOC_DQBUF11111");
				}
		}

		for (i = 0; i < cam->n_buffers; ++i)
				if (buf.m.userptr == (unsigned long)cam->buffers[i].start
					&& buf.length == cam->buffers[i].length)
						break;

		assert(i < cam->n_buffers);

		//process_image((void *)buf.m.userptr, buf.bytesused);
		//fwrite((void*)buf.m.userptr, buf.bytesused, 1, stdout);
		memcpy(target, (void*)buf.m.userptr, buf.bytesused);
		if (-1 == xioctl(cam->fd, VIDIOC_QBUF, &buf))
				errno_exit("VIDIOC_QBUF2222");

        return 1;
}


static void get_frame_buffer(cameraState* cam, struct v4l2_buffer* buf)
{
        //struct v4l2_buffer buf;
        unsigned int i;

		CLEAR(buf[0]);

		buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf->memory = V4L2_MEMORY_USERPTR;

		if (-1 == xioctl(cam->fd, VIDIOC_DQBUF, buf)) {
				switch (errno) {
				case EAGAIN:
						return ;

				case EIO:
						/* Could ignore EIO, see spec. */

						/* fall through */

				default:
						errno_exit("VIDIOC_DQBUF11111");
				}
		}
		
		for (i = 0; i < cam->n_buffers; ++i)
				if (buf->m.userptr == (unsigned long)cam->buffers[i].start
					&& buf->length == cam->buffers[i].length)
						break;

		assert(i < cam->n_buffers);
		
		return ;
}		
		
void pushBuffer(cameraState* cam, struct v4l2_buffer* buf){
			
		
		if (-1 == xioctl(cam->fd, VIDIOC_QBUF, buf))
				errno_exit("VIDIOC_QBUF2222");

        return ;
}



void copyFrames(cameraState* cam, void* hostBuffer)
{		
		void* hostBufferCurrent = hostBuffer;
        unsigned int count = cam->frame_count;
        while (count > 0) {
                for (;;) {
                        fd_set fds;
                        struct timeval tv;
                        int r;

                        FD_ZERO(&fds);
                        FD_SET(cam->fd, &fds);

                        /* Timeout. */
                        tv.tv_sec = 1200;
                        tv.tv_usec = 0;

                        r = select(cam->fd + 1, &fds, NULL, NULL, &tv);

                        if (-1 == r) {
                                if (EINTR == errno)
                                        continue;
                                errno_exit("select");
                        }

                        if (0 == r) {
                                fprintf(stderr, "select timeout\n");
                                exit(EXIT_FAILURE);
                        }

                        if (read_frame_to_buffer(cam, hostBufferCurrent))
                                break;
                        /* EAGAIN - continue select loop. */
                }
                count -- ;
                //fprintf(stderr, "added\n"); fflush(stderr);
                hostBufferCurrent = (void*)((char*)hostBufferCurrent+(cam->bytePerPixel*(cam->width)*(cam->height)));
        }
}

void getBuffer(cameraState* cam, struct v4l2_buffer* buf)
{		
	
        
			for (;;) {
				if(getBufferTimeOut(cam, buf, 5)!=0){
					return;
				}
				fprintf(stderr, "waiting for image\n"); fflush(stderr);
			}
}


int getBufferTimeOut(cameraState* cam, struct v4l2_buffer* buf, int timeout)
{		
	
      
    fd_set fds;
    struct timeval tv;
    int r;
    
    FD_ZERO(&fds);
    FD_SET(cam->fd, &fds);
    
    /* Timeout. */
    tv.tv_sec = timeout;
    tv.tv_usec = 0;
    
    r = select(cam->fd + 1, &fds, NULL, NULL, &tv);
    // nothing is return to me
    if (r == 0) {
      return 0;
    }
    if (-1 == r) {
      if (EINTR == errno) {

	fprintf(stderr, "Faltal error, select should never return -1 unless there is an error in the stream\n");
	//continue;
	//      errno_exit("select");
      }
    }
    fprintf(stderr, "getting buff\n"); fflush(stderr);
    get_frame_buffer(cam, buf);
    return 1;
}



void uninit_device(cameraState* cam)
{
		enum v4l2_buf_type type;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == xioctl(cam->fd, VIDIOC_STREAMOFF, &type))
				errno_exit("VIDIOC_STREAMOFF");
	
        unsigned int i;

		for (i = 0; i < cam->n_buffers; ++i)
			free(cam->buffers[i].start);
			
        free(cam->buffers);
        
        
		if (-1 == close(cam->fd))
                errno_exit("close");
        cam->fd = -1;
        free(cam);
}


static void init_userp(unsigned int buffer_size, cameraState* cam)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = cam->n_buffers;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(cam->fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                 "user pointer i/o\n", cam->dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        cam->buffers = (struct buffer*)calloc(cam->n_buffers, sizeof(*cam->buffers));//////////

        if (!cam->buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
		int n_buffers;
        for (n_buffers = 0; n_buffers < cam->n_buffers; ++n_buffers) {
                cam->buffers[n_buffers].length = buffer_size;
                
                //buffers[n_buffers].start = malloc(buffer_size);
                //align memory, must be page aligned
                cam->buffers[n_buffers].start = (void*)memalign(getpagesize(), buffer_size);

                if (!cam->buffers[n_buffers].start) {
                        fprintf(stderr, "Out of memory\n");
                        exit(EXIT_FAILURE);
                }
        }
}

cameraState* init_camera(char* dev_name, int w, int h, int frame_count, int num_buffers, int bytePerPixel)
{
		
		cameraState* cam = (cameraState*) malloc(sizeof(cameraState));
		cam->dev_name = dev_name;
		cam->fd = -1;
		cam->buffers = NULL;
		cam->n_buffers = num_buffers;
		cam->frame_count = frame_count;
		cam->width = w;
		cam->height = h;
		cam->bytePerPixel = bytePerPixel;
		 struct stat st;

        if (-1 == stat(cam->dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         cam->dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", cam->dev_name);
                exit(EXIT_FAILURE);
        }

        cam->fd = open(cam->dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == cam->fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         cam->dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
	
	
	
	
        struct v4l2_capability cap;
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        struct v4l2_format fmt;
        unsigned int min;

        if (-1 == xioctl(cam->fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s is no V4L2 device\n",
                                 cam->dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_QUERYCAP");
                }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                fprintf(stderr, "%s is no video capture device\n",
                         cam->dev_name);
                exit(EXIT_FAILURE);
        }

		
		if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                        fprintf(stderr, "%s does not support streaming i/o\n",
                                 cam->dev_name);
                        exit(EXIT_FAILURE);
		}

        /* Select video input, video standard and tune here. */


        CLEAR(cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == xioctl(cam->fd, VIDIOC_CROPCAP, &cropcap)) {
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect; /* reset to default */

                if (-1 == xioctl(cam->fd, VIDIOC_S_CROP, &crop)) {
                        switch (errno) {
                        case EINVAL:
                                /* Cropping not supported. */
                                break;
                        default:
                                /* Errors ignored. */
                                break;
                        }
                }
        } else {
                /* Errors ignored. */
        }


        CLEAR(fmt);

        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt.fmt.pix.width       = w;
		fmt.fmt.pix.height      = h;
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
		//fmt.fmt.pix.field       = V4L2_FIELD_NONE;

		if (-1 == xioctl(cam->fd, VIDIOC_S_FMT, &fmt))
				errno_exit("VIDIOC_S_FMT");

		/* Note VIDIOC_S_FMT may change width and height. */


        /* Buggy driver paranoia. */
        min = fmt.fmt.pix.width * 2;
        if (fmt.fmt.pix.bytesperline < min)
                fmt.fmt.pix.bytesperline = min;
        min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        if (fmt.fmt.pix.sizeimage < min)
                fmt.fmt.pix.sizeimage = min;
		//debug, check w, h, size
		//fprintf(stderr, "width:%d\nheight:%d\nsize:%d\n", fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);
		
		
		init_userp(fmt.fmt.pix.sizeimage, cam);
		
		
        unsigned int i;
        enum v4l2_buf_type type;

        for (i = 0; i < cam->n_buffers; ++i) {
			struct v4l2_buffer buf;

			CLEAR(buf);
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_USERPTR;
			buf.index = i;
			buf.m.userptr = (unsigned long)cam->buffers[i].start;
			buf.length = cam->buffers[i].length;

			if (-1 == xioctl(cam->fd, VIDIOC_QBUF, &buf))
					errno_exit("VIDIOC_QBUF33333");
		}
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == xioctl(cam->fd, VIDIOC_STREAMON, &type))
				errno_exit("VIDIOC_STREAMON");
        
		return cam;
		

}
