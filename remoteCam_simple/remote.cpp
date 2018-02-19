#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include "camera/camera.h"
#include "socklib/socklib.h"
#include "socketHelper.h"
#include "liblz4/lz4.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include "remoteCam.h"
#include <unistd.h>
#include <getopt.h>
#include <syslog.h>
#include <math.h>
//magic number!
const int BYTESPERPIXEL = 2;

//evil global for closing camera...
char* spidev_g = NULL;
cameraState* camera_g = NULL;
int isSPIMaster=0;





void printError(char* errMsg){
	fprintf(stderr,"%s\n", errMsg);
	fprintf(stderr, "exiting\n");
	fflush(stderr);
	exit(0);
}
void bayerDownsample(uint16_t* imgBuff, uint16_t* imgOut, int width, int height, int scalingFactor, int endWidth, int endHeight){
	int i, j, a;
	a=0;
	int rowSize = endWidth/scalingFactor;
	//subsample group of RGGB in each of inner loop
	for(i=0; i<endHeight; i+=scalingFactor*2){
		for(j=0; j<endWidth; j+=scalingFactor*2){
			imgOut[a] = imgBuff[i*width + j];
			imgOut[a+1] = imgBuff[i*width + j +1];
			imgOut[a+rowSize] = imgBuff[((i+1)*width) + j];
			imgOut[a+rowSize+1] = imgBuff[((i+1)*width) + j+1];
			a+=2;
		}
		a+=rowSize;
	}
	return;
}
//a=a+b
void addImage(uint16_t* a, uint16_t*b, int numpix){
	int i;
	for(i=0;i<numpix;i++){
		a[i]=a[i]+b[i];
	}
}

//a=a+c and b=b+c^2
void addImageSQ32(uint32_t* a, uint32_t* b, uint16_t* c, int numpix){
	int i;
	for(i=0;i<numpix;i++){
		a[i] = a[i] + c[i];
		b[i] = b[i]+((uint32_t)c[i]*(uint32_t)c[i]);
	}
}

int enableSync(int en, char* spidevName){
	int exposure = 2;
	int triggerDelay = 0;
	uint8_t mode=0;
	uint8_t bits = 8;
	uint32_t speed = 25000000/4;
	uint16_t delay=0;
	int ret = 0;
	int fd;
	char* gpioName = "/sys/class/gpio/gpio237/value";
	//pull SPI_CS low
	
	int gpio_fd=-1;
	
	gpio_fd = fopen(gpioName, "w");
	if(gpio_fd==NULL){
		printError("Can't open GPIO, you may not have enough privilege");
	}
	
	fwrite("0", sizeof(char), 1, gpio_fd);
	fflush(gpio_fd);
	fclose(gpio_fd);
	usleep(20000);	//wait for 20 milliseconds
	
	fd = open(spidevName, O_RDWR);
	if (fd < 0)
		printError("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		printError("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		printError("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		printError("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		printError("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printError("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printError("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	//buffers
	uint8_t* tx = (uint8_t*) malloc(4*sizeof(uint8_t));
	//exposure should not be more than 0xFFF
	if(exposure>0xFFF)
	{
		printError("exposure overflow, max is 0xFFF\n");
	}
	if(exposure<1)
	{
		printError("minimum exposure is 0x001\n");
	}
	
	uint8_t expoHigh = exposure >> 8;
	uint8_t expoLow = exposure %256;
	uint8_t triggerDelayReg = triggerDelay%256;
	
	fprintf(stderr, "triggerDelay:%d\n", triggerDelayReg);
	fflush(stderr);
	if(en)	tx[0] = 0xc0;
	else	tx[0] = 0xb0;
	tx[1]=triggerDelayReg;	//triggerDelay
	tx[2]=expoHigh;	// tx[2][3:0] is exposure[11:8].
	tx[3]=expoLow; // tx[3][7:0] is exposure[7:0].
	uint8_t* rx = (uint8_t*) malloc(4*sizeof(uint8_t));
	struct spi_ioc_transfer tr{};
		tr.tx_buf = (unsigned long)tx;
		tr.rx_buf = (unsigned long)rx;
		tr.len = 4;
		tr.delay_usecs = delay;
		tr.speed_hz = speed;
		tr.bits_per_word = bits;
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	//to do add verification
	
	
	////////////////////////

	

	close(fd);
	free(tx);
	free(rx);
	usleep(20000);	//wait for 20 milliseconds
	
	gpio_fd=-1;
	
	gpio_fd = fopen(gpioName, "w");
	if(gpio_fd==NULL){
		printError("Can't open GPIO, you may not have enough privilege");
	}
	
	fwrite("1", sizeof(char), 1, gpio_fd);
	fflush(gpio_fd);
	fclose(gpio_fd);
	usleep(20000);	//wait for 20 milliseconds
	
	
}

int setSyncExposure(int exposure, int triggerDelay, char* spidevName){
	
	uint8_t mode=0;
	uint8_t bits = 8;
	uint32_t speed = 25000000/4;
	uint16_t delay=0;
	int ret = 0;
	int fd;
	char* gpioName = "/sys/class/gpio/gpio237/value";
	//pull SPI_CS low
	
	int gpio_fd=-1;
	
	gpio_fd = fopen(gpioName, "w");
	if(gpio_fd==NULL){
		printError("Can't open GPIO, you may not have enough privilege");
	}
	
	fwrite("0", sizeof(char), 1, gpio_fd);
	fflush(gpio_fd);
	fclose(gpio_fd);
	usleep(20000);	//wait for 20 milliseconds
	
	fd = open(spidevName, O_RDWR);
	if (fd < 0)
		printError("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		printError("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		printError("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		printError("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		printError("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printError("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printError("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	//buffers
	uint8_t* tx = (uint8_t*) malloc(4*sizeof(uint8_t));
	//exposure should not be more than 0xFFF
	if(exposure>0xFFF)
	{
		printError("exposure overflow, max is 0xFFF\n");
	}
	if(exposure<1)
	{
		printError("minimum exposure is 0x001\n");
	}
	
	uint8_t expoHigh = exposure >> 8;
	uint8_t expoLow = exposure %256;
	uint8_t triggerDelayReg = triggerDelay%256;
	
	fprintf(stderr, "triggerDelay:%d\n", triggerDelayReg);
	fflush(stderr);
	
	tx[0]=0x10;	//command, 0x10 is set exposure.
	tx[1]=triggerDelayReg;	//triggerDelay
	tx[2]=expoHigh;	// tx[2][3:0] is exposure[11:8].
	tx[3]=expoLow; // tx[3][7:0] is exposure[7:0].
	uint8_t* rx = (uint8_t*) malloc(4*sizeof(uint8_t));
	struct spi_ioc_transfer tr{};
		tr.tx_buf = (unsigned long)tx;
		tr.rx_buf = (unsigned long)rx;
		tr.len = 4;
		tr.delay_usecs = delay;
		tr.speed_hz = speed;
		tr.bits_per_word = bits;
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	//to do add verification
	
	
	////////////////////////

	

	close(fd);
	free(tx);
	free(rx);
	usleep(20000);	//wait for 20 milliseconds
	
	gpio_fd=-1;
	
	gpio_fd = fopen(gpioName, "w");
	if(gpio_fd==NULL){
		printError("Can't open GPIO, you may not have enough privilege");
	}
	
	fwrite("1", sizeof(char), 1, gpio_fd);
	fflush(gpio_fd);
	fclose(gpio_fd);
	usleep(20000);	//wait for 20 milliseconds
	
	
}

flushBuffer(char* spidevName, int numFlush){
	uint8_t mode=0;
	uint8_t bits = 8;
	uint32_t speed = 25000000/4;
	uint16_t delay=0;
	int ret = 0;
	int fd;
	int exposure = 2;
	char* gpioName = "/sys/class/gpio/gpio237/value";
	
	int gpio_fd=-1;
	//pull gpio high in case it was previously low
	gpio_fd = fopen(gpioName, "w");
	if(gpio_fd==NULL){
		printError("Can't open GPIO, you may not have enough privilege");
	}
	
	fwrite("1", sizeof(char), 1, gpio_fd);
	fflush(gpio_fd);
	fclose(gpio_fd);
	usleep(20000);	//wait for 20 milliseconds
	
	
	fd = open(spidevName, O_RDWR);
	if (fd < 0)
		printError("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		printError("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		printError("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		printError("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		printError("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printError("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		printError("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	//buffers
	uint8_t* tx = (uint8_t*) malloc(4*sizeof(uint8_t));
	uint8_t expoHigh = exposure >> 8;
	uint8_t expoLow = exposure %256;
	uint8_t* rx = (uint8_t*) malloc(4*sizeof(uint8_t));
	struct spi_ioc_transfer tr{};
	
	

	int i;
	for(i=0;i<numFlush;i++){
	
		gpio_fd=-1;
		
		gpio_fd = fopen(gpioName, "w");
		if(gpio_fd==NULL){
			printError("Can't open GPIO, you may not have enough privilege");
		}
		
		fwrite("0", sizeof(char), 1, gpio_fd);
		fflush(gpio_fd);
		fclose(gpio_fd);
		
		
		usleep(20000);	//wait for 20 milliseconds
		
		
		tx[0]=0x90;	//command, 0x90 is set trigger.
		tx[1]=0x00;	//trigger offset
		tx[2]=expoHigh;	// tx[2][3:0] is exposure[11:8].
		tx[3]=expoLow; // tx[3][7:0] is exposure[7:0].
			tr.tx_buf = (unsigned long)tx;
			tr.rx_buf = (unsigned long)rx;
			tr.len = 4;
			tr.delay_usecs = delay;
			tr.speed_hz = speed;
			tr.bits_per_word = bits;
		ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
		usleep(20000);	//wait for 20 milliseconds
		
		gpio_fd=-1;
		
		gpio_fd = fopen(gpioName, "w");
		if(gpio_fd==NULL){
			printError("Can't open GPIO, you may not have enough privilege");
		}
		
		fwrite("1", sizeof(char), 1, gpio_fd);
		fflush(gpio_fd);
		fclose(gpio_fd);
		
		openlog ("remote", LOG_PID, LOG_USER);
		syslog (LOG_INFO, "SPI sent");
		closelog ();
		
		usleep(250000);	//250ms to for readout
	}
	
	//to do add verification
	
	
	////////////////////////

	free(tx);
	free(rx);

	close(fd);
}



//sigpipe handler
void sigPipeHandler(){
	//flush buffer, close camera and exit
	if((spidev_g!=NULL)&&isSPIMaster){
		flushBuffer(spidev_g, 11);
	}
	if(camera_g!=NULL){
		uninit_device(camera_g);
	}
		
	
	fprintf(stderr, "server disconnected, exiting\n"); fflush(stderr);
	exit(0);
}
#define I2C_SETUP_DELAY     5000000    // microsecond
#define CAMERA_SPI_DELAY    1          // second
#define RETRY_DELAY         1000000	   // second

void preview(int comm_socket, int width, int height, captureInfo* cinfo, char* devName, int isSPIMaster, char* spidevName){
	
	int32_t exposure = cinfo->exposure;
	int32_t gain = cinfo-> gain;
	int32_t grr = cinfo->grr;
	int32_t multiexposure = cinfo->multiexposure;
	int32_t triggerDelay = cinfo->triggerDelay;
	int numberOfBuffer = 3;	//minimize latency
	int bytesSent=0;
	int totalSent=0;
	int r=0;
	int32_t sendSize;
	
	//work around for preview resolution
	int finalWidth = width;
	int finalHeight = height;
	int endWidth = width;
	int endHeight = height;
	char* convertBuff = NULL;
	int softdownsamp =0;
	
	if(width == 1216 && height == 920){
		endWidth = 2432;
		endHeight = 1840;
		width = 2432;
		height = 1842;
		convertBuff = (char*)malloc(sizeof(uint16_t)*1216*920);
		softdownsamp=1;
	}
	////////////////////////////////////
	
	cameraState* c1 = init_camera(devName, width, height, 1, numberOfBuffer, BYTESPERPIXEL);	//#################################
	camera_g = c1;
	//buffer for multi exposure
	uint16_t* multiBuffer = (uint16_t*)malloc(finalWidth*finalHeight*sizeof(uint16_t));
	
	
	//v4l2 settings
	int controlStatus = 0;
	controlStatus = v4l2SetControl(c1, V4L2_CID_EXPOSURE, exposure);	//#################################
	if(controlStatus<0){
		fprintf(stderr, "set exposure error\n"); fflush(stderr);
		exit(0);
	}
	v4l2SetControl(c1, V4L2_CID_GAIN, gain);	//#################################
	if(controlStatus<0){
		fprintf(stderr, "set gain error\n"); fflush(stderr);
		exit(0);
	}
	v4l2SetControl(c1, V4L2_CID_CONTRAST, grr);	//#################################
	if(controlStatus<0){
		fprintf(stderr, "set globalReset error\n"); fflush(stderr);
		exit(0);
	}
	///////////////
	
	
	struct v4l2_buffer buff1;
	
	//flush 1 frame if in sync mode
	
	if(grr){
		//wait for camera setup
		usleep(I2C_SETUP_DELAY);
		if(1/*isSPIMaster*/){	//assume that only one camera is in preview mode
			enableSync(1, spidevName);
			setSyncExposure(exposure, triggerDelay, spidevName);
		}
	}else{
		if(1/*isSPIMaster*/)enableSync(0, spidevName);		
	}
	sendHeader(comm_socket, HEADER_CAMERA_READY);
	while(1){
		int exposed_multi;
		//clear multiexp buffer
		memset(multiBuffer, 0x00, finalWidth*finalHeight*sizeof(uint16_t));
		for(exposed_multi = 0; exposed_multi<multiexposure ; exposed_multi++){
			getBuffer(c1, &buff1);	//#################################
			bytesSent=0;
			totalSent=0;
			char* toSend = buff1.m.userptr;
			sendSize = buff1.bytesused;
			//fprintf(stderr, "waiting header\n"); fflush(stderr);
			//if a header is coming, handle it...
			int32_t header;
			if(0<peekHeader_nowait(comm_socket, &header)){
				header = recvHeader(comm_socket);
				//if header is setting, change setting and startover
				fprintf(stderr, "changing settings\n"); fflush(stderr);
				if(HEADER_PREVIEW_SETTING==header){
					int32_t action=0;
					recvBytes(comm_socket, (char*)&action, sizeof(action));
					//do change setting
					int32_t settingValue;
					int32_t settingType = SETTING_NONE;
					switch(action){
						case EXPOSURE_INC: 
							settingType = EXPOSURE_SETTING;
							settingValue = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
							settingValue++;
							v4l2SetControl(c1, V4L2_CID_EXPOSURE, settingValue);
							settingValue = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
							sendHeader(comm_socket, HEADER_SETTING_RETURN);
							sendBytes(comm_socket, (char*)&settingType, sizeof(settingType));
							sendBytes(comm_socket, (char*)&settingValue, sizeof(settingValue));
							break;
						
						case EXPOSURE_DEC: 
							settingType = EXPOSURE_SETTING;
							settingValue = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
							settingValue--;
							v4l2SetControl(c1, V4L2_CID_EXPOSURE, settingValue);
							settingValue = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
							sendHeader(comm_socket, HEADER_SETTING_RETURN);
							sendBytes(comm_socket, (char*)&settingType, sizeof(settingType));
							sendBytes(comm_socket, (char*)&settingValue, sizeof(settingValue));
							break;
						
						case GAIN_INC: 
							settingType = GAIN_SETTING;
							settingValue = v4l2GetControl(c1, V4L2_CID_GAIN);
							settingValue++;
							v4l2SetControl(c1, V4L2_CID_GAIN, settingValue);
							settingValue = v4l2GetControl(c1, V4L2_CID_GAIN);
							sendHeader(comm_socket, HEADER_SETTING_RETURN);
							sendBytes(comm_socket, (char*)&settingType, sizeof(settingType));
							sendBytes(comm_socket, (char*)&settingValue, sizeof(settingValue));
							break;
						
						case GAIN_DEC: 
							settingType = GAIN_SETTING;
							settingValue = v4l2GetControl(c1, V4L2_CID_GAIN);
							settingValue--;
							v4l2SetControl(c1, V4L2_CID_GAIN, settingValue);
							settingValue = v4l2GetControl(c1, V4L2_CID_GAIN);
							sendHeader(comm_socket, HEADER_SETTING_RETURN);
							sendBytes(comm_socket, (char*)&settingType, sizeof(settingType));
							sendBytes(comm_socket, (char*)&settingValue, sizeof(settingValue));
							break;
						default:
						fprintf(stderr, "invalid setting, exiting\n"); fflush(stderr); exit(0);
					}
					
					
				}else {
					fprintf(stderr, "header invalid, should recieve PREVIEW_SETTING, exiting\n"); fflush(stderr);
					exit(0);
				}
			}
			///////////////////////////////////////////////////
			
			
			//do size conversion, preview resolution workaround
			if(softdownsamp){
				bayerDownsample((uint16_t*)buff1.m.userptr, (uint16_t*)convertBuff, 2432, 1842, 2, 2432, 1840);	
				sendSize = BYTESPERPIXEL*1216*920;
				toSend = convertBuff;
			}
			///////////////////////////////////////////////////
			//do multiexposure thingies
			//fprintf(stderr, "adding images\n"); fflush(stderr);
			addImage(multiBuffer, (uint16_t*)toSend, finalWidth*finalHeight);
			
			
			//fprintf(stderr, "sent:%d\n", totalSent); fflush(stderr);
			//fwrite(toSend, 1, sendSize, stdout);
			//fflush(stdout);
			pushBuffer(c1, &buff1);
		}
		//send actual image
		sendHeader(comm_socket, HEADER_CAPSTREAM);
		sendBytes(comm_socket, (char*)multiBuffer, sendSize);
	}
	if(grr /*&& isSPIMaster*/){	//assume that only one camera is in preview mode
		flushBuffer(spidevName, numberOfBuffer+1);
		enableSync(0, spidevName);
	}
	uninit_device(c1);	//#################################
	free(multiBuffer);
	if(width == 1216 && height == 920){
		free(convertBuff);
	}
	return;
}

#define NMODIMGS     22

void streamFrame(int comm_socket, int width, int height, captureInfo* cinfo, char* devName, int isSPIMaster, char* spidevName){
	
	int32_t mode = cinfo->mode;
	int32_t number = cinfo->number;
	int32_t exposure = cinfo->exposure;
	int32_t gain = cinfo->gain;
	int32_t grr = cinfo->grr;
	int32_t multiexposure = cinfo->multiexposure;
	int32_t triggerDelay = cinfo->triggerDelay;
	int32_t highPrecisionMode = cinfo->highPrecisionMode;
	//MAGIC NUMBER!!!, deal with this later
	int numberOfBuffer = 4;
	
	
	//work around for preview resolution
	int finalWidth = width;
	int finalHeight = height;
	int endWidth = width;
	int endHeight = height;
	char* convertBuff = NULL;
	int softdownsamp =0;
	
	if(width == 1216 && height == 920){
		endWidth = 2432;
		endHeight = 1840;
		width = 2432;
		height = 1842;
		softdownsamp=1;
		convertBuff = (char*)malloc(sizeof(uint16_t)*1216*920);
	}
	////////////////////////////////////
	
	
	int r=0;
	int32_t sendSize;
	cameraState* c1 = init_camera(devName, width, height, 1, numberOfBuffer, BYTESPERPIXEL);
	camera_g = c1;
	//buffer for multi exposure
	char* multiBuffer;
	char* multiBufferSQ;
	if(highPrecisionMode>0){
		multiBuffer = (char*)malloc(finalWidth*finalHeight*sizeof(uint32_t));
		if(highPrecisionMode==2){
			multiBufferSQ = (char*)malloc(finalWidth*finalHeight*sizeof(uint32_t));
		}
	}else{
		multiBuffer = (char*)malloc(finalWidth*finalHeight*sizeof(uint16_t));
	}
	//~ CudaSolver::cudaSolverIn  *tmpIn = NULL;
	//~ if(mode==3){
		//~ tmpIn = new CudaSolver::cudaSolverIn[width*height];
	//~ }
	//v4l2 settings
	int controlStatus = 0;
	controlStatus = v4l2SetControl(c1, V4L2_CID_EXPOSURE, exposure);
	if(controlStatus<0){
		fprintf(stderr, "set exposure error\n"); fflush(stderr);
		exit(0);
	}
	v4l2SetControl(c1, V4L2_CID_GAIN, gain);
	if(controlStatus<0){
		fprintf(stderr, "set gain error\n"); fflush(stderr);
		exit(0);
	}
	v4l2SetControl(c1, V4L2_CID_CONTRAST, grr);
	if(controlStatus<0){
		fprintf(stderr, "set globalReset error\n"); fflush(stderr);
		exit(0);
	}
	
	struct v4l2_buffer buff1;
	
	if(grr){
		//wait for camera setup
		usleep(I2C_SETUP_DELAY);
		if(isSPIMaster){
			enableSync(1, spidevName);
			//~ flushBuffer(spidevName, 1);
			setSyncExposure(exposure, triggerDelay, spidevName);
		}
		while(getBufferTimeOut( c1, &buff1, CAMERA_SPI_DELAY) == 0) {
		  // Here is a bug: If the primary cam finish
		  // everything with no problem, but the secondary has
		  // trouble, then it will have trouble sending "sync" signals
		  // Maybe ,the trigger for clearing should depends on which signal it is being sent from, synchronous
		  fprintf(stderr, "resending trigger\n"); fflush(stderr);
		  if(isSPIMaster){ 
			enableSync(1, spidevName);
			flushBuffer(spidevName, 1);
			setSyncExposure(exposure, triggerDelay, spidevName);
			usleep(RETRY_DELAY);
		    }
		}		
		//getBuffer(c1, &buff1);
		pushBuffer(c1, &buff1);
	}else{
		if(isSPIMaster)enableSync(0, spidevName);		
	}
	//getBufferTimeOut( c1, &buff1, CAMERA_SPI_DELAY);
	sendHeader(comm_socket, HEADER_CAMERA_READY);
	if(mode==1||mode==3){
		int i=0;
		while(i<number){
			int exposed_multi;
			//clear multiexp buffer
			if(highPrecisionMode>0){
				memset(multiBuffer, 0x00, finalWidth*finalHeight*sizeof(uint32_t));
				memset(multiBufferSQ, 0x00, finalWidth*finalHeight*sizeof(uint32_t));
			}else{
				memset(multiBuffer, 0x00, finalWidth*finalHeight*sizeof(uint16_t));
				if(mode==3){
					
					}
			}
			for(exposed_multi = 0; exposed_multi<multiexposure ; exposed_multi++){
				getBuffer(c1, &buff1);
				char* toSend = buff1.m.userptr;
				sendSize = buff1.bytesused;
				
				//do size conversion, preview resolution workaround
				if(softdownsamp){
					bayerDownsample(buff1.m.userptr, (uint16_t*)convertBuff, 2432, 1842, 2, 2432, 1840);	
					sendSize = BYTESPERPIXEL*1216*920;
					toSend = convertBuff;
				}
				/////////////////////////////////////////////////////
				if(highPrecisionMode>0){
					addImageSQ32((uint32_t*)multiBuffer, (uint32_t*)multiBufferSQ, (uint16_t*)toSend, finalWidth*finalHeight);
				}else{
					addImage((uint16_t*)multiBuffer, (uint16_t*)toSend, finalWidth*finalHeight);
				}
				pushBuffer(c1, &buff1);
				fprintf(stderr, "."); fflush(stderr);
			}
			
			if(highPrecisionMode>0){
				sendSize = finalHeight*finalWidth*sizeof(uint32_t);
				sendHeader(comm_socket, HEADER_CAPSTREAM_32);
				sendBytes(comm_socket, multiBuffer, sendSize);
				if(highPrecisionMode==2){
					sendHeader(comm_socket, HEADER_CAPSTREAM_32);
					sendBytes(comm_socket, multiBufferSQ, sendSize);
				}
			}else{
				
					sendSize = finalHeight*finalWidth*sizeof(uint16_t);
					sendHeader(comm_socket, HEADER_CAPSTREAM);
					sendBytes(comm_socket, multiBuffer, sendSize);
				
			}
			i++;
		}
		if(mode==3){
			//do something here
			
		}
		fprintf(stderr, "-\n"); fflush(stderr);
	}else{
		int sizeOfTxBuffer=0;
		char* txBuffer;
		char* txBuffer2;
		if(highPrecisionMode>0){
			sizeOfTxBuffer = sizeof(uint32_t)*width*height*2;
			txBuffer = (char*)malloc(sizeOfTxBuffer);
			txBuffer2 = (char*)malloc(sizeOfTxBuffer);
			
		}else{	
			sizeOfTxBuffer = sizeof(uint16_t)*width*height*2;
			txBuffer = (char*)malloc(sizeOfTxBuffer);
		}
		 
		int i=0;
		
		
		while(i<number){
			int exposed_multi;
			//clear multiexp buffer
			if(highPrecisionMode>0){
				memset(multiBuffer, 0x00, finalWidth*finalHeight*sizeof(uint32_t));
				memset(multiBufferSQ, 0x00, finalWidth*finalHeight*sizeof(uint32_t));
			}else{
				memset(multiBuffer, 0x00, finalWidth*finalHeight*sizeof(uint16_t));
			}
			for(exposed_multi = 0; exposed_multi<multiexposure ; exposed_multi++){
				getBuffer(c1, &buff1);
				char* toSend = buff1.m.userptr;
				sendSize = buff1.bytesused;
				//do size conversion, preview resolution workaround
				if(softdownsamp){
					bayerDownsample(buff1.m.userptr, (uint16_t*)convertBuff, 2432, 1842, 2, 2432, 1840);	
					sendSize = BYTESPERPIXEL*1216*920;
					toSend = convertBuff;
				}
				/////////////////////////////////////////////////////
				if(highPrecisionMode>0){
					addImageSQ32((uint32_t*)multiBuffer, (uint32_t*)multiBufferSQ, (uint16_t*)toSend, finalWidth*finalHeight);
				}else{
					addImage((uint16_t*)multiBuffer, (uint16_t*)toSend, finalWidth*finalHeight);
				}
				pushBuffer(c1, &buff1);
			}
			
			int sendSize2;
			if(highPrecisionMode>0){			
				sendSize = LZ4_compress_default((char*)multiBuffer, txBuffer, finalWidth*finalHeight*sizeof(uint32_t), sizeOfTxBuffer);
				if(highPrecisionMode==2)sendSize2 = LZ4_compress_default((char*)multiBufferSQ, txBuffer2, finalWidth*finalHeight*sizeof(uint32_t), sizeOfTxBuffer);
				if(sendSize<0){
					fprintf(stderr, "compress error\n");
					fflush(stderr);
				}
			}else{
				sendSize = LZ4_compress_default((char*)multiBuffer, txBuffer, finalWidth*finalHeight*sizeof(uint16_t), sizeOfTxBuffer);
				if(sendSize<0){
					fprintf(stderr, "compress error\n");
					fflush(stderr);
				}
			}
			int size_sendSize = sizeof(sendSize);
			
			
			//send compressed image size
			sendHeader(comm_socket, HEADER_CAPSTREAM_COMPRESSED);
			sendBytes(comm_socket, (char*)&sendSize, size_sendSize);
			
			//send actual compressed image
			sendBytes(comm_socket, (char*)txBuffer, sendSize);
			
			if(highPrecisionMode==2){
				//send compressed image size
				sendHeader(comm_socket, HEADER_CAPSTREAM_COMPRESSED);
				sendBytes(comm_socket, (char*)&sendSize2, size_sendSize);
				
				//send actual compressed image
				sendBytes(comm_socket, (char*)txBuffer2, sendSize2);
			}
			fprintf(stderr, "."); fflush(stderr);
			i++;
		}
		fprintf(stderr, "-\n"); fflush(stderr);
		free(txBuffer);
		//////////////////
	}
	if(width == 1216 && height == 920){
		free(convertBuff);
	}
	free(multiBuffer);
	if(highPrecisionMode==2){
		free(multiBufferSQ);
	}
	if(grr && isSPIMaster){
		flushBuffer(spidevName, numberOfBuffer+1);
		enableSync(0, spidevName);
	}
	uninit_device(c1);
	return;
	
}


int main(){
	
	//handle signal generated when server disconnect
	signal(SIGPIPE, sigPipeHandler);
	
	
	
	
	//to do, read id and camera device name from file
	int32_t id = -1;
	char serverIP[30];
	char devName[30];
	char spidevName[30];
	spidev_g = spidevName;
	int32_t isSPIMaster=0;
	FILE* f1 = fopen("camConfig.txt", "r");
	if(f1==NULL){
		printError("can't open config file");
	}
	fscanf(f1, "%s", serverIP);
	fscanf(f1, "%d", (int*)&id);
	fscanf(f1, "%s", devName);
	fscanf(f1, "%d", (int*)&isSPIMaster);
	fscanf(f1, "%s", spidevName);
	fclose(f1);
	
	
	
	int comm_port = COMM_PORT;
	int comm_socket = -1;
	
	comm_socket = sockopen(serverIP, comm_port);
	if(comm_socket==-1)
		exit(0);
		
	fprintf(stderr, "connected\n"); fflush(stderr);
	
	cameraState* c1 = init_camera(devName, 4896, 3684, 1, 2, BYTESPERPIXEL);
	if(v4l2ResetControl(c1, V4L2_CID_CONTRAST)!=0){
		fprintf(stderr, "camera settings error, exiting\n"); fflush(stderr);
		if(c1!=NULL){
			uninit_device(c1);
		}
		exit(0);
	}
	if(c1==NULL){
		exit(0);
	}
	struct v4l2_buffer buff1;
	getBuffer(c1, &buff1);
	pushBuffer(c1, &buff1);
	uninit_device(c1);
	
	
	
	//////////////////////////////////////////////////
	//send id
	fprintf(stderr, "camID:%d\n", id);
	fflush(stderr);
	int32_t sendSize = sizeof(id);
	sendHeader(comm_socket, HEADER_ID);
	sendBytes(comm_socket, (char*)&id, sendSize);
	
	
	//recieve capture info
	if(HEADER_CAPINFO != recvHeader(comm_socket)){
		fprintf(stderr, "Does not recieve capture info, exiting\n"); fflush(stderr);
		exit(0);
	}
	captureInfo cInfo;
	sendSize = (int) sizeof(cInfo);
	recvBytes(comm_socket, (char*)&cInfo, sendSize);
	
	int32_t usenewSettings = cInfo.usenewSettings;
	int32_t overwrite = cInfo.overwrite;
	
	//update every capture
	int32_t mode = cInfo.mode;
	int32_t number = cInfo.number;
	int32_t grr = cInfo.grr;
	int32_t resolution = cInfo.resolution;
	int32_t gain = cInfo.gain;
	int32_t multiexposure = cInfo.multiexposure;
	int32_t triggerDelay = cInfo.triggerDelay;
	int32_t highPrecisionMode = cInfo.highPrecisionMode;
	/////////////////
	fprintf(stderr, "mode:%d\n", mode);
	fprintf(stderr, "number:%d\n", number);
	fprintf(stderr, "resolution:%d\n", resolution);
	fflush(stderr);
	int width;
	int height;
	
	switch(resolution){
		case 0: width = 4896; height = 3684; break;
		case 1: width = 2432; height = 1842; break;
		case 2: width = 1216; height = 920; break;
		default : width = 4896 ; height = 3684;
	}
	switch(mode){
		case 0: preview(comm_socket, width, height, &cInfo, devName, isSPIMaster, spidevName); break;
		case 1: 
		case 2: 
		case 3: streamFrame(comm_socket, width, height, &cInfo, devName, isSPIMaster, spidevName); break;
		default: cInfo.mode = 0; preview(comm_socket, width, height, &cInfo, devName, isSPIMaster, spidevName); break;
	}
	
	
	fprintf(stderr, "starting over\n"); fflush(stderr);
	
	sockclose(comm_socket);
	
	
	return 0;
}
