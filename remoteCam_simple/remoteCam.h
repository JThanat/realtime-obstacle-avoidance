#include <stdlib.h>


typedef struct s_captureInfo{
	int32_t mode;
	int32_t number;
	int32_t resolution;
	int32_t exposure;
	int32_t gain;
	int32_t grr;
	//maintained for compatability, to be removed
	int32_t usenewSettings;
	int32_t overwrite;
	/////////////////////////////////////////////
	int32_t multiexposure;
	int32_t triggerDelay;
	int32_t highPrecisionMode;
} captureInfo;

//for broadcast
const int32_t STARTCODE = 0x00000010;
const int32_t EXITCODE = 0x000000FF;

//header constants

const int32_t HEADER_INVALID = -1;
const int32_t HEADER_CAPSTREAM = 1;
const int32_t HEADER_CAPINFO = 2;
const int32_t HEADER_PREVIEW_SETTING = 3;
const int32_t HEADER_ID = 4;
const int32_t HEADER_SETTING_RETURN = 5;
const int32_t HEADER_CAPSTREAM_COMPRESSED = 6;
const int32_t HEADER_CAMERA_READY = 7;
const int32_t HEADER_CAPSTREAM_32 = 8;
const int32_t HEADER_CAPSTREAM_COMPRESSED_32 = 9;


//settings constants
const int32_t EXPOSURE_INC = 1;
const int32_t EXPOSURE_DEC = 2;
const int32_t GAIN_INC = 3;
const int32_t GAIN_DEC = 4;
//maintained for compatability, to be removed
const int32_t SAVE_SETTING = 5;
/////////////////////////////////////////////
const int32_t ZOOM_IN =6;
const int32_t ZOOM_OUT =7;
const int32_t ZOOM_DEF =8;

const int32_t EXPOSURE_SETTING = 9;
const int32_t GAIN_SETTING = 10;
const int32_t EXIT_SETTING = 11;
const int32_t SETTING_NONE = 0;

//port numbers

const int COMM_PORT = 21000;
const int BROADCAST_PORT = 21002;
const int ALT_PORT = 42000;


//evil 
const int MAX_CAM = 50;
const int PAYLOAD_LENGTH = 1024;
const int HEADERSIZE=12;
