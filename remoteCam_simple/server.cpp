#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "camera/camera.h"
#include "socklib/socklib.h"
#include "socketHelper.h"


extern "C" {
#include "liblz4/lz4.h"
}

#include <thread>
#include <mutex>
#include <stdint.h>
#include "remoteCam.h"
#include "shader.h"
// GLEW
#define GLEW_STATIC
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

#ifndef WIN32
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#else
#define KNRM  ""
#define KRED  ""
#define KGRN  ""
#define KYEL  ""
#define KBLU  ""
#define KMAG  ""
#define KCYN  ""
#define KWHT  ""

#endif



//evil globals!
int32_t newSetting=0;
int32_t settingNum = SETTING_NONE;
int32_t started =0;
int32_t previewCam=-1;


typedef struct s_recvCallArgs{
	int32_t mode;
	int32_t number;
	int32_t resolution;
	int32_t exposure;
	int32_t gain;
	int32_t grr;
	int32_t* camList;
	std::mutex* camListLock;
	int32_t multiexposure;
	int32_t triggerDelay;
	int32_t highPrecisionMode;
} recvCallArgs;

typedef struct s_startThreadParams{
	int mode;
} startThreadParams;

void errorAndExit(const char* errmsg){
	fprintf(stderr, "%s\n", errmsg);
	fflush(stderr);
	exit(0);
}
/*
 * //not used
int64_t calculateSharpness(uint16_t* imgBuff, int width, int height){
	int i,j;
	int64_t ret =0;
	//sampling border and bound checking
	int leftborder = width*4/10;
	leftborder = leftborder<1?1:leftborder;
	int rightborder = width*6/10;
	rightborder = rightborder>width-1?width-1:rightborder;
	int topborder = height*4/10;
	topborder = topborder<1?1:topborder;
	int bottomborder = height*6/10;;
	bottomborder = bottomborder>height-1?height-1:bottomborder;
	for(i=1;i<height;i++)
		for(j=1;j<width;j++){
			int self = imgBuff[j+i*width];
			int left = imgBuff[j-1+i*width];
			int up = imgBuff[j+(i-1)*width];
			int xdiff = self-left;
			int ydiff = self-up;
			ret += xdiff*xdiff + ydiff*ydiff;
		}
	ret = ret/(rightborder-leftborder)/(bottomborder-topborder);
	return ret;
}
*/
void loadImage(GLuint* textureRef, void* image, int width, int height){
	glGenTextures(1, textureRef);
    
    glBindTexture(GL_TEXTURE_2D, textureRef[0]); // All upcoming GL_TEXTURE_2D operations now have effect on our texture object
    // Set our texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);	// Set texture wrapping to GL_REPEAT
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // Set texture filtering
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    // Load, create texture and generate mipmaps
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, (uint16_t*)image);
    glBindTexture(GL_TEXTURE_2D, 0); // Unbind texture when done, so we won't accidentily mess up our texture.
    glFlush();
    return;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
        
    //preview settings    
        
    if(newSetting==1) return;//send old setting first, ignore new setting
    //to do, add camera exit setting
    
	if (key == GLFW_KEY_INSERT && action == GLFW_PRESS){
		settingNum = EXPOSURE_INC; newSetting = 1;
	}
	if (key == GLFW_KEY_DELETE && action == GLFW_PRESS){
		settingNum = EXPOSURE_DEC; newSetting = 1;
	}
	if (key == GLFW_KEY_HOME && action == GLFW_PRESS){
        settingNum = GAIN_INC; newSetting = 1;
    }
    if (key == GLFW_KEY_END && action == GLFW_PRESS){
        settingNum = GAIN_DEC; newSetting = 1;
    }
    if (key == GLFW_KEY_MINUS && action == GLFW_PRESS){
        settingNum = ZOOM_OUT; newSetting = 1;
    }
    if (key == GLFW_KEY_EQUAL && action == GLFW_PRESS){
        settingNum = ZOOM_IN; newSetting = 1;
    }
    if (key == GLFW_KEY_0 && action == GLFW_PRESS){
        settingNum = ZOOM_DEF; newSetting = 1;
    }
}

void start_function(startThreadParams* p1){
	if(p1->mode == 0){
	fprintf(stderr, "enter preview camera id:");
	fflush(stderr);
	fscanf(stdin, "%d", &previewCam);
	}
	fprintf(stderr, "press enter to start\n"); fflush(stderr);
	char dummyBuff[10];
	fgets(dummyBuff, 10, stdin); //temporary fix for fgets, fscanf thingies...
	fgets(dummyBuff, 10, stdin);
	fprintf(stderr, "starting!!!\n"); fflush(stderr);
	started=1;
	
	
	
	return;
}

int32_t connectedCamList[MAX_CAM] = {0};
void DisplayCameraList() {
	int count = 0;
	for(int i=0;i< MAX_CAM ;i++) {
		if (connectedCamList[i] == 0)
			fprintf(stderr, KRED "%3d" KNRM, i);
        else {
			count++;			
            fprintf(stderr, KGRN "%3d" KNRM, i);
		}
	}
	printf("\nTotal Camera Connected %d\n", count);
}

void DisplayCameraList(int32_t* readyCam) {
	int count = 0;
	int finCount = 0;
	for(int i=0;i< MAX_CAM ;i++) {
		switch(readyCam[i]){
			//case 0:fprintf(stderr, KRED"%3d"KNRM, i); break;
			case 1:fprintf(stderr, KRED "%3d" KNRM, i); break;
			case 2:count++; fprintf(stderr, KGRN "%3d" KNRM, i); break;
			case 3:finCount++; fprintf(stderr, KBLU "%3d" KNRM, i); break;
		}
	}
	printf("\nTotal Camera ready %d\n", count);
}

void recvCall(int comm_socket, recvCallArgs* args1){
	int32_t mode = args1->mode;
	int32_t number = args1->number;
	int32_t resolution = args1->resolution;
	int32_t exposure = args1->exposure;
	int32_t gain = args1->gain;
	int32_t grr = args1->grr;
	int32_t* camList = args1->camList;
	std::mutex* camListLock = args1->camListLock;
	int32_t multiexposure = args1->multiexposure;
	int32_t triggerDelay = args1->triggerDelay;
	int32_t highPrecisionMode = args1->highPrecisionMode;
	//if started reject camera
	//	fprintf(stderr, "Receiving something\n");
	if(started){
		//wait a bit to prevent retry flood
		#ifdef WIN32
		Sleep(1000);	
		#else
		usleep(1000000);
		#endif
		sockclose(comm_socket);
		return;
	}
	

	
	//recieve id
	int32_t id = 0;	
	int32_t sendSize = sizeof(id);
	if(HEADER_ID!=recvHeader(comm_socket)){
		fprintf(stderr, "Does not recieve camera ID, exiting\n"); fflush(stderr);
		exit(0);
	}
	recvBytes(comm_socket, (char*)&id, sendSize);
	//wait until can start and write camList
	fprintf(stderr, KGRN "Cam ID:%d connected.\n" KNRM, id);
	connectedCamList[id] = 1;
	DisplayCameraList();
	while(!started){
		#ifdef WIN32
		Sleep(250);	
		#else
		usleep(250000);
		#endif
	}
	
	
	camListLock->lock();
	
	//critical section
	/*
	//poor implementation T_T
	int u;
	for(u=0;u<MAX_CAM;u++){
		if(camList[u]==id){
			fprintf(stderr, "duplicate camera ID, camera rejected\n"); fflush(stderr);
			return;
		}
		if(camList[u]==-1){
			camList[u] = id;
			break;
		}
	}
	
	
	if(u==MAX_CAM){
		fprintf(stderr, "Too many cameras connecting\n"); fflush(stderr);
	}
	*/
	if(camList[id]==1){
			fprintf(stderr, "duplicate camera ID, camera rejected\n"); fflush(stderr);
			return;
	}
	//fprintf(stderr, KGRN"Cam ID:%d connected.\n"KNRM, id);
	camList[id] = 1;
	//DisplayCameraList(camList);
	//////////////////
	
	camListLock->unlock();
	
	if(mode==0){
			if(id!=previewCam)
				return;
	}
	
	
	
	char fileName[20];
	sprintf(fileName, "%d.raw", id);
	FILE* f1;
	
	if(mode){
	//force binary mode on windows
	f1 = (FILE*)fopen(fileName, "wb");
	}
	
	
	
	//////////////////////////////////
	
	
	int width;
	int height;
	
	switch(resolution){
		case 0: width = 4896; height = 3684; break;
		case 1: width = 2432; height = 1842; break;
		case 2: width = 1216; height = 920; break;
		default : width = 4896 ; height = 3684;
	}
	//send capture info
	captureInfo cInfo;
	cInfo.mode = mode;
	cInfo.number = number;
	cInfo.resolution = resolution;
	cInfo.exposure = exposure;
	cInfo.gain = gain;
	cInfo.grr = grr;
	cInfo.multiexposure = multiexposure;
	cInfo.triggerDelay = triggerDelay;
	cInfo.highPrecisionMode = highPrecisionMode;
	sendSize = (int) sizeof(captureInfo);
	//send capture info header
	sendHeader(comm_socket, HEADER_CAPINFO);
	// send capture info
	sendBytes(comm_socket, (char*)&cInfo, sendSize);
	if(recvHeader(comm_socket)!=HEADER_CAMERA_READY){
		fprintf(stderr, "wrong header recieved, exiting\n");
		fflush(stderr);
		exit(0);
	}else{
		fprintf(stderr, "camera:%d ready\n", id); fflush(stderr);
		camListLock->lock();
		camList[id]=2;
		DisplayCameraList(camList);
		camListLock->unlock();
	}
	
	//mainloop
	switch(mode){
	case 0:{
		//openGL setup
		//to do, change hardcoded window size
		int window_width = 1440;
		int window_height = 1080;
		// Init GLFW
		glfwInit();
		// Set all the required options for GLFW
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

		// Create a GLFWwindow object that we can use for GLFW's functions
		GLFWwindow* window = glfwCreateWindow(window_width, window_height, "Debayer", nullptr, nullptr);
		glfwMakeContextCurrent(window);

		// Set the required callback functions
		glfwSetKeyCallback(window, key_callback);

		// Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
		glewExperimental = GL_TRUE;
		// Initialize GLEW to setup the OpenGL Function pointers
		glewInit();

		// Define the viewport dimensions
		glViewport(0, 0, window_width, window_height);


		// Build and compile our shader program
		Shader ourShader("shader.vert", "shader.frag");


		// Set up vertex data (and buffer(s)) and attribute pointers
		GLfloat vertices[] = {
			// Positions          // Colors           // Texture Coords
			 1.0f,  1.0f, 0.0f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f, // Top Right
			 1.0f, -1.0f, 0.0f,   0.0f, 1.0f, 0.0f,   1.0f, 0.0f, // Bottom Right
			-1.0f, -1.0f, 0.0f,   0.0f, 0.0f, 1.0f,   0.0f, 0.0f, // Bottom Left
			-1.0f,  1.0f, 0.0f,   1.0f, 1.0f, 0.0f,   0.0f, 1.0f  // Top Left 
		};
		GLuint indices[] = {  // Note that we start from 0!
			0, 1, 3, // First Triangle
			1, 2, 3  // Second Triangle
		};
		GLuint VBO, VAO, EBO;
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);
		glGenBuffers(1, &EBO);

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

		// Position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (GLvoid*)0);
		glEnableVertexAttribArray(0);
		// Color attribute
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
		glEnableVertexAttribArray(1);
		// TexCoord attribute
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (GLvoid*)(6 * sizeof(GLfloat)));
		glEnableVertexAttribArray(2);

		glBindVertexArray(0); // Unbind VAO
		

		// Load and create a texture 
		GLuint texture1;
		
		
		//print message for preview mode
		fprintf(stderr, "Preview mode\nPress INSERT, DELETE to increase or decrease exposure\nPress HOME, END to increase or decrease gain\nPress Esc to exit\n"); fflush(stderr);
		
		
		//init buffers for image
		sendSize = (int) width*height*sizeof(uint16_t);
		uint16_t* imgBuff = (uint16_t*)malloc(sendSize);
		memset(imgBuff, 0x00, sendSize);
		float scale = 1.0f;
		while(!glfwWindowShouldClose(window)){
			// Check if any events have been activiated (key pressed, mouse moved etc.) and call corresponding response functions
			glfwPollEvents();
			
			int32_t header;
			//send setting change
			if(newSetting){
				if(settingNum<6 || settingNum>8){
					sendHeader(comm_socket, HEADER_PREVIEW_SETTING);
					sendBytes(comm_socket, (char*)&settingNum, sizeof(settingNum));
					newSetting = 0;
					settingNum = SETTING_NONE;
				}else{
					switch(settingNum){
						case(ZOOM_IN): scale-=0.1; break;
						case(ZOOM_OUT): scale+=0.1; break;
						case(ZOOM_DEF): scale=1; break;
					}
					newSetting=0;
				}
			}	
			
			//read for setting value return
			header = recvHeader(comm_socket);
			switch(header){
				case HEADER_SETTING_RETURN:
					//recvHeader(comm_socket);
					//report settings
					char* setting_str;
					int32_t settingType;
					int32_t settingValue;
					recvBytes(comm_socket, (char*)&settingType, sizeof(settingType));
					recvBytes(comm_socket, (char*)&settingValue, sizeof(settingValue));
					switch(settingType){
						char* setting_str;
						case EXPOSURE_SETTING: fprintf(stderr, "exposure:"); break;
						case GAIN_SETTING: fprintf(stderr, "gain:"); break;
						default : fprintf(stderr, "invalid setting type reported, exiting\n"); fflush(stderr); exit(0);
					}
					fprintf(stderr, "%d\n", settingValue); fflush(stderr);
					continue;
				case HEADER_CAPSTREAM:
					break;
				default:
					fprintf(stderr, "header:%x\n", header);
					fprintf(stderr, "invalid header, exiting\n"); fflush(stderr);
					exit(0);
			}
			
			
				
			//recieve the image
			
			recvBytes(comm_socket, (char*)imgBuff, sendSize);
			
			//draw to screen
			
			
			
			
			loadImage(&texture1, imgBuff, width, height);
			// Render
			// Clear the colorbuffer
			glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			// Activate shader
			ourShader.Use();     
			glUniform1i(glGetUniformLocation(ourShader.Program, "texImageWidth"), width);
			glUniform1i(glGetUniformLocation(ourShader.Program, "texImageHeight"), height);
			glUniform1f(glGetUniformLocation(ourShader.Program, "scale"), scale);

			// Bind Textures using texture units
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture1);
			glUniform1i(glGetUniformLocation(ourShader.Program, "ourTexture1"), 0);
			
			
			// Draw container
			glBindVertexArray(VAO);
			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
			glBindVertexArray(0);
			glfwSwapInterval(1);
			// Swap the screen buffers
			glfwSwapBuffers(window);
			glDeleteTextures(1, &texture1);
			//////////////////////////
			//calculate sharpness
			//int64_t s = calculateSharpness(imgBuff, width, height);
			//fprintf(stderr, "sharpness:%ld\n", s); fflush(stderr);
			
			
		}
		// properly terminate openGL
		// Properly de-allocate all resources once they've outlived their purpose
		glDeleteVertexArrays(1, &VAO);
		glDeleteBuffers(1, &VBO);
		glDeleteBuffers(1, &EBO);
		// Terminate GLFW, clearing any resources allocated by GLFW.
		glfwTerminate();
		//////////////////
		free(imgBuff);
		
		} break;

	}
	
	camListLock->lock();
		camList[id]=3;
		DisplayCameraList(camList);
		camListLock->unlock();
	fprintf(stderr, "id:%d done\n", id); fflush(stderr);
	if(mode)fclose(f1);
	sockclose(comm_socket);	
}




int main(int argc, const char ** argv){	
	int32_t capmode=1;
	int32_t resolution=0;
	int32_t number=1;
	int32_t exposure = 10;
	int32_t gain = 1;
	int32_t grr = 0;
	int32_t multiexposure =1;
	int32_t triggerDelay=0;
	int32_t highPrecisionMode=0;
	int useconfig=0;

	char *configFile = "serverConfig.txt";
	
	if (argc > 0) {
	  useconfig = 1;
	  configFile = argv[1];
	  FILE* serverCfg = fopen(configFile, "rb");
	  if (serverCfg == NULL) {
	    fprintf(stderr, "File %s does not exist\n", configFile);
	    useconfig = 0;
	  } else
	    fclose(serverCfg);  
	} else {
	  fprintf(stderr, "use ConfigFile? 0 or 1:");
	  fscanf(stdin, "%d", &useconfig);
	}

	if(!useconfig){
	
	  fprintf(stderr, "enter mode: 0=preview 1=raw 2=compressed:");
	  fscanf(stdin, "%d", &capmode);
	  fprintf(stderr, "enter resolution: 0=full 1= 2:1 binning 2= 4:1 binning :");
	  fscanf(stdin, "%d", &resolution);
	  if(capmode!=0){
	    fprintf(stderr, "enter number of frames:");
	    fscanf(stdin, "%d", &number);
	  }else{
	    number=1;
	  }
	  fprintf(stderr, "grr(0 or 1):");
	  fscanf(stdin, "%d", &grr);
	  fprintf(stderr, "enter exposure(1-1000):");
	  fscanf(stdin, "%d", &exposure);
	  fprintf(stderr, "enter gain(0-4):");
	  fscanf(stdin, "%d", &gain);
	  fprintf(stderr, "enter number of exposures per image(1-63):");
	  fscanf(stdin, "%d", &multiexposure);
	  if(capmode!=0){
	    fprintf(stderr, "High precision mode? (0=No 1=Yes 2=Yes with x^2 sum):");
	    fscanf(stdin, "%d", &highPrecisionMode);
	  }
	
	
	  /*
	    if(grr){
	    fprintf(stderr, "triggerDelay(0-255):");
	    fscanf(stdin, "%d", &triggerDelay);
	    }
	  */
	}else{
	  FILE* serverCfg = fopen(configFile, "rb");
		if(serverCfg == NULL){
			fprintf(stderr, "cannot open serverConfig.txt, exiting\n"); fflush(stderr);
			exit(0);
		}
		char cfgArgs[20];
		while(fscanf(serverCfg, "%s", cfgArgs)!=EOF){
			if(strncmp("mode", cfgArgs, 4)==0){
				int t1;
				fscanf(serverCfg, "%d", &t1);
				if(t1<0||t1>3){
					errorAndExit("wrong mode in config file");
				}
				capmode=t1;
				continue;
			}
			if(strncmp("resolution", cfgArgs, 10)==0){
				int t1;
				fscanf(serverCfg, "%d", &t1);
				if(t1<0||t1>2){
					errorAndExit("wrong resolution in config file");
				}
				resolution=t1;
				continue;
			}
			if(strncmp("number", cfgArgs, 6)==0){
				int t1;
				fscanf(serverCfg, "%d", &t1);
				if(t1<0){
					errorAndExit("wrong number in config file");
				}
				number=t1;
				continue;
			}
			if(strncmp("exposure", cfgArgs, 8)==0){
				int t1;
				fscanf(serverCfg, "%d", &t1);
				if(t1<1||t1>1000){
					errorAndExit("wrong exposure in config file");
				}
				exposure=t1;
				continue;
			}
			if(strncmp("gain", cfgArgs, 4)==0){
				int t1;
				fscanf(serverCfg, "%d", &t1);
				if(t1<0||t1>4){
					errorAndExit("wrong gain in config file");
				}
				gain=t1;
				continue;
			}
			if(strncmp("grr", cfgArgs, 3)==0){
				int t1;
				fscanf(serverCfg, "%d", &t1);
				if(t1<0||t1>1){
					errorAndExit("wrong grr in config file");
				}
				grr=t1;
				continue;
			}
			if(strncmp("multi", cfgArgs, 4)==0){
				int t1;
				fscanf(serverCfg, "%d", &t1);
				if(t1<0){
					errorAndExit("wrong multi in config file");
				}
				multiexposure=t1;
				continue;
			}
			if(strncmp("32bit", cfgArgs, 5)==0){
				int t1;
				fscanf(serverCfg, "%d", &t1);
				if(t1<0){
					errorAndExit("wrong 32-bit setting in config file");
				}
				highPrecisionMode=t1;
				continue;
			}
		}
		fclose(serverCfg);
	}
	//print settings
	fprintf(stderr, "mode:%d\n", capmode);
	fprintf(stderr, "resolution:%d\n", resolution);
	fprintf(stderr, "number:%d\n", number);
	fprintf(stderr, "exposure:%d\n", exposure);
	fprintf(stderr, "gain:%d\n", gain);
	fprintf(stderr, "grr:%d\n", grr);
	fprintf(stderr, "multi:%d\n", multiexposure);
	fprintf(stderr, "32-bit:%d\n", highPrecisionMode);
	fflush(stderr);
	

	int32_t camList[MAX_CAM];
	int g;
	for(g=0; g<MAX_CAM; g++)
		camList[g] = -1;
	std::mutex camListLock;
	recvCallArgs args1;
	args1.mode = capmode;
	args1.number = number;
	args1.resolution = resolution;
	args1.exposure = exposure;
	args1.gain = gain;
	args1.grr = grr;
	args1.camList = camList;
	args1.camListLock = &camListLock;
	args1.multiexposure = multiexposure;
	args1.triggerDelay = triggerDelay;
	args1.highPrecisionMode = highPrecisionMode;
	//create thread to recieve start command
	
	
	startThreadParams startParams;
	startParams.mode = capmode;
	std::thread start_thread(start_function, &startParams);
	
	int comm_port = COMM_PORT;
	int comm_socket = sockopen(NULL, comm_port);
	
	PF_SOCKETHANDLER h1 = (PF_SOCKETHANDLER)(comm_socket, (void*)recvCall);
	socklisten(comm_socket, &args1, h1);
	
	
	
	
	return 0;
}
