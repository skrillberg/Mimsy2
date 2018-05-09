// A port of Adafruit's VC0706 library in C and for windows (currently)

#ifndef _VC0706_DRIVER_H_
#define _VC0706_DRIVER_H_

#if defined (_WIN32)
#include "../SerialDriver/serial.h"
#else // assumes arm

#define TRUE 1
#define FALSE 0

#endif

#include <stdint.h>

#define VC0706_RESET  0x26
#define VC0706_GEN_VERSION 0x11
#define VC0706_SET_PORT 0x24
#define VC0706_READ_FBUF 0x32
#define VC0706_GET_FBUF_LEN 0x34
#define VC0706_FBUF_CTRL 0x36
#define VC0706_DOWNSIZE_CTRL 0x54
#define VC0706_DOWNSIZE_STATUS 0x55
#define VC0706_READ_DATA 0x30
#define VC0706_WRITE_DATA 0x31
#define VC0706_COMM_MOTION_CTRL 0x37
#define VC0706_COMM_MOTION_STATUS 0x38
#define VC0706_COMM_MOTION_DETECTED 0x39
#define VC0706_MOTION_CTRL 0x42
#define VC0706_MOTION_STATUS 0x43
#define VC0706_TVOUT_CTRL 0x44
#define VC0706_OSD_ADD_CHAR 0x45

#define VC0706_STOPCURRENTFRAME 0x0
#define VC0706_STOPNEXTFRAME 0x1
#define VC0706_RESUMEFRAME 0x3
#define VC0706_STEPFRAME 0x2

#define VC0706_640x480 0x00
#define VC0706_320x240 0x11
#define VC0706_160x120 0x22

#define VC0706_MOTIONCONTROL 0x0
#define VC0706_UARTMOTION 0x01
#define VC0706_ACTIVATEMOTION 0x01

#define VC0706_SET_ZOOM 0x52
#define VC0706_GET_ZOOM 0x53

#define CAMERABUFFSIZ 100
#define CAMERADELAY 10

typedef struct {
#if defined (_WIN32)
    HANDLE hComm;
#else // assumes arm
    uint32_t uartBase;
#endif
    uint8_t camerabuff[CAMERABUFFSIZ + 1];
    uint8_t bufferLen;
    uint16_t frameptr;
} VC0706_t;

void VC0706Close(VC0706_t *vc0706);

#if defined(_WIN32)
int8_t VC0706Begin(VC0706_t *vc0706, const char *comPort, uint32_t baudRate);
#else // assumes arm
int8_t VC0706Begin(VC0706_t *vc0706, uint32_t uartBase);
#endif

int8_t VC0706Reset(VC0706_t *vc0706);

int8_t VC0706TakePicture(VC0706_t *vc0706);

uint8_t *VC0706readPicture(VC0706_t *vc0706, uint8_t n);

int8_t VC0706ResumeVideo(VC0706_t *vc0706);

uint32_t VC0706FrameLength(VC0706_t *vc0706);

char *VC0706GetVersion(VC0706_t *vc0706);

uint8_t VC0706Available(VC0706_t *vc0706);

uint8_t VC0706GetImageSize(VC0706_t *vc0706);

int8_t VC0706SetImageSize(VC0706_t *vc0706, uint8_t);

int8_t VC0706CameraFrameBuffCtrl(VC0706_t *vc0706, uint8_t command);

uint8_t VC0706GetCompression(VC0706_t *vc0706);

int8_t VC0706SetCompression(VC0706_t *vc0706, uint8_t c);

#endif