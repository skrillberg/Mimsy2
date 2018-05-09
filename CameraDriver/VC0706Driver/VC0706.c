#include "VC0706.h"
#include <stdio.h>

#if defined(_WIN32)

#else // assumes arm

#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_ioc.h"
#include "sys_ctrl.h"
#include "uart.h"
#include "gpio.h"
#include "ioc.h"

#endif

void common_init(VC0706_t *vc0706) {
#if defined(_WIN32)
    vc0706->hComm = NULL;
#else // assumes arm
    vc0706->uartBase = NULL;
#endif
    vc0706->frameptr = 0;
    vc0706->bufferLen = 0;
}

void sendCommand(VC0706_t *vc0706, uint8_t cmd, uint8_t args[], uint8_t argn) {
#if defined(_WIN32)
    serialWriteByte(vc0706->hComm, 0x56);
    serialWriteByte(vc0706->hComm, 0); //serial num
    serialWriteByte(vc0706->hComm, cmd);
    
    serialWrite(vc0706->hComm, args, argn);
#else // assumes arm
    UARTCharPut(vc0706->uartBase, 0x56);
    UARTCharPut(vc0706->uartBase, 0);
    UARTCharPut(vc0706->uartBase, cmd);
    
    uint8_t i;
    for (i = 0; i < argn; ++i) {
       UARTCharPut(vc0706->uartBase, args[i]);
    }
#endif
}

uint8_t readResponse(VC0706_t *vc0706, uint8_t numbytes, uint8_t timeout) {
    uint8_t counter = 0;
    vc0706->bufferLen = 0;
#if defined(_WIN32)
    while ((timeout != counter) && (vc0706->bufferLen != numbytes)) {      
        uint32_t numRead = serialRead(vc0706->hComm, 
            &vc0706->camerabuff[vc0706->bufferLen], numbytes - vc0706->bufferLen);    
        if (numRead <= 0) {
            ++counter;
            continue;
        }
        counter = 0;
        vc0706->bufferLen += numRead;
    }
#else // assumes arm
    uint16_t avail;
    while ((timeout != counter) && (vc0706->bufferLen != numbytes)) {
        avail = UARTCharsAvail(vc0706->uartBase);
        if (avail <= 0) {
            ++counter;
            continue;
        }
        counter = 0;
        vc0706->camerabuff[vc0706->bufferLen++] = UARTCharGet(vc0706->uartBase);
    }
#endif   
    return vc0706->bufferLen;
}

int8_t verifyResponse(VC0706_t *vc0706, uint8_t command) {
    if ((vc0706->camerabuff[0] != 0x76) ||
        (vc0706->camerabuff[1] != 0) || // serial num
        (vc0706->camerabuff[2] != command) ||
        (vc0706->camerabuff[3] != 0x0))
        return FALSE;
    return TRUE;
}

int8_t runCommand(VC0706_t *vc0706, uint8_t cmd, uint8_t *args, uint8_t argn,
    uint8_t resplen, int8_t flushflag) {
    // flush out anything in the buffer?
    if (flushflag) {
        readResponse(vc0706, 100, 10); 
    }

    sendCommand(vc0706, cmd, args, argn);
    if (readResponse(vc0706, resplen, 200) != resplen) {
#if defined(DEBUG)      
        fprintf(stderr, "[vc0706] did not get response to command\r\n");
#endif        
        return FALSE;
    }
    if (!verifyResponse(vc0706, cmd)) {
#if defined(DEBUG)      
        fprintf(stderr, "[vc0706] response to command was invalid\r\n");
#endif        
        return FALSE;
    }
    return TRUE;
}

#if defined(DEBUG)
void printBuff() {
    fprintf(stderr, "[vc0706] printBuff called, but not implemented\r\n");
}
#endif

void VC0706Close(VC0706_t *vc0706) {
#if defined(_WIN32)  
    serialClose(&vc0706->hComm);
#endif  
}

#if defined(_WIN32)
int8_t VC0706Begin(VC0706_t *vc0706, const char *comPort, uint32_t baudRate) {
#else // assumes arm
int8_t VC0706Begin(VC0706_t *vc0706, uint32_t uartBase) {
#endif
    common_init(vc0706);
    int8_t status;
#if defined(_WIN32)    
    status = serialInit(comPort, baudRate, &vc0706->hComm);
#else // assumes arm
    vc0706->uartBase = uartBase;
    status = TRUE;
#endif    
    if (!status) {
#if defined(DEBUG)      
        fprintf(stderr, "[vc0706] failed to begin\r\n");
#endif        
        return FALSE;
    }
    return VC0706Reset(vc0706);
}

int8_t VC0706Reset(VC0706_t *vc0706) {
    uint8_t args[] = {0x0};
    return runCommand(vc0706, VC0706_RESET, args, 1, 5, TRUE);
}

int8_t VC0706TakePicture(VC0706_t *vc0706) {
    vc0706->frameptr = 0;
    return VC0706CameraFrameBuffCtrl(vc0706, VC0706_STOPCURRENTFRAME);
}

uint8_t *VC0706readPicture(VC0706_t *vc0706, uint8_t n) {
    uint8_t args[] = {0x0C, 0x0, 0x0A, 
                        0, 0, vc0706->frameptr >> 8, vc0706->frameptr & 0xFF, 
                        0, 0, 0, n, 
                        CAMERADELAY >> 8, CAMERADELAY & 0xFF};

    if (!runCommand(vc0706, VC0706_READ_FBUF, args, sizeof(args), 5, FALSE))
        return 0;

    // read into the buffer PACKETLEN!
    if (readResponse(vc0706, n+5, CAMERADELAY) == 0) 
        return 0;

    vc0706->frameptr += n;

    return vc0706->camerabuff;
}

int8_t VC0706ResumeVideo(VC0706_t *vc0706) {
    return VC0706CameraFrameBuffCtrl(vc0706, VC0706_RESUMEFRAME);
}

uint32_t VC0706FrameLength(VC0706_t *vc0706) {
    uint8_t args[] = {0x01, 0x00};
    if (!runCommand(vc0706, VC0706_GET_FBUF_LEN, args, sizeof(args), 9, TRUE))
        return 0;

    uint32_t len;
    len = vc0706->camerabuff[5];
    len <<= 8;
    len |= vc0706->camerabuff[6];
    len <<= 8;
    len |= vc0706->camerabuff[7];
    len <<= 8;
    len |= vc0706->camerabuff[8];

    return len;
}

char *VC0706GetVersion(VC0706_t *vc0706) {
    uint8_t args[] = {0x01};

    sendCommand(vc0706, VC0706_GEN_VERSION, args, 1);
    // get reply
    if (!readResponse(vc0706, CAMERABUFFSIZ, 50)) 
        return 0;
    vc0706->camerabuff[vc0706->bufferLen] = 0;  // end it!
    return (char *)vc0706->camerabuff;  // return it!
}

uint8_t VC0706Available(VC0706_t *vc0706) {
    return vc0706->bufferLen;
}

uint8_t VC0706GetImageSize(VC0706_t *vc0706) {
    uint8_t args[] = {0x4, 0x4, 0x1, 0x00, 0x19};
    if (!runCommand(vc0706, VC0706_READ_DATA, args, sizeof(args), 6, TRUE))
        return -1;

    return vc0706->camerabuff[5];
}

int8_t VC0706SetImageSize(VC0706_t *vc0706, uint8_t x) {
    uint8_t args[] = {0x05, 0x04, 0x01, 0x00, 0x19, x};
    return runCommand(vc0706, VC0706_WRITE_DATA, args, sizeof(args), 5, TRUE);
}

int8_t VC0706CameraFrameBuffCtrl(VC0706_t *vc0706, uint8_t command) {
    uint8_t args[] = {0x1, command};
    return runCommand(vc0706, VC0706_FBUF_CTRL, args, sizeof(args), 5, TRUE);
}

uint8_t VC0706GetCompression(VC0706_t *vc0706) {
    uint8_t args[] = {0x4, 0x1, 0x1, 0x12, 0x04};
    runCommand(vc0706, VC0706_READ_DATA, args, sizeof(args), 6, TRUE);
#if defined(DEBUG)    
    printBuff(vc0706);
#endif    
    return vc0706->camerabuff[5];
}

int8_t VC0706SetCompression(VC0706_t *vc0706, uint8_t c) {
    uint8_t args[] = {0x5, 0x1, 0x1, 0x12, 0x04, c};
    return runCommand(vc0706, VC0706_WRITE_DATA, args, sizeof(args), 5, TRUE);
}
