#ifndef _SERIAL_DRIVER_H
#define _SERIAL_DRIVER_H

#include <windows.h>
#include <stdint.h>

// sets up hComm, and returns whether set up was successful
BOOL serialInit(const char comPort[], DWORD baudRate, HANDLE *hComm);

BOOL serialClose(HANDLE *hComm);

// returns number of bytes written
DWORD serialWrite(HANDLE hComm, const char *buff, int buffLen);

DWORD serialWriteByte(HANDLE hComm, const char byte);

// returns number of bytes read
DWORD serialRead(HANDLE hComm, char *buff, int buffLen);

// 
void serialFlush(HANDLE hComm);

#endif