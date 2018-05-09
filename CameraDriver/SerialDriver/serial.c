#include "serial.h"

#include <stdio.h>

BOOL setSerialControlSettings(HANDLE hComm, DWORD baudRate) {
    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);

    BOOL status;

    status = GetCommState(hComm, &dcb);
    if (!status) {
        fprintf(stderr, "[serial] failed to get serial params\r\n");
        CloseHandle(hComm);
        return FALSE;
    }

    dcb.BaudRate = baudRate;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    status = SetCommState(hComm, &dcb);
    if (!status) {
        fprintf(stderr, "[serial] failed to set serial params\r\n");
        CloseHandle(hComm);
        return FALSE;
    }

    return TRUE;
}

BOOL setTimeoutSettings(HANDLE hComm) {
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    BOOL status;
    status = SetCommTimeouts(hComm, &timeouts);
    if (!status) {
        fprintf(stderr, "[serial] failed to set timeouts on com port\r\n");
        CloseHandle(hComm);
        return FALSE;
    }

    return TRUE;
}

BOOL serialInit(const char comPort[], DWORD baudRate, HANDLE *hComm) {
    *hComm = CreateFile(
        comPort,
        GENERIC_READ | GENERIC_WRITE,
        0,
        0,
        OPEN_EXISTING,
        0,
        0);

    if (*hComm == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "[serial] error in opening com port\r\n");
        CloseHandle(hComm);
        return FALSE;
    }

    BOOL status;

    status = setSerialControlSettings(*hComm, baudRate);
    if (!status) {
        return FALSE;
    }    

    status = setTimeoutSettings(*hComm);
    if (!status) {
        return FALSE;
    }

    return TRUE;
}

BOOL serialClose(HANDLE *hComm) {
    return CloseHandle(*hComm);
}

DWORD serialWrite(HANDLE hComm, const char *buff, int buffLen) {
    DWORD nBytesWritten;
    WriteFile(hComm, buff, buffLen, &nBytesWritten, NULL);
    return nBytesWritten;
}

DWORD serialWriteByte(HANDLE hComm, const char byte) {
    return serialWrite(hComm, &byte, 1);
}

DWORD serialRead(HANDLE hComm, char *buff, int buffLen) {
    BOOL status;
    DWORD nBytesRead;
    status = ReadFile(hComm, buff, buffLen, &nBytesRead, NULL);
    if (!status) {
        return 0;
    } else {
        return nBytesRead;
    }
}

void serialFlush(HANDLE hComm) {
    char c;
    DWORD nBytes;
    do {
        nBytes = serialRead(hComm, &c, 1);
    } while (nBytes != 0);
}