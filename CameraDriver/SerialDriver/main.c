#include <windows.h>
#include <stdio.h>
#include "serial.h"
#include <conio.h>

int main() {
    HANDLE hComm;
    char comPort[] = "\\\\.\\COM13";
    DWORD baudRate = 38400;
    BOOL status;
    status = serialInit(comPort, baudRate, &hComm);
    if (!status) {
        fprintf(stderr, "failed to create handle\r\n");
        return 1;
    }

    serialFlush(hComm);

    char send[] = "hello\r\n";
    serialWrite(hComm, send, sizeof(send));

    DWORD nBytesRead;
    DWORD nBytesWritten;
    char buf[100];
    while (TRUE) {
        nBytesRead = serialRead(hComm, buf, sizeof(buf));
        buf[nBytesRead] = '\0';
        printf(buf);
        while (kbhit()) {
            char c = getch();
            nBytesWritten = serialWrite(hComm, &c, 1);
        }
    }
    serialClose(&hComm);
    return 0;
}

//     status = SetCommMask(hComm, EV_RXCHAR);
//     if (!status) {
//         printf("failed to set to monitor recieve character event\r\n");
//         CloseHandle(hComm);
//         return 1;
//     }

//     DWORD dwEventMask;
//     status = WaitCommEvent(hComm, &dwEventMask, NULL);
//     if (!status) {
//         printf("failed to get event\r\n");
//         serialClose(hComm);
//         return 1;
//     }

//     char c;
//     char serialBuff[256];
//     DWORD noBytesRead;
//     int i = 0;
//     do {
//         status = ReadFile(hComm, &c, 1, &noBytesRead, NULL);
//         if (!status) {
//             printf("failed to read character\r\n");
//             serialClose(hComm);
//             return 1;
//         }

//         serialBuff[i] = c;
//         ++i;
//     } while (noBytesRead > 0);
//     serialBuff[i-1] = '\0';
//     printf(serialBuff);

//     CloseHandle(hComm);
// }