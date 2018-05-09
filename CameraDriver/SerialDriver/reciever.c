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

    char buf[1024];
    FILE *outFile;
    DWORD nread;
    outFile = fopen("tmpOUTRX.jpg", "wb");
    if (outFile) {
        nread = serialRead(hComm, buf, sizeof(buf));
        while ( nread > 0) {
            // fwrite(buf, 1, nread, stdout);   
            fwrite(buf, 1, nread, outFile);
            printf("nread: %i", nread);
            nread = serialRead(hComm, buf, sizeof(buf));
        }
        if (ferror(outFile)) {
            fprintf(stderr, "error in outfile");
        }
        fclose(outFile);
    }

    // DWORD nBytesRead;
    // DWORD nBytesWritten;
    // char buf[100];
    // while (TRUE) {
    //     nBytesRead = serialRead(hComm, buf, sizeof(buf));
    //     buf[nBytesRead] = '\0';
    //     printf(buf);
    //     while (kbhit()) {
    //         char c = getch();
    //         nBytesWritten = serialWrite(hComm, &c, 1);
    //     }
    // }
    serialClose(&hComm);
    return 0;
}