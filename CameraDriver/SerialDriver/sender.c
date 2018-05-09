#include <windows.h>
#include <stdio.h>
#include "serial.h"
#include <conio.h>

int main() {
    HANDLE hComm;
    char comPort[] = "\\\\.\\COM15";
    DWORD baudRate = 38400;
    BOOL status;
    status = serialInit(comPort, baudRate, &hComm);
    if (!status) {
        fprintf(stderr, "failed to create handle\r\n");
        return 1;
    }

    serialFlush(hComm);

    char buf[1024];
    FILE *inFile;
    FILE *outf;
    size_t nread;
    inFile = fopen("tmp.jpg", "rb");
    outf = fopen("tmpOUT.jpg", "wb");
    if (inFile && outf) {
        nread = fread(buf, 1, sizeof buf, inFile);
        printf("nread: %i", nread);                           
        while ( nread > 0) {
            // fwrite(buf, 1, nread, stdout);
            serialWrite(hComm, buf, nread); 
            fwrite(buf, 1, nread, outf);
            nread = fread(buf, 1, sizeof buf, inFile);
            printf("nread: %i", nread);                   
        }
        if (ferror(inFile)) {
            fprintf(stderr, "error in inFile");
        }
        if (ferror(outf)) {
            fprintf(stderr, "error in outf");
        }
        fclose(inFile);
        fclose(outf);
    }
    serialFlush(hComm);
    serialClose(&hComm);
    return 0;
}