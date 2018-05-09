#include "VC0706.h"
#include <stdio.h>

int main() {
    char comPort[] = "\\\\.\\COM15";
    DWORD baudRate = 38400;

    BOOL status;

    VC0706_t *vc0706 = VC0706New();

    status = VC0706Begin(vc0706, comPort, baudRate);
    if (!status) {
        fprintf(stderr, "failed to begin vc0706\r\n");
        return 1;
    }
    char *reply = VC0706GetVersion(vc0706);
    if (reply == 0) {
        fprintf(stderr, "failed to get version\r\n");
        return 1;
    }
    printf("version: %s", reply);

    printf("setting image size\r\n");
    status = VC0706SetImageSize(vc0706, VC0706_160x120);
    if (!status) {
        fprintf(stderr, "failed to set image size\r\n");
        return 1;
    }

    printf("getting image size\r\n");
    uint8_t imgsize = VC0706GetImageSize(vc0706);
    if (imgsize == VC0706_640x480) {
        printf("640x480\r\n");
    } else if (imgsize == VC0706_320x240) {
        printf("320x240\r\n");
    } else if (imgsize == VC0706_160x120) {
        printf("160x120\r\n");
    } else {
        fprintf(stderr, "failed to get image size\r\n");
        return 1;
    }

    printf("reading picture\r\n");
    status = VC0706TakePicture(vc0706);
    if (!status) {
        fprintf(stderr, "failed to take picture\r\n");
        return 1;
    }

    FILE *f;
    f = fopen("out.jpg", "wb");
    uint8_t wCount = 0;
    if (f) {
        uint16_t jpglen = VC0706FrameLength(vc0706);
        printf("frame length: %i\r\n", jpglen);
        while (jpglen > 0) {
            uint8_t *buffer;
            uint8_t bytesToRead = min(64, jpglen);
            buffer = VC0706readPicture(vc0706, bytesToRead);
            fwrite(buffer, 1, sizeof buffer, f);
            jpglen -= sizeof buffer;
            printf("jpglen: %i\r\n", jpglen);
            ++wCount;
            if (wCount >= 64) {
                printf(".");
                wCount = 0;
            }
        }
        fclose(f);
    }

    printf("finished reading picture\r\n");
    VC0706Close(vc0706);
}