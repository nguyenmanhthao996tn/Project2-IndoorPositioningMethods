/*************************************************************
 * @author ThaoNguyen                                        *
 * @modified 2018/04/17 14:21                                *
 * @desciption This program write every package data         *
 *             received from serial port (ttyUSB0) and time  *
 *             that package received to text file.           *
 *************************************************************/

/********************* LIBRARIES *********************/
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include "PosDataType.h"

/********************* CONFIGURATIONS *********************/
#define BAUDRATE B115200
#define MODEMDEVICE "/dev/ttyUSB0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define MAX_DATA_RECEIVE_PACKAGE_COUNTER 25
#define MAX_BYTE_EACH_PACKAGE 5

/********************* GLOBAL VARIABLES *********************/
volatile int fd;
volatile struct timespec timeReceived;
volatile struct termios oldTSetting;
volatile char buf[255];

volatile int receivedDataIndex;
volatile PosDataType_p receivedDataArray[MAX_DATA_RECEIVE_PACKAGE_COUNTER];

/********************* METHODS *********************/
void setupSerialPort(struct termios *oldTSetting);
void setupReceivedDataVariables(void);
void writeToTextFile(void);
void signalIO_handler(int status); /* definition of signal handler */

/********************* MAIN *********************/
int main(int argc, char *argv[])
{
    setupReceivedDataVariables();
    setupSerialPort(&oldTSetting);

    printf("Setting up complete!\n");

    while (true)
    {
        // Background worker save data to text file
        if (receivedDataIndex >= MAX_DATA_RECEIVE_PACKAGE_COUNTER)
        {
            receivedDataIndex = 0;
            writeToTextFile();
        }
    }

    return EXIT_SUCCESS;
}

void setupReceivedDataVariables(void)
{
    int i = 0;
    for (; i < MAX_DATA_RECEIVE_PACKAGE_COUNTER; i++)
    {
        receivedDataArray[i] = (PosDataType_p)malloc(sizeof(PosDataType));
        receivedDataArray[i]->data = (char *)malloc(sizeof(char) * MAX_BYTE_EACH_PACKAGE);
    }

    receivedDataIndex = 0;
}

void setupSerialPort(struct termios *oldTSetting)
{
    struct termios newTSetting;
    struct sigaction saio; /* definition of signal action */

    /* open the device to be non-blocking (read will return immediatly) */
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        perror(MODEMDEVICE);
        exit(EXIT_FAILURE);
    }

    /* install the signal handler before making the device asynchronous */
    saio.sa_handler = signalIO_handler;
    sigemptyset(&saio.sa_mask); // saio.sa_mask = 0;
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGIO, &saio, NULL);

    /* allow the process to receive SIGIO */
    fcntl(fd, F_SETOWN, getpid());

    /* Make the file descriptor asynchronous (the manual page says only 
           O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
    fcntl(fd, F_SETFL, FASYNC);

    /* save current port settings */
    tcgetattr(fd, oldTSetting);

    /* set new port settings for canonical input processing */
    newTSetting.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    newTSetting.c_iflag = IGNPAR | ICRNL;
    newTSetting.c_oflag = 0;
    newTSetting.c_lflag = ICANON;
    newTSetting.c_cc[VMIN] = 1;
    newTSetting.c_cc[VTIME] = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newTSetting);
}

void writeToTextFile(void)
{
    int i = 0;
    char strFileName[30], textBuffer[100];
    char *strTime;
    FILE *fp;

    // Get name for text file
    snprintf(strFileName, 40, "%.15ld_%.15ld.txt\0", receivedDataArray[0]->timeReceived.tv_sec, receivedDataArray[0]->timeReceived.tv_nsec);

    // Open
    fp = fopen(strFileName, "w+");

    // Write
    for (; i < MAX_DATA_RECEIVE_PACKAGE_COUNTER; i++)
    {
        strTime = ctime(&(receivedDataArray[i]->timeReceived.tv_sec));
        snprintf(textBuffer, 100, "Time: %s%15.9ldns | Data: %s\n", strTime, receivedDataArray[i]->timeReceived.tv_nsec, receivedDataArray[i]->data);
        fprintf(fp, "%s", textBuffer);
    }

    // Close
    fclose(fp);

    // Notify to console
    printf("Received packages and wrote to file %.30s\n", strFileName);
}

void signalIO_handler(int status)
{
    int res;
    // Get time
    if (clock_gettime(CLOCK_REALTIME, &timeReceived) == -1)
    {
        printf("Get time ERROR!");
        perror(CLOCK_REALTIME);
    }

    // Get serial data
    res = read(fd, buf, 255);
    buf[res] = 0;

    // Add to workload
    strncpy(receivedDataArray[receivedDataIndex]->data, buf, MAX_BYTE_EACH_PACKAGE);
    receivedDataArray[receivedDataIndex]->timeReceived.tv_sec = timeReceived.tv_sec;
    receivedDataArray[receivedDataIndex]->timeReceived.tv_nsec = timeReceived.tv_nsec;

    // Print to console
    printf("received SIGIO signal No.%d: %s\n", ++receivedDataIndex, buf);
}
