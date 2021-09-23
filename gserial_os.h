#ifndef __GSERIAL_OS_H__
#define __GSERIAL_OS_H__



#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifdef _WIN32

#include <Windows.h>

typedef struct gserialOsHandleType{
	HANDLE fd;
} gserialOsHandleType;

#else

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

typedef struct gserialOsHandleType {
	int fd;
} gserialOsHandleType;

#endif

void gserial_os_set_interface_attr(gserialOsHandleType* fd, int baud, int bytesize, int parity);
gserialOsHandleType* gserial_os_open(char* path);
void gserial_os_close(gserialOsHandleType* fd);
int gserial_os_is_open(gserialOsHandleType* fd);
int gserial_os_available(gserialOsHandleType* fd);
int gserial_os_read(gserialOsHandleType* fd, char* bytes, int len);
int gserial_os_write(gserialOsHandleType* fd, char* bytes, int len);

#endif


