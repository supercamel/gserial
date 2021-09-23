#include "gserial_os.h"

#ifndef _WIN32
static int baud_to_b_value(int baud) {
	//B0,  B50,  B75,  B110,  B134,  B150,  B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
	switch(baud) {
		case 50:
			return B50;
		case 75:
			return B75;
		case 110:
			return B110;
		case 134:
			return B134;
		case 150:
			return B150;
		case 200:
			return B200;
		case 300:
			return B300;
		case 600:
			return B600;
		case 1200:
			return B1200;
		case 1800:
			return B1800;
		case 2400:
			return B2400;
		case 4800:
			return B4800;
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		case 57600:
			return B57600;
		case 115200:
			return B115200;
		case 230400:
			return B230400;
		case 460800:
			return B460800;
		}
	return -1;
}
#endif

void gserial_os_set_interface_attr(gserialOsHandleType* fd, int baud, int bytesize, int parity)
{
#ifdef _WIN32
	DCB dcb = {0};
	dcb.DCBlength = sizeof(DCB);

	if (!GetCommState (fd->fd, &dcb))
	{
		g_warn_if_reached();
		return;
	}

	dcb.BaudRate  = baud;
	dcb.ByteSize = bytesize;

	if(parity == 0)
		dcb.Parity = NOPARITY;
	else if(parity == 1)
		dcb.Parity = ODDPARITY;
	else if(parity == 2)
		dcb.Parity = EVENPARITY;

	dcb.StopBits  = ONESTOPBIT;
	if (!SetCommState(fd->fd, &dcb))
	{
		g_warn_if_reached();
	}
#else

	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd->fd, &tty) != 0)
	{
		//error_message ("error %d from tcgetattr", errno);
		return;
	}

	int b_baud = baud_to_b_value(baud);
	if(b_baud == -1) { //custom baud rate
		b_baud = baud; // this may or may not work at all	
	}
	cfsetospeed (&tty, b_baud);
	cfsetispeed (&tty, b_baud);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched baud tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	if(parity == 1)
		tty.c_cflag |= PARENB|PARODD;
	else if(parity == 2)
		tty.c_cflag |= PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ISIG;
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR;

	if (tcsetattr (fd->fd, TCSANOW, &tty) != 0)
	{
		//error_message ("error %d from tcsetattr", errno);
	}

#endif

	return;
}


gserialOsHandleType* gserial_os_open(char* path) {
#ifdef _WIN32
	char fpath[128];
	snprintf(fpath, 128, "\\\\.\\%s", path);
	gserialOsHandleType fd = CreateFile(fpath, GENERIC_READ|GENERIC_WRITE,
			0, 0, OPEN_EXISTING, 0, 0);
#else
	gserialOsHandleType* fd = (gserialOsHandleType*)malloc(sizeof(gserialOsHandleType));;
   	fd->fd = open(path, O_RDWR |O_NOCTTY | O_SYNC);
#endif
	return fd;
}

void gserial_os_close(gserialOsHandleType* fd) {
#ifdef _WIN32
	CloseHandle(fd->fd);
#else
	close(fd->fd);
	fd->fd = -1;
#endif
}

int gserial_os_is_open(gserialOsHandleType* fd) {
#ifdef _WIN32
	if(fd->fd != INVALID_HANDLE_VALUE)
		return 1;
#else
	if(fd->fd != -1)
		return 1; 
#endif
	return 0;
}

int gserial_os_available(gserialOsHandleType* fd) {
	int bytes_avail = 0;
#ifdef _WIN32
	COMSTAT com_stat;
	ClearCommError(fd->fd, NULL, &com_stat);
	return com_stat.cbInQue;
#else
	ioctl(fd->fd, FIONREAD, &bytes_avail);
#endif
	return bytes_avail;

}

int gserial_os_read(gserialOsHandleType* fd, char* bytes, int len)
{
#ifdef _WIN32
	DWORD bytes_read;
	ReadFile(fd->fd, (char*)bytes, len, &bytes_read, NULL);
	return bytes_read;
#else
	int r = read(fd->fd, bytes, len);
	return r;
#endif
}

int gserial_os_write(gserialOsHandleType* fd, char* bytes, int len)
{
#ifdef _WIN32
	DWORD bw;
	WriteFile(fd->fd, (char*)bytes, len, &bw, NULL);
	return bw;
#else
	return write(fd->fd, (char*)bytes, len);
#endif

}


