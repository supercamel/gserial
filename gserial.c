#include "GSerial-1.0.h"
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>

#define GSERIAL_HANDLE_TYPE HANDLE

#else

#define GSERIAL_HANDLE_TYPE FILE* 

#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>

#endif



struct _GSerialPort {
    GObject parent_instance;
    GSERIAL_HANDLE_TYPE fd;
    int baud;
    int timeout;
    GSerialBYTE_SIZE byte_size;
    GSerialPARITY parity;
    GSerialSTOP_BITS stop_bits;
};


G_DEFINE_TYPE(GSerialPort, gserial_port, G_TYPE_OBJECT)


    enum {
        PROP_0,
        LAST_PROP
    };

enum {
    CONNECTED,
    DISCONNECTED,
    DATA_READY,
    LAST_SIGNAL
};

static guint signals[LAST_SIGNAL];


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

static void set_interface_attr(
#ifdef _WIN32
        GSERIAL_HANDLE_TYPE fd, 
#else
        int fd,
#endif
        int baud, 
        GSerialBYTE_SIZE byte_size, 
        GSerialPARITY parity,
        GSerialSTOP_BITS stop_bits)
{
#ifdef _WIN32
    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);

    if (!GetCommState (fd, &dcb))
    {
        g_warn_if_reached();
        return;
    }

    dcb.BaudRate  = baud;
    switch(bytesize) {
        case GSERIAL_BYTE_SIZE_FIVEBIT:
            dcb.ByteSize = 5;
            break;
        case GSERIAL_BYTE_SIZE_SIXBIT:
            dcb.ByteSize = 6;
            break;
        case GSERIAL_BYTE_SIZE_SEVENBIT:
            dcb.ByteSize = 7;
            break;
        case GSERIAL_BYTE_SIZE_EIGHTBIT:
            dcb.ByteSize = 8;
            break;
    }

    if(parity == GSERIAL_PARITY_NONE)
        dcb.Parity = NOPARITY;
    else if(parity == GSERIAL_PARITY_ODD)
        dcb.Parity = ODDPARITY;
    else if(parity == GSERIAL_PARITY_EVEN)
        dcb.Parity = EVENPARITY;

    if(stop_bits == GSERIAL_STOP_BITS_ONE) {
        dcb.StopBits = ONESTOPBIT;
    }
    else if(stop_bits == GSERIAL_STOP_BITS_ONE_HALF) {
        dcb.StopBits = ONE5STOPBITS;
    }
    else if(stop_bits == GSERIAL_STOP_BITS_TWO) {
        dcb.StopBits = TWOSTOPBITS;
    }

    if(!SetCommState(fd, &dcb))
    {
        g_warn_if_reached();
    }
#else

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(fd, &tty) != 0)
    {
        g_warn_if_reached();
        return;
    }

    int b_baud = baud_to_b_value(baud);
    if(b_baud == -1) { //custom baud rate
        b_baud = baud; // this may or may not work at all	
    }
    cfsetospeed(&tty, b_baud);
    cfsetispeed(&tty, b_baud);

    if(byte_size == GSERIAL_BYTE_SIZE_FIVEBIT) {
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS5;
    }
    else if(byte_size == GSERIAL_BYTE_SIZE_SIXBIT) {
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS6;
    }
    else if(byte_size == GSERIAL_BYTE_SIZE_SEVENBIT) {
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS7;
    }
    else if(byte_size == GSERIAL_BYTE_SIZE_EIGHTBIT) {
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    }

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

    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    if(parity == GSERIAL_PARITY_ODD) {
        tty.c_cflag |= (PARENB|PARODD);
    }
    else if(parity == GSERIAL_PARITY_EVEN) {
        tty.c_cflag |= PARENB;
    }


    if(stop_bits == GSERIAL_STOP_BITS_ONE) {
        tty.c_cflag &= ~CSTOPB;
    }
    else {
        tty.c_cflag |= CSTOPB;
    }

    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;

    if(tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        g_warn_if_reached();
    }


#endif

    return;
}


static void gserial_port_dispose(GObject* object) 
{
    GSerialPort *self = 
        G_TYPE_CHECK_INSTANCE_CAST(object, GSERIAL_TYPE_PORT, GSerialPort);


    if(gserial_port_is_open(self))
    {
        gserial_port_close(self);
    }

    G_OBJECT_CLASS (gserial_port_parent_class)->dispose(object);
}


static void gserial_port_finalize(GObject* object)
{
    GSerialPort *self = 
        G_TYPE_CHECK_INSTANCE_CAST(object, GSERIAL_TYPE_PORT, GSerialPort);

    G_OBJECT_CLASS(gserial_port_parent_class)->finalize(object);
}


static void gserial_port_class_init(GSerialPortClass* klass) 
{
    GObjectClass* gobject_class = G_OBJECT_CLASS(klass);

    signals[CONNECTED] = g_signal_new("connected",
            G_TYPE_FROM_CLASS(gobject_class),
            G_SIGNAL_RUN_LAST,
            0,
            NULL, NULL, NULL,
            G_TYPE_NONE, 0);

    signals[DISCONNECTED] = g_signal_new("disconnected",
            G_TYPE_FROM_CLASS(gobject_class),
            G_SIGNAL_RUN_LAST,
            0,
            NULL, NULL, NULL,
            G_TYPE_NONE, 0);

    signals[DATA_READY] = g_signal_new("on_data",
            G_TYPE_FROM_CLASS(gobject_class),
            G_SIGNAL_RUN_LAST,
            0,
            NULL, NULL, NULL,
            G_TYPE_NONE, 0);

    gobject_class->dispose = gserial_port_dispose;
    gobject_class->finalize = gserial_port_finalize;
}

static void gserial_port_init(GSerialPort* self) 
{
    self->fd = NULL;
    self->baud = 57600;
    self->byte_size = GSERIAL_BYTE_SIZE_EIGHTBIT;
    self->parity = GSERIAL_PARITY_NONE;
    self->stop_bits = GSERIAL_STOP_BITS_ONE;
    self->timeout = 1000000;
}

GSerialPort* gserial_port_new()
{
    return g_object_new(GSERIAL_TYPE_PORT, NULL);
}


GSerialPort* gserial_port_new_with_params(
        GSerialBYTE_SIZE byte_size, 
        GSerialPARITY parity,
        guint timeout,
        guint baud
        )
{
    GSerialPort* self = gserial_port_new();
    gserial_port_set_byte_size(self, byte_size);
    gserial_port_set_parity(self, parity);
    gserial_port_set_timeout(self, timeout);
    gserial_port_set_baud(self, baud);

    return self;
}


gboolean gserial_port_open(GSerialPort* self, gchar* path)
{
#ifdef _WIN32
    char fpath[128];
    snprintf(fpath, 128, "\\\\.\\%s", path);
    self->fd = CreateFile(fpath, GENERIC_READ | GENERIC_WRITE,
            0, 0, OPEN_EXISTING, 0, 0);

    if(self->fd == INVALID_HANDLE_VALUE)
    {
        return FALSE;
    }
#else
    int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
    if(fd == -1) 
    {
        return FALSE;
    }

    ioctl(fd, TIOCNXCL);

    set_interface_attr(fd, 
            self->baud, 
            self->byte_size, 
            self->parity, 
            self->stop_bits);

    self->fd = fdopen(fd, "wb+");
#endif

    g_signal_emit(self, signals[CONNECTED], 0);
    return TRUE;
}

void gserial_port_close(GSerialPort* self)
{
#ifdef _WIN32
    CloseHandle(self->fd);
    self->fd = INVALID_HANDLE_VALUE;
#else
    fclose(self->fd);
    self->fd = NULL;
#endif
    g_signal_emit(self, signals[DISCONNECTED], 0);
}

gboolean gserial_port_is_open(GSerialPort* self)
{
    g_return_val_if_fail(self != NULL, FALSE);
#ifdef _WIN32
    if(self->fd != INVALID_HANDLE_VALUE) 
    {
        return TRUE;
    }
#else
    if(self->fd != NULL)
    {
        return TRUE;
    }
#endif
    return FALSE;    
}

guint gserial_port_bytes_available(GSerialPort* self)
{
    if(gserial_port_is_open(self)) {
#ifdef _WIN32
        COMSTAT com_stat;
        ClearCommError(self->fd, NULL, &com_stat);
        return com_stat.cbInQue;
#else
        int bytes_available;
        if(ioctl(fileno(self->fd), FIONREAD, &bytes_available) == -1) 
        {
            gserial_port_close(self);
            return 0;
        }
        else {
            return (guint)bytes_available;
        }
#endif
    }
    else
    {
        g_warning("gserial_port_bytes_available port is not open");
        return 0;
    }
}

guint8* gserial_port_read_bytes(GSerialPort* self, guint len, gint* result_length)
{
    if(gserial_port_is_open(self)) {
        guint8* buf = (guint8*)g_malloc(len);
#ifdef _WIN32
        DWORD bytes_read;
        ReadFile(self->fd, (char*)buf, len, &bytes_read, NULL);
        *result_length = bytes_read;
#else
        *result_length = fread(buf, len, 1, self->fd); 
        if(*result_length != len) 
        {
            if(ferror(self->fd)) 
            {
                gserial_port_close(self);
            }
        }
#endif
        return buf; 
    }
    else 
    {
        g_warning("gserial_port_read_bytes failed because the port is not open");
        *result_length = 0;
        return NULL;
    }
}

gchar* gserial_port_read_string(GSerialPort* self, guint len)
{
    if(gserial_port_is_open(self)) 
    {
        gchar* ret = (gchar*)g_malloc(len);
        int pos = 0;
        char c = -1;
#ifdef _WIN32
        do
        {
            int bytes_read;
            if((ReadFile(self->fd, &c, 1, &bytes_read, NULL) == FALSE) ||
                    (bytes_read == 0)) 
            {
                break;
            }
            ret[pos++] = c;
        }
        while((c != '\0') && (c != EOF));

#else
        while ((c = getc(self->fd)) != '\0' && c != EOF) 
        {
            ret[pos++] = c;
            if(pos >= len) 
                break;
        }

        if(ferror(self->fd)) 
        {
            gserial_port_close(self);
        }
#endif
        return ret;
    }
    else 
    {
        g_warning("gserial_port_read_string failed because the port is not open");
        return NULL;
    }
}

gint gserial_port_write_bytes(GSerialPort* self, guint8* bytes, guint length)
{
    if(gserial_port_is_open(self)) {
#ifdef _WIN32
        DWORD bw;
        WriteFile(self->fd, (char*)bytes, length, &bw, NULL);
        return (gint)bw;
#else
        int r = fwrite(bytes, length, 1, self->fd);
        if(ferror(self->fd)) 
        {
            gserial_port_close(self);
        }
        return r;
#endif
    }
    else {
        g_warning("gserial_port_write_bytes failed because the port is not open");
        return 0;
    }
}

gint gserial_port_write_string(GSerialPort* self, const gchar* s)
{
    if(gserial_port_is_open(self)) {
#ifdef _WIN32
        DWORD bw;
        WriteFile(self->fd, s, strlen(s), &bw, NULL);
        return (gint)bw;
#else
        int r = fwrite(s, strlen(s), 1, self->fd);
        if(ferror(self->fd)) 
        {
            gserial_port_close(self);
        }
        return r;
#endif
    }
    else
    {
        g_warning("gserial_port_write_string failed because the port is not open");
        return 0;
    }
}


guint gserial_port_get_baud(GSerialPort* self)
{
    g_return_val_if_fail(self != NULL, 0);
    return self->baud;
}

void gserial_port_set_baud(GSerialPort* self, guint baud)
{
    g_return_if_fail(self != NULL);
    self->baud = baud;
}

GSerialBYTE_SIZE gserial_port_get_byte_size(GSerialPort* self)
{
    g_return_val_if_fail(self != NULL, 0);
    return self->byte_size;
}

void gserial_port_set_byte_size(GSerialPort* self, GSerialBYTE_SIZE byte_size)
{
    g_return_if_fail(self != NULL);
    self->byte_size = byte_size;
}

GSerialPARITY gserial_port_get_parity(GSerialPort* self)
{
    g_return_val_if_fail(self != NULL, 0);
    return self->parity;
}

void gserial_port_set_parity(GSerialPort* self, GSerialPARITY parity)
{
    g_return_if_fail(self != NULL);
    self->parity = parity;
}

GSerialSTOP_BITS gserial_port_get_stop_bits(GSerialPort* self)
{
    g_return_val_if_fail(self != NULL, 0);
    return self->stop_bits;
}

void gserial_port_set_stop_bits(GSerialPort* self, GSerialSTOP_BITS stop_bits)
{
    g_return_if_fail(self != NULL);
    self->stop_bits = stop_bits;
}

guint gserial_port_get_timeout(GSerialPort* self)
{
    g_return_val_if_fail(self != NULL, 0);
    return self->timeout;
}

void gserial_port_set_timeout(GSerialPort* self, guint timeout)
{
    g_return_if_fail(self != NULL);
    self->timeout = timeout;
}



