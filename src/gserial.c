#include "gserial.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32

#include <Windows.h>

#else

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#endif



static void port_enum_clear_func(gpointer data)
{
    g_free(data);
}

GArray* gserial_enumerate_ports()
{
    GArray* port_arr = g_array_new(FALSE, FALSE, sizeof(gchar*));
    g_array_set_clear_func(port_arr, port_enum_clear_func);

    return port_arr;
}


struct _gserialPortPrivate
{
#ifdef _WIN32
    HANDLE fd;
#else
    int fd;
#endif

    gserialByteSize byte_size;
    gserialParity parity;
    int baud;
    int timeout;
    gchar *msg;
};



#define GSERIAL_PORT_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE ((obj), GSERIAL_TYPE_PORT, gserialPortPrivate))

enum {
    PROP_0,
    PROP_BYTESIZE,
    PROP_PARITY,
    PROP_BAUD,
    PROP_TIMEOUT,
    N_PROPERTIES
};


static void gserial_port_get_property(GObject* object,
                                      guint property_id,
                                      GValue *value,
                                      GParamSpec *pspec)
{
    gserialPort *port = GSERIAL_PORT(object);

    switch (property_id)
    {
    case PROP_BYTESIZE:
        g_value_set_int(value, port->priv->byte_size);
        break;
    case PROP_PARITY:
        g_value_set_int(value, port->priv->parity);
        break;
    case PROP_BAUD:
        g_value_set_int(value, port->priv->baud);
        break;
    case PROP_TIMEOUT:
        g_value_set_int(value, port->priv->timeout);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
    }
}

static void gserial_port_set_property (GObject      *object,
                                       guint         property_id,
                                       const GValue *value,
                                       GParamSpec   *pspec)
{
    gserialPort *port = GSERIAL_PORT(object);

    switch (property_id)
    {
    case PROP_BYTESIZE:
        port->priv->byte_size = g_value_get_int(value);
        break;
    case PROP_PARITY:
        port->priv->parity = g_value_get_int(value);
        break;
    case PROP_BAUD:
        port->priv->baud = g_value_get_int(value);
        break;
    case PROP_TIMEOUT:
        port->priv->timeout = g_value_get_int(value);
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
    }
}


static void
gserial_port_class_init(gserialPortClass *klass)
{
    g_type_class_add_private (klass, sizeof (gserialPortPrivate));

    GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
    gobject_class->set_property = gserial_port_set_property;
    gobject_class->get_property = gserial_port_get_property;

    g_object_class_install_property(gobject_class, PROP_BYTESIZE,
                                    g_param_spec_int("byte_size", "ByteSize", "ByteSize",
                                            GSERIAL_BYTE_SIZE_FIVEBITS, GSERIAL_BYTE_SIZE_EIGHTBITS,
                                            GSERIAL_BYTE_SIZE_EIGHTBITS,
                                            G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

    g_object_class_install_property(gobject_class, PROP_PARITY,
                                    g_param_spec_int("parity", "Parity", "Parity",
                                            GSERIAL_PARITY_NONE, GSERIAL_PARITY_EVEN,
                                            GSERIAL_PARITY_NONE,
                                            G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

    g_object_class_install_property(gobject_class, PROP_BAUD,
                                    g_param_spec_int("baud", "Baud", "Baud",
                                            0, 230400,
                                            57600,
                                            G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

    g_object_class_install_property(gobject_class, PROP_BAUD,
                                    g_param_spec_int("timeout", "Timeout", "Timeout",
                                            0, 10000000,
                                            1000,
                                            G_PARAM_READWRITE | G_PARAM_CONSTRUCT));
}


static void
gserial_port_init(gserialPort* self)
{
    gserialPortPrivate *priv;
    self->priv = priv = GSERIAL_PORT_GET_PRIVATE(self);
}

GType gserial_port_get_type (void)
{
    static volatile gsize gserial_port_type_id_volatile = 0;
    if (g_once_init_enter (&gserial_port_type_id_volatile))
    {
        static const GTypeInfo g_define_type_info =
        {   sizeof (gserialPortClass), (GBaseInitFunc) NULL, (GBaseFinalizeFunc) NULL,
            (GClassInitFunc) gserial_port_class_init, (GClassFinalizeFunc) NULL, NULL,
            sizeof (gserialPort), 0, (GInstanceInitFunc) gserial_port_init, NULL
        };
        GType gserial_port_type_id;
        gserial_port_type_id = g_type_register_static (G_TYPE_OBJECT, "gserialPort", &g_define_type_info, 0);
        g_once_init_leave (&gserial_port_type_id_volatile, gserial_port_type_id);
    }
    return gserial_port_type_id_volatile;
}


static void set_interface_attr(gserialPort* self)
{
#ifdef _WIN32
    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);

    if (!GetCommState (self->priv->fd, &dcb))
    {
        g_warn_if_reached();
        return;
    }

    dcb.BaudRate  = self->priv->baud;
    if(self->priv->byte_size == GSERIAL_BYTE_SIZE_FIVEBITS)
        dcb.ByteSize  = 5;
    else if(self->priv->byte_size == GSERIAL_BYTE_SIZE_FIVEBITS)
        dcb.ByteSize = 6;
    else if(self->priv->byte_size == GSERIAL_BYTE_SIZE_SEVENBITS)
        dcb.ByteSize = 7;
    else if(self->priv->byte_size == GSERIAL_BYTE_SIZE_EIGHTBITS)
        dcb.ByteSize = 8;

    if(self->priv->parity == GSERIAL_PARITY_NONE)
        dcb.Parity = NOPARITY;
    else if(self->priv->parity == GSERIAL_PARITY_EVEN)
        dcb.Parity = EVENPARITY;
    else if(self->priv->parity == GSERIAL_PARITY_ODD)
        dcb.Parity = ODDPARITY;

    dcb.StopBits  = ONESTOPBIT;
    if (!SetCommState(self->priv->fd, &dcb))
    {
        g_warn_if_reached();
    }
#else

    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (self->priv->fd, &tty) != 0)
    {
        //error_message ("error %d from tcgetattr", errno);
        return;
    }

    cfsetospeed (&tty, self->priv->baud);
    cfsetispeed (&tty, self->priv->baud);

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
    if(self->priv->parity == GSERIAL_PARITY_ODD)
        tty.c_cflag |= PARENB|PARODD;
    else if(self->priv->parity == GSERIAL_PARITY_EVEN)
        tty.c_cflag |= PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (self->priv->fd, TCSANOW, &tty) != 0)
    {
        //error_message ("error %d from tcsetattr", errno);
    }

#endif

    return;
}


void gserial_port_set_byte_size(gserialPort* self, gserialByteSize sz)
{
    g_return_if_fail (GSERIAL_IS_PORT(self));
    self->priv->byte_size = sz;
}

void gserial_port_set_parity(gserialPort* self, gserialParity p)
{
    g_return_if_fail (GSERIAL_IS_PORT(self));
    self->priv->parity = p;
}

void gserial_port_set_timeout(gserialPort* self, int timeout)
{
    g_return_if_fail (GSERIAL_IS_PORT(self));
    self->priv->timeout = timeout;
}

void gserial_port_set_baud(gserialPort* self, int baud)
{
    g_return_if_fail (GSERIAL_IS_PORT(self));
    self->priv->baud = baud;
}

void gserial_port_open(gserialPort* self, const gchar* path, GError** error)
{
    g_return_if_fail (GSERIAL_IS_PORT(self));

#ifdef _WIN32
    self->priv->fd = CreateFile(path, GENERIC_READ|GENERIC_WRITE,
                                0, 0, OPEN_EXISTING, 0, 0);

    g_warn_if_fail(self->priv->fd != INVALID_HANDLE_VALUE);

    if(self->priv->fd == INVALID_HANDLE_VALUE)
    {
        LPVOID lpMsgBuf;
        DWORD dw = GetLastError();
        FormatMessage(
            FORMAT_MESSAGE_ALLOCATE_BUFFER |
            FORMAT_MESSAGE_FROM_SYSTEM |
            FORMAT_MESSAGE_IGNORE_INSERTS,
            NULL,
            dw,
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            (LPTSTR) &lpMsgBuf,
            0, NULL );
        printf("%s\n", (char*)lpMsgBuf);
        LocalFree(lpMsgBuf);
    }
#else
    self->priv->fd = open(path, O_RDWR |O_NOCTTY | O_SYNC);
#endif
    set_interface_attr(self);
}

gboolean gserial_port_is_open(gserialPort* self)
{
    g_return_val_if_fail (GSERIAL_IS_PORT(self), FALSE);

    if(self->priv->fd != 0)
        return TRUE;

    return FALSE;
}

void gserial_port_close(gserialPort* self)
{
    g_return_if_fail(GSERIAL_IS_PORT(self));
#ifdef _WIN32
    CloseHandle(self->priv->fd);
#else
    close(self->priv->fd);
#endif
    self->priv->fd = 0;
}

guint gserial_port_available(gserialPort* self)
{
    g_return_val_if_fail (GSERIAL_IS_PORT(self), 0);
    guint bytes_avail = 0;
#ifdef _WIN32
    COMSTAT com_stat;
    ClearCommError(self->priv->fd, NULL, &com_stat);
    return com_stat.cbInQue;
#else
    ioctl(self->priv->fd, FIONREAD, &bytes_avail);
#endif
    return bytes_avail;
}

void gserial_port_read_bytes(gserialPort* self, guint len_in, GArray** garray)
{
    g_return_if_fail (GSERIAL_IS_PORT(self));
    char* arr = malloc(len_in);
    guint len_r;
#ifdef _WIN32
    DWORD bytes_read;
    ReadFile(self->priv->fd, arr, len_in, &bytes_read, NULL);
    len_r = bytes_read;
#else
    len_r = read(self->priv->fd, arr, len_in);
#endif
    *garray = g_array_new (FALSE, FALSE, sizeof (gchar));
    g_array_append_vals(*garray, arr, len_r);
}

void gserial_port_read_string(gserialPort* self, guint len_in, gchar** str)
{
    g_return_if_fail (GSERIAL_IS_PORT(self));
    gchar* s = g_malloc0(len_in+1);

    guint len_r;
#ifdef _WIN32
    DWORD bytes_read;
    ReadFile(self->priv->fd, s, len_in, &bytes_read, NULL);
    len_r = bytes_read;
#else
    len_r = read(self->priv->fd, s, len_in);
#endif

    if((len_r >= 0) && (len_r < len_in))
        s[len_r] = '\0';
    else
        s[len_in-1] = '\0';

    *str = g_strdup(s);
    g_free(s);
}

void gserial_port_read_bytes_until(gserialPort* self, gchar stop_char, GArray** garray)
{
    g_return_if_fail (GSERIAL_IS_PORT(self));
    *garray = g_array_new (FALSE, FALSE, sizeof (gchar));

    while(gserial_port_available(self))
    {
        gchar c;
#ifdef _WIN32
        DWORD bytes_read;
        ReadFile(self->priv->fd, &c, 1, &bytes_read, NULL);
#else
        int bytes_read = read(self->priv->fd, &c, 1);
#endif
        if(bytes_read == 0)
            break;

        g_array_append_val(*garray, c);
        if(c == stop_char)
            break;
    }
}

gint gserial_port_write(gserialPort* self, GArray* garray)
{
    g_return_val_if_fail(GSERIAL_IS_PORT(self), 0);
#ifdef _WIN32
    printf("writing %i bytes\n", garray->len);
    DWORD bw;
    WriteFile(self->priv->fd, garray->data, garray->len, &bw, NULL);
    return bw;
#else
    return write(self->priv->fd, garray->data, garray->len);
#endif
}

void gserial_port_get_stuff(gserialPort* self, GArray** garray)
{
    g_return_if_fail(GSERIAL_IS_PORT(self));

    *garray = g_array_new (FALSE, FALSE, sizeof (gchar));
    for(int i = 0; i < 5; i++)
        g_array_append_val(*garray, i);
}


gserialPort* gserial_port_new (void)
{
    return g_object_new(GSERIAL_TYPE_PORT, NULL);
}

gserialPort* gserial_port_new_with_params(gserialByteSize sz, gserialParity p, int timeout, int baud)
{
    gserialPort* port = g_object_new(GSERIAL_TYPE_PORT, NULL);
    port->priv->byte_size = sz;
    port->priv->parity = p;
    port->priv->timeout = timeout;
    port->priv->baud = baud;
    return port;
}
