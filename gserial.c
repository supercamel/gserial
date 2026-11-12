#include "GSerial-1.0.h"
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>

#define GSERIAL_HANDLE_TYPE HANDLE

#else

#define GSERIAL_HANDLE_TYPE int

#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <linux/serial.h>

#endif

struct _GSerialPort
{
	GObject parent_instance;
	GSERIAL_HANDLE_TYPE fd;
	int baud;
	guint timeout;
	gboolean nonblocking;
	GSerialBYTE_SIZE byte_size;
	GSerialPARITY parity;
	GSerialSTOP_BITS stop_bits;
	guint poll_source_id;
	gboolean disposing;
};

G_DEFINE_TYPE(GSerialPort, gserial_port, G_TYPE_OBJECT)

GType gserial_byte_size_get_type(void)
{
	static gsize type_id;
	if (g_once_init_enter(&type_id))
	{
		static const GEnumValue values[] = {
			{GSERIAL_BYTE_SIZE_FIVEBIT, "GSERIAL_BYTE_SIZE_FIVEBIT", "fivebit"},
			{GSERIAL_BYTE_SIZE_SIXBIT, "GSERIAL_BYTE_SIZE_SIXBIT", "sixbit"},
			{GSERIAL_BYTE_SIZE_SEVENBIT, "GSERIAL_BYTE_SIZE_SEVENBIT", "sevenbit"},
			{GSERIAL_BYTE_SIZE_EIGHTBIT, "GSERIAL_BYTE_SIZE_EIGHTBIT", "eightbit"},
			{0, NULL, NULL}
		};
		GType type = g_enum_register_static("GSerialBYTE_SIZE", values);
		g_once_init_leave(&type_id, type);
	}
	return type_id;
}

GType gserial_parity_get_type(void)
{
	static gsize type_id;
	if (g_once_init_enter(&type_id))
	{
		static const GEnumValue values[] = {
			{GSERIAL_PARITY_NONE, "GSERIAL_PARITY_NONE", "none"},
			{GSERIAL_PARITY_ODD, "GSERIAL_PARITY_ODD", "odd"},
			{GSERIAL_PARITY_EVEN, "GSERIAL_PARITY_EVEN", "even"},
			{0, NULL, NULL}
		};
		GType type = g_enum_register_static("GSerialPARITY", values);
		g_once_init_leave(&type_id, type);
	}
	return type_id;
}

GType gserial_stop_bits_get_type(void)
{
	static gsize type_id;
	if (g_once_init_enter(&type_id))
	{
		static const GEnumValue values[] = {
			{GSERIAL_STOP_BITS_ONE, "GSERIAL_STOP_BITS_ONE", "one"},
			{GSERIAL_STOP_BITS_TWO, "GSERIAL_STOP_BITS_TWO", "two"},
			{GSERIAL_STOP_BITS_ONEHALF, "GSERIAL_STOP_BITS_ONEHALF", "onehalf"},
			{0, NULL, NULL}
		};
		GType type = g_enum_register_static("GSerialSTOP_BITS", values);
		g_once_init_leave(&type_id, type);
	}
	return type_id;
}

enum
{
	PROP_0,
	PROP_BAUD,
	PROP_TIMEOUT,
	PROP_BYTE_SIZE,
	PROP_PARITY,
	PROP_STOP_BITS,
	LAST_PROP
};

enum
{
	CONNECTED,
	DISCONNECTED,
	DATA_RECV,
	LAST_SIGNAL
};

static guint signals[LAST_SIGNAL];
static GParamSpec *properties[LAST_PROP];

#ifdef _WIN32

#else
static int baud_to_b_value(int baud)
{
    switch (baud)
    {
    case 50: return B50;
    case 75: return B75;
    case 110: return B110;
    case 134: return B134;
    case 150: return B150;
    case 200: return B200;
    case 300: return B300;
    case 600: return B600;
    case 1200: return B1200;
    case 1800: return B1800;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
	case 500000: return B500000;
	case 1000000: return B1000000;
    default: return -1; // Indicate custom baud rate
    }
}

static int set_custom_baud(int fd, int baudrate)
{
	struct serial_struct ser;

	if (baudrate <= 0) {
		fprintf(stderr, "Invalid custom baud rate: %d\n", baudrate);
		return -1;
	}

	if (ioctl(fd, TIOCGSERIAL, &ser) < 0) {
		perror("TIOCGSERIAL failed");
		return -1;
    }

    ser.flags &= ~ASYNC_SPD_MASK;
    ser.flags |= ASYNC_SPD_CUST;
    ser.custom_divisor = ser.baud_base / baudrate;

    if (ser.custom_divisor == 0) {
        fprintf(stderr, "Invalid custom baud rate: %d\n", baudrate);
        return -1;
    }

    if (ioctl(fd, TIOCSSERIAL, &ser) < 0) {
        perror("TIOCSSERIAL failed");
        return -1;
    }

    return 0;
}
#endif

static gboolean set_interface_attr(
#ifdef _WIN32
	GSERIAL_HANDLE_TYPE fd,
#else
	int fd,
#endif
	int baud,
	guint timeout,
	GSerialBYTE_SIZE byte_size,
	GSerialPARITY parity,
	GSerialSTOP_BITS stop_bits)
{
	if (baud <= 0)
	{
		g_warning("Invalid baud rate: %d", baud);
		return FALSE;
	}

#ifdef _WIN32
	DCB dcb = {0};
	dcb.DCBlength = sizeof(DCB);

	if (!GetCommState(fd, &dcb))
	{
		g_warn_if_reached();
		return FALSE;
	}

	dcb.BaudRate = baud;
	switch (byte_size)
	{
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
	default:
		g_warning("Invalid byte size");
		return FALSE;
	}

	if (parity == GSERIAL_PARITY_NONE)
		dcb.Parity = NOPARITY;
	else if (parity == GSERIAL_PARITY_ODD)
		dcb.Parity = ODDPARITY;
	else if (parity == GSERIAL_PARITY_EVEN)
		dcb.Parity = EVENPARITY;
	else
	{
		g_warning("Invalid parity");
		return FALSE;
	}

	if (stop_bits == GSERIAL_STOP_BITS_ONE)
	{
		dcb.StopBits = ONESTOPBIT;
	}
	else if (stop_bits == GSERIAL_STOP_BITS_ONEHALF)
	{
		dcb.StopBits = ONE5STOPBITS;
	}
	else if (stop_bits == GSERIAL_STOP_BITS_TWO)
	{
		dcb.StopBits = TWOSTOPBITS;
	}
	else
	{
		g_warning("Invalid stop bits");
		return FALSE;
	}

	dcb.fBinary = TRUE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	dcb.BaudRate = baud;

	if (!SetCommState(fd, &dcb))
	{
		g_warn_if_reached();
		return FALSE;
	}

	if (!SetCommMask(fd, 0x1F9))
	{
		g_warn_if_reached();
		return FALSE;
	}

	COMMTIMEOUTS timeouts = {0};
	timeouts.ReadIntervalTimeout = (DWORD)timeout;
	timeouts.ReadTotalTimeoutConstant = (DWORD)timeout;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = (DWORD)timeout;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	if (SetCommTimeouts(fd, &timeouts) == FALSE)
	{
		printf_s("\nError to Setting Time outs");
		g_warn_if_reached();
		return FALSE;
	}

#else
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		perror("tcgetattr failed");
		return FALSE;
	}

	int b_baud = baud_to_b_value(baud);
	if (b_baud == -1) { // Custom baud rate
		if (set_custom_baud(fd, baud) < 0) {
			fprintf(stderr, "Failed to set custom baud rate: %d\n", baud);
			return FALSE;
		}
	} else {
		cfsetospeed(&tty, b_baud);
        cfsetispeed(&tty, b_baud);
    }

    switch (byte_size)
    {
    case GSERIAL_BYTE_SIZE_FIVEBIT:
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS5;
        break;
    case GSERIAL_BYTE_SIZE_SIXBIT:
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS6;
        break;
    case GSERIAL_BYTE_SIZE_SEVENBIT:
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS7;
        break;
	case GSERIAL_BYTE_SIZE_EIGHTBIT:
		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
		break;
	default:
		fprintf(stderr, "Invalid byte size\n");
		return FALSE;
	}

	tty.c_iflag &= ~IGNBRK;
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = (cc_t)((MIN(timeout, 25500u) + 99u) / 100u);
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_cflag |= (CLOCAL | CREAD);

	tty.c_cflag &= ~(PARENB | PARODD);
	if (parity == GSERIAL_PARITY_NONE)
		;
	else if (parity == GSERIAL_PARITY_ODD)
		tty.c_cflag |= (PARENB | PARODD);
	else if (parity == GSERIAL_PARITY_EVEN)
		tty.c_cflag |= PARENB;
	else
	{
		g_warning("Invalid parity");
		return FALSE;
	}

	if (stop_bits == GSERIAL_STOP_BITS_ONE)
		tty.c_cflag &= ~CSTOPB;
	else if (stop_bits == GSERIAL_STOP_BITS_TWO)
		tty.c_cflag |= CSTOPB;
	else
	{
		g_warning("Invalid stop bits");
		return FALSE;
	}

	tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		perror("tcsetattr failed");
		return FALSE;
	}
#endif

	return TRUE;
}

static gboolean poll_for_data(GSerialPort *self)
{
	/* Signal handlers may release the owner or close and reopen the port. */
	g_object_ref(self);
	guint source_id = self->poll_source_id;

	if (gserial_port_is_open(self))
	{
		guint bytes_available = gserial_port_bytes_available(self);
		if (bytes_available > 0 && self->poll_source_id == source_id)
		{
			g_signal_emit(self, signals[DATA_RECV], 0, bytes_available);
		}
	}

	gboolean keep_polling = self->poll_source_id == source_id &&
		gserial_port_is_open(self);
	g_object_unref(self);
	return keep_polling;
}

static void stop_polling(GSerialPort *self)
{
	if (self->poll_source_id != 0)
	{
		guint source_id = self->poll_source_id;
		self->poll_source_id = 0;
		/* GLib also permits removing the source currently being dispatched. */
		g_source_remove(source_id);
	}
}

static void gserial_port_get_property(GObject *object,
									  guint property_id,
									  GValue *value,
									  GParamSpec *pspec)
{
	GSerialPort *self = GSerial_Port(object);

	switch (property_id)
	{
	case PROP_BAUD:
		g_value_set_uint(value, self->baud);
		break;
	case PROP_TIMEOUT:
		g_value_set_uint(value, self->timeout);
		break;
	case PROP_BYTE_SIZE:
		g_value_set_uint(value, self->byte_size);
		break;
	case PROP_PARITY:
		g_value_set_uint(value, self->parity);
		break;
	case PROP_STOP_BITS:
		g_value_set_uint(value, self->stop_bits);
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
	}
}

static void gserial_port_set_property(GObject *object,
									  guint property_id,
									  const GValue *value,
									  GParamSpec *pspec)
{
	GSerialPort *self = GSerial_Port(object);

	switch (property_id)
	{
	case PROP_BAUD:
		gserial_port_set_baud(self, g_value_get_uint(value));
		break;
	case PROP_TIMEOUT:
		gserial_port_set_timeout(self, g_value_get_uint(value));
		break;
	case PROP_BYTE_SIZE:
		gserial_port_set_byte_size(self, g_value_get_uint(value));
		break;
	case PROP_PARITY:
		gserial_port_set_parity(self, g_value_get_uint(value));
		break;
	case PROP_STOP_BITS:
		gserial_port_set_stop_bits(self, g_value_get_uint(value));
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
	}
}

static void gserial_port_dispose(GObject *object)
{
	GSerialPort *self =
		G_TYPE_CHECK_INSTANCE_CAST(object, GSERIAL_TYPE_PORT, GSerialPort);

	self->disposing = TRUE;
	if (gserial_port_is_open(self))
	{
		gserial_port_close(self);
	}
	else
	{
		stop_polling(self);
	}

	G_OBJECT_CLASS(gserial_port_parent_class)->dispose(object);
}

static void gserial_port_finalize(GObject *object)
{
	/*
	   GSerialPort *self =
	   G_TYPE_CHECK_INSTANCE_CAST(object, GSERIAL_TYPE_PORT, GSerialPort);
	   */

	G_OBJECT_CLASS(gserial_port_parent_class)->finalize(object);
}

static void gserial_port_class_init(GSerialPortClass *klass)
{
	GObjectClass *gobject_class = G_OBJECT_CLASS(klass);

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

	signals[DATA_RECV] = g_signal_new("on_data",
									  G_TYPE_FROM_CLASS(gobject_class),
									  G_SIGNAL_RUN_LAST,
									  0,
									  NULL, NULL, NULL,
									  G_TYPE_NONE, 1, G_TYPE_INT);

	gobject_class->dispose = gserial_port_dispose;
	gobject_class->finalize = gserial_port_finalize;
	gobject_class->get_property = gserial_port_get_property;
	gobject_class->set_property = gserial_port_set_property;

	properties[PROP_BAUD] =
		g_param_spec_uint("baud",
						  "Baud",
						  "Serial baud rate",
						  1,
						  G_MAXUINT,
						  57600,
						  G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS | G_PARAM_EXPLICIT_NOTIFY);
	properties[PROP_TIMEOUT] =
		g_param_spec_uint("timeout",
						  "Timeout",
						  "Read and write timeout in milliseconds",
						  0,
						  G_MAXUINT,
						  500,
						  G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS | G_PARAM_EXPLICIT_NOTIFY);
	properties[PROP_BYTE_SIZE] =
		g_param_spec_uint("byte-size",
						  "Byte Size",
						  "Number of data bits",
						  GSERIAL_BYTE_SIZE_FIVEBIT,
						  GSERIAL_BYTE_SIZE_EIGHTBIT,
						  GSERIAL_BYTE_SIZE_EIGHTBIT,
						  G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS | G_PARAM_EXPLICIT_NOTIFY);
	properties[PROP_PARITY] =
		g_param_spec_uint("parity",
						  "Parity",
						  "Parity setting",
						  GSERIAL_PARITY_NONE,
						  GSERIAL_PARITY_EVEN,
						  GSERIAL_PARITY_NONE,
						  G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS | G_PARAM_EXPLICIT_NOTIFY);
	properties[PROP_STOP_BITS] =
		g_param_spec_uint("stop-bits",
						  "Stop Bits",
						  "Stop bit setting",
						  GSERIAL_STOP_BITS_ONE,
						  GSERIAL_STOP_BITS_ONEHALF,
						  GSERIAL_STOP_BITS_ONE,
						  G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS | G_PARAM_EXPLICIT_NOTIFY);
	g_object_class_install_properties(gobject_class, LAST_PROP, properties);
}

static void gserial_port_init(GSerialPort *self)
{
#ifdef _WIN32
	self->fd = INVALID_HANDLE_VALUE;
#else
	self->fd = -1;
#endif

	self->baud = 57600;
	self->byte_size = GSERIAL_BYTE_SIZE_EIGHTBIT;
	self->parity = GSERIAL_PARITY_NONE;
	self->stop_bits = GSERIAL_STOP_BITS_ONE;
	self->timeout = 500;
	self->poll_source_id = 0;
}

GSerialPort *gserial_port_new()
{
	return g_object_new(GSERIAL_TYPE_PORT, NULL);
}

GSerialPort *gserial_port_new_with_params(
	GSerialBYTE_SIZE byte_size,
	GSerialPARITY parity,
	guint timeout,
	guint baud)
{
	GSerialPort *self = gserial_port_new();
	gserial_port_set_byte_size(self, byte_size);
	gserial_port_set_parity(self, parity);
	gserial_port_set_timeout(self, timeout);
	gserial_port_set_baud(self, baud);

	return self;
}

void gserial_port_set_nonblocking(GSerialPort *self, gboolean enabled)
{
	g_return_if_fail(GSerial_IS_Port(self));
	g_return_if_fail(!gserial_port_is_open(self));
	self->nonblocking = enabled;
}

gboolean gserial_port_open(GSerialPort *self, gchar *path)
{
	g_return_val_if_fail(GSerial_IS_Port(self), FALSE);
	g_return_val_if_fail(path != NULL, FALSE);
	if (self->disposing)
		return FALSE;

	if (gserial_port_is_open(self))
	{
		g_warning("gserial_port_open called while the port is already open");
		return FALSE;
	}

#ifdef _WIN32
	char fpath[128];
	snprintf(fpath, 128, "\\\\.\\%s", path);
	self->fd = CreateFile(fpath, GENERIC_READ | GENERIC_WRITE,
						  0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

	if (self->fd == INVALID_HANDLE_VALUE)
	{
		return FALSE;
	}

	if (!set_interface_attr(self->fd,
							self->baud,
							self->timeout,
							self->byte_size,
							self->parity,
							self->stop_bits))
	{
		CloseHandle(self->fd);
		self->fd = INVALID_HANDLE_VALUE;
		return FALSE;
	}

	// self->thread = g_thread_new(NULL, (GThreadFunc)read_thread, self);

#else
	self->fd = open(path, O_RDWR | O_NOCTTY | (self->nonblocking ? O_NONBLOCK : O_SYNC));
	if (self->fd == -1)
	{
		return FALSE;
	}

	ioctl(self->fd, TIOCNXCL);

	if (!set_interface_attr(self->fd,
							self->baud,
							self->timeout,
							self->byte_size,
							self->parity,
							self->stop_bits))
	{
		close(self->fd);
		self->fd = -1;
		return FALSE;
	}

	// Flush any data in the buffer
    tcflush(self->fd, TCIOFLUSH);
#endif

	self->poll_source_id = g_timeout_add(1, (GSourceFunc)poll_for_data, (gpointer)self);
	g_signal_emit(self, signals[CONNECTED], 0);
	return TRUE;
}

void gserial_port_close(GSerialPort *self)
{
	g_return_if_fail(GSerial_IS_Port(self));

	if (!gserial_port_is_open(self))
	{
		stop_polling(self);
		return;
	}

	stop_polling(self);

#ifdef _WIN32
	CloseHandle(self->fd);
	self->fd = INVALID_HANDLE_VALUE;
#else
	close(self->fd);
	self->fd = -1;
#endif
	g_signal_emit(self, signals[DISCONNECTED], 0);
}

gboolean gserial_port_is_open(GSerialPort *self)
{
	g_return_val_if_fail(self != NULL, FALSE);
#ifdef _WIN32
	if (self->fd != INVALID_HANDLE_VALUE)
	{
		return TRUE;
	}
#else
	if (self->fd != -1)
	{
		return TRUE;
	}
#endif
	return FALSE;
}

guint gserial_port_bytes_available(GSerialPort *self)
{
	if (gserial_port_is_open(self))
	{
#ifdef _WIN32
		COMSTAT cs;
			if (ClearCommError(self->fd, NULL, &cs) == FALSE)
			{
				gserial_port_close(self);
				return 0;
			}
			return cs.cbInQue;
		// return (4096 + self->rb_end - self->rb_start) % 4096;
#else
		int bytes_available;
		int r = ioctl(self->fd, FIONREAD, &bytes_available);
		if (r == -1)
		{
			gserial_port_close(self);
			return 0;
		}
		else
		{
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

guint8 *gserial_port_read_bytes(GSerialPort *self, guint len, gint *result_length)
{
	if (gserial_port_is_open(self))
	{
		guint8 *buf = (guint8 *)g_malloc(len + 1);
#ifdef _WIN32
		/*
		guint counter = 0;
		guint available = gserial_port_bytes_available(self);
		//printf("available: %u\n", available);
		if(len >= available) {
			*result_length = 0;
			g_free(buf);
			return NULL;
		}

		//g_mutex_lock(&self->mutex);
		while(counter != len) {
			buf[counter++] = self->buffer[self->rb_start];
			self->rb_start = (self->rb_start + 1) % 4096;
		}
		//g_mutex_unlock(&self->mutex);
		*result_length = counter;
		*/

		guint i = 0;
		for (; i < len; i++)
		{
			DWORD bytes_read = 0;
			ReadFile(self->fd, &buf[i], 1, &bytes_read, NULL);
			if (bytes_read != 1)
			{
				break;
			}
		}
		/*
		   DWORD dwlen = len;
		   DWORD bytes_read = 0;
		   ReadFile(self->fd, buf, dwlen, &bytes_read, NULL);
		   */
		*result_length = i;
#else
			*result_length = read(self->fd, buf, len);
			if (*result_length == -1)
			{
				if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
					gserial_port_close(self);
				*result_length = 0;
				g_free(buf);
				return NULL;
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

gchar *gserial_port_read_string(GSerialPort *self, guint len)
{
	if (!gserial_port_is_open(self))
	{
		g_warning("gserial_port_read_string failed because the port is not open");
		return NULL;
	}

	// Use read_bytes to read the data
	gint result_length = 0;
	guint8 *data = gserial_port_read_bytes(self, len, &result_length);

	if (data == NULL || result_length == 0)
	{
		g_warning("Failed to read data from the serial port");
		g_free(data);
		return NULL;
	}

	// Allocate memory for the string and ensure null-termination
	gchar *ret = (gchar *)g_malloc(result_length + 1); // +1 for null-terminator
	memcpy(ret, data, result_length);
	ret[result_length] = '\0'; // Null-terminate the string

	// Free the original data buffer
	g_free(data);

	return ret;
}

gchar *gserial_port_read_line(GSerialPort *self)
{
	if (!gserial_port_is_open(self))
	{
		g_warning("gserial_port_read_line failed because the port is not open");
		return NULL;
	}

	GString *line_buffer = g_string_new(NULL); // Dynamic buffer to store the line
	gboolean line_complete = FALSE;			   // Flag to indicate if a complete line has been read

	while (!line_complete)
	{
		// Read one character at a time
		gint result_length = 0;
		guint8 *data = gserial_port_read_bytes(self, 1, &result_length);

		if (data == NULL || result_length == 0)
		{
			// No more data available or an error occurred
			g_free(data);
			break;
		}

		// Append the character to the buffer
		g_string_append_c(line_buffer, (gchar)data[0]);

		// Check for newline character
		if (data[0] == '\n')
		{
			line_complete = TRUE;
		}

		g_free(data); // Free the temporary data buffer
	}

	// Return the line as a null-terminated string
	if (line_buffer->len > 0)
	{
		return g_string_free(line_buffer, FALSE); // Transfer ownership of the buffer
	}
	else
	{
		g_string_free(line_buffer, TRUE); // Free the buffer if no data was read
		return NULL;
	}
}

#ifndef _WIN32
static gint write_with_timeout(int fd, const guint8 *bytes, guint length,
		guint timeout, gboolean nonblocking)
{
	if (nonblocking)
	{
		ssize_t written = write(fd, bytes, length);
		if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
			return 0;
		return (gint)written;
	}

	/* A readiness check alone cannot bound a subsequent blocking write. */
	int flags = fcntl(fd, F_GETFL);
	if (flags == -1 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1)
		return -1;

	gint64 deadline = g_get_monotonic_time() + (gint64)timeout * 1000;
	gint total = 0;
	while ((guint)total < length)
	{
		ssize_t written = write(fd, bytes + total, length - (guint)total);
		if (written > 0)
			total += (gint)written;
		else if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
		{
			total = -1;
			break;
		}
		if ((guint)total == length)
			break;

		gint64 remaining = deadline - g_get_monotonic_time();
		if (remaining <= 0)
			break;
		struct pollfd poll_fd = {fd, POLLOUT, 0};
		int wait_ms = (int)MIN((remaining + 999) / 1000, G_MAXINT);
		int ready = poll(&poll_fd, 1, wait_ms);
		if (ready < 0 && errno == EINTR)
			continue;
		if (ready < 0 || (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)))
		{
			total = -1;
			break;
		}
		if (ready == 0 && g_get_monotonic_time() >= deadline)
			break;
	}

	/* Restore blocking reads before returning or emitting a disconnection. */
	if (fcntl(fd, F_SETFL, flags) == -1)
		return -1;
	return total;
}
#endif

gint gserial_port_write_bytes(GSerialPort *self, guint8 *bytes, guint length)
{
	g_return_val_if_fail(GSerial_IS_Port(self), -1);
	g_return_val_if_fail(bytes != NULL || length == 0, -1);
	/* The public result is signed; larger buffers can be sent in chunks. */
	length = MIN(length, G_MAXINT);
	if (gserial_port_is_open(self))
	{
#ifdef _WIN32
		DWORD bw = 0;
		if (!WriteFile(self->fd, bytes, length, &bw, NULL))
		{
			gserial_port_close(self);
			return -1;
		}
		return (gint)bw;
#else
		int r = write_with_timeout(self->fd, bytes, length, self->timeout, self->nonblocking);
		if (r == -1)
		{
			gserial_port_close(self);
		}
		return r;
#endif
	}
	else
	{
		g_warning("gserial_port_write_bytes failed because the port is not open");
		return 0;
	}
}

gint gserial_port_write_string(GSerialPort *self, const gchar *s)
{
	g_return_val_if_fail(s != NULL, -1);
	return gserial_port_write_bytes(self, (guint8 *)s, (guint)MIN(strlen(s), G_MAXINT));
}

guint gserial_port_get_baud(GSerialPort *self)
{
	g_return_val_if_fail(self != NULL, 0);
	return self->baud;
}

void gserial_port_set_baud(GSerialPort *self, guint baud)
{
	g_return_if_fail(GSerial_IS_Port(self));
	g_return_if_fail(baud > 0);

	if ((guint)self->baud == baud)
		return;
	g_return_if_fail(!gserial_port_is_open(self));

	self->baud = baud;
	g_object_notify_by_pspec(G_OBJECT(self), properties[PROP_BAUD]);
}

GSerialBYTE_SIZE gserial_port_get_byte_size(GSerialPort *self)
{
	g_return_val_if_fail(self != NULL, 0);
	return self->byte_size;
}

void gserial_port_set_byte_size(GSerialPort *self, GSerialBYTE_SIZE byte_size)
{
	g_return_if_fail(GSerial_IS_Port(self));
	g_return_if_fail(byte_size >= GSERIAL_BYTE_SIZE_FIVEBIT && byte_size <= GSERIAL_BYTE_SIZE_EIGHTBIT);

	if (self->byte_size == byte_size)
		return;
	g_return_if_fail(!gserial_port_is_open(self));

	self->byte_size = byte_size;
	g_object_notify_by_pspec(G_OBJECT(self), properties[PROP_BYTE_SIZE]);
}

GSerialPARITY gserial_port_get_parity(GSerialPort *self)
{
	g_return_val_if_fail(self != NULL, 0);
	return self->parity;
}

void gserial_port_set_parity(GSerialPort *self, GSerialPARITY parity)
{
	g_return_if_fail(GSerial_IS_Port(self));
	g_return_if_fail(parity >= GSERIAL_PARITY_NONE && parity <= GSERIAL_PARITY_EVEN);

	if (self->parity == parity)
		return;
	g_return_if_fail(!gserial_port_is_open(self));

	self->parity = parity;
	g_object_notify_by_pspec(G_OBJECT(self), properties[PROP_PARITY]);
}

GSerialSTOP_BITS gserial_port_get_stop_bits(GSerialPort *self)
{
	g_return_val_if_fail(self != NULL, 0);
	return self->stop_bits;
}

void gserial_port_set_stop_bits(GSerialPort *self, GSerialSTOP_BITS stop_bits)
{
	g_return_if_fail(GSerial_IS_Port(self));
	g_return_if_fail(stop_bits >= GSERIAL_STOP_BITS_ONE && stop_bits <= GSERIAL_STOP_BITS_ONEHALF);

	if (self->stop_bits == stop_bits)
		return;
	g_return_if_fail(!gserial_port_is_open(self));

	self->stop_bits = stop_bits;
	g_object_notify_by_pspec(G_OBJECT(self), properties[PROP_STOP_BITS]);
}

guint gserial_port_get_timeout(GSerialPort *self)
{
	g_return_val_if_fail(self != NULL, 0);
	return self->timeout;
}

void gserial_port_set_timeout(GSerialPort *self, guint timeout)
{
	g_return_if_fail(GSerial_IS_Port(self));

	if ((guint)self->timeout == timeout)
		return;
	g_return_if_fail(!gserial_port_is_open(self));

	self->timeout = timeout;
	g_object_notify_by_pspec(G_OBJECT(self), properties[PROP_TIMEOUT]);
}
