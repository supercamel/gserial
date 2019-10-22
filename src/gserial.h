
#ifndef __GSERIAL_H__
#define __GSERIAL_H__

#include <glib-object.h>

G_BEGIN_DECLS

typedef enum
{
    GSERIAL_BYTE_SIZE_FIVEBITS,
    GSERIAL_BYTE_SIZE_SIXBITS,
    GSERIAL_BYTE_SIZE_SEVENBITS,
    GSERIAL_BYTE_SIZE_EIGHTBITS
} gserialByteSize;

typedef enum
{
	GSERIAL_PARITY_NONE,
	GSERIAL_PARITY_ODD,
	GSERIAL_PARITY_EVEN
} gserialParity;

typedef enum
{
    GSERIAL_STOPBITS_ONE,
    GSERIAL_STOPBITS_TWO,
    GSERIAL_STOPBITS_ONEHALF
} gserialStopBits;


typedef enum
{
	GSERIAL_ERROR_PORT_OPEN,
	GSERIAL_ERROR_PORT_READ,
	GSERIAL_ERROR_PORT_WRITE
} gserialErrorPort;
#define GSERIAL_ERROR_PORT gserial_error_port_quark()

GQuark gserial_error_port_quark(void);

typedef struct _gserialPortInfo gserialPortInfo;

struct _gserialPortInfo
{
	gchar* path;
};

#define GSERIAL_TYPE_PORT              (gserial_port_get_type ())
#define GSERIAL_PORT(obj)                  (G_TYPE_CHECK_INSTANCE_CAST ((obj), GSERIAL_TYPE_PORT, gserialPort))
#define GSERIAL_IS_PORT(obj)               (G_TYPE_CHECK_INSTANCE_TYPE ((obj), GSERIAL_TYPE_PORT))
#define GSERIAL_PORT_CLASS(klass)          (G_TYPE_CHECK_CLASS_CAST ((klass), GSERIAL_TYPE_PORT, gserialPortClass))
#define GSERIAL_IS_PORT_CLASS(klass)       (G_TYPE_CHECK_CLASS_TYPE ((klass), GSERIAL_TYPE_PORT))
#define GSERIAL_PORT_GET_CLASS(obj)        (G_TYPE_INSTANCE_GET_CLASS ((obj), GSERIAL_TYPE_PORT, gserialPortClass))

typedef struct _gserialPort gserialPort;
typedef struct _gserialPortClass gserialPortClass;
typedef struct _gserialPortPrivate gserialPortPrivate;


struct _gserialPort
{
    GObject parent_instance;
    gserialPortPrivate *priv;
};

/* class */
struct _gserialPortClass
{
    GObjectClass parent_class;
};


 /**
  * gserial_enumerate_ports:
  *
  * Return value: (element-type gserialPortInfo)(transfer full):
  */

GArray* gserial_enumerate_ports(void);

 /**
  * gserial_port_get_type:
  *
  * Return value: (element-type GType):
  */

GType gserial_port_get_type (void) G_GNUC_CONST;

 /**
  * gserial_port_new: (constructor):
  *
  */
gserialPort* gserial_port_new (void);

 /**
  * gserial_port_new_with_params: (constructor):
  *
  */
gserialPort* gserial_port_new_with_params(gserialByteSize sz, gserialParity p, int timeout, int baud);

 /**
  * gserial_port_set_byte_size:
  * @port: self
  * @sz: size of byte data
  */
void gserial_port_set_byte_size(gserialPort* port, gserialByteSize sz);

 /**
  * gserial_port_set_parity:
  * @port: self
  * @p: parity type. GSERIAL_PARITY_NONE is the most common.
  */
void gserial_port_set_parity(gserialPort* port, gserialParity p);

 /**
  * gserial_port_set_stopbits:
  * @port: self
  * @s: stop bits. Can be one, two or 1.5
  */
void gserial_port_set_stopbits(gserialPort* port, gserialStopBits s);

///TODO fix gserial_port_set_timeout. doesn't work at all now.
 /**
  * gserial_port_set_timeout:
  * @port: self
  * @timeout: length of read/write timeout in milliseconds
  */
void gserial_port_set_timeout(gserialPort* port, int timeout);

 /**
  * gserial_port_set_baud:
  * @port: self
  * @baud: coms port bit rate.
  */
void gserial_port_set_baud(gserialPort* port, int baud);

 /**
  * gserial_port_open:
  * @port: (in): a gserialPort
  * @path: (in): the port name
  * @error: location for error
  */
void gserial_port_open(gserialPort* port, const gchar* path, GError** error);

 /**
  * gserial_port_is_open:
  * @port: (in): a gserialPort
  * Returns: true if port is open
  */
gboolean gserial_port_is_open(gserialPort* port);

 /**
  * gserial_port_close:
  * @port: (in): a gserialPort
  */
void gserial_port_close(gserialPort* port);

 /**
  * gserial_port_available:
  * @port: (in): a gserialPort
  * Returns: number of bytes that can be read
  */
guint gserial_port_available(gserialPort* port, GError** error);

 /**
  * gserial_port_read_bytes:
  * @port: (in): a gserialPort
  * @len_in: (in): length to read
  * @garray: (out)(element-type gchar)(transfer full):
  */
void gserial_port_read_bytes(gserialPort* port, guint len_in, GArray** garray, GError** error);

 /**
  * gserial_port_read_bytes_until:
  * @port: (in): a gserialPort
  * @stop_char: stop reading when this character is reached.
  * @garray: (out)(element-type gchar)(transfer full):
  */
void gserial_port_read_bytes_until(gserialPort* port, gchar stop_char, GArray** garray, GError** error);

 /**
  * gserial_port_read_string:
  * @port: (in): a gserialPort
  * @len_in: length of string to read
  * @str: (out)(transfer full): string that has been read. null terminated.
  */
void gserial_port_read_string(gserialPort* port, guint len_in, gchar** str, GError** error);

 /**
  * gserial_port_write:
  * @port: (in): a gserialPort
  * @garray: (in)(element-type gchar): byte array containing data to write
  * Returns: number of bytes that were written
  */
gint gserial_port_write(gserialPort* port, GArray* garray, GError** error);


G_END_DECLS

#endif /* __GSERIAL_H__ */
