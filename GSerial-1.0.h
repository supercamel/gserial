#ifndef GSERIAL_H_INCLUDED
#define GSERIAL_H_INCLUDED

#include <glib-object.h>
#include <glib.h>

G_BEGIN_DECLS

typedef enum {
    GSERIAL_BYTE_SIZE_FIVEBIT,
    GSERIAL_BYTE_SIZE_SIXBIT,
    GSERIAL_BYTE_SIZE_SEVENBIT,
    GSERIAL_BYTE_SIZE_EIGHTBIT
} GSerialBYTE_SIZE;

#define GSERIAL_TYPE_BYTE_SIZE (gserial_byte_size_get_type())

typedef enum {
    GSERIAL_PARITY_NONE,
    GSERIAL_PARITY_ODD,
    GSERIAL_PARITY_EVEN
} GSerialPARITY;

#define GSERIAL_TYPE_PARITY (gserial_parity_get_type())

typedef enum {
    GSERIAL_STOP_BITS_ONE,
    GSERIAL_STOP_BITS_TWO,
    GSERIAL_STOP_BITS_ONEHALF
} GSerialSTOP_BITS;

#define GSERIAL_TYPE_STOP_BITS (gserial_stop_bits_get_type())

#define GSERIAL_TYPE_PORT (gserial_port_get_type())

G_DECLARE_FINAL_TYPE(GSerialPort, gserial_port, GSerial, Port, GObject)

GSerialPort* gserial_port_new();
GSerialPort* gserial_port_new_with_params(
        GSerialBYTE_SIZE byte_size, 
        GSerialPARITY parity,
        guint timeout,
        guint baud_rate
        );

gboolean gserial_port_open(GSerialPort* self, gchar* path);
void gserial_port_close(GSerialPort* self);
gboolean gserial_port_is_open(GSerialPort* self);
guint gserial_port_bytes_available(GSerialPort* self);

/**
 * gserial_port_read_bytes:
 * @self: the self
 * @len: maximum number of bytes to read
 * @result_length: number of bytes actually read
 * Returns: (transfer full) (array length=result_length):
 */
guint8* gserial_port_read_bytes(GSerialPort* self, guint len, gint* result_length);
gchar* gserial_port_read_string(GSerialPort* self, guint len);
gchar* gserial_port_read_line(GSerialPort* self);

/**
 * gserial_port_write_bytes:
 * @self: the self
 * @bytes: (array length=length): the bytes to write
 * @length: the number of bytes to write
 * Returns: the number of bytes written
 */
gint gserial_port_write_bytes(GSerialPort* self, guint8* bytes, guint length);
gint gserial_port_write_string(GSerialPort* self, const gchar* s);

guint gserial_port_get_baud(GSerialPort* self);
void gserial_port_set_baud(GSerialPort* self, guint baud);
GSerialBYTE_SIZE gserial_port_get_byte_size(GSerialPort* self);
void gserial_port_set_byte_size(GSerialPort* self, GSerialBYTE_SIZE byte_size);
GSerialPARITY gserial_port_get_parity(GSerialPort* self);
void gserial_port_set_parity(GSerialPort* self, GSerialPARITY parity);
GSerialSTOP_BITS gserial_port_get_stop_bits(GSerialPort* self);
void gserial_port_set_stop_bits(GSerialPort* self, GSerialSTOP_BITS stop_bits);
guint gserial_port_get_timeout(GSerialPort* self);
void gserial_port_set_timeout(GSerialPort* self, guint timeout);




G_END_DECLS


#endif


