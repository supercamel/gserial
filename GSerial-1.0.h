#ifndef GSERIAL_H_INCLUDED
#define GSERIAL_H_INCLUDED

#include <glib-object.h>
#include <glib.h>

G_BEGIN_DECLS

/**
 * GSerialBYTE_SIZE:
 * @GSERIAL_BYTE_SIZE_FIVEBIT: 5 data bits
 * @GSERIAL_BYTE_SIZE_SIXBIT: 6 data bits
 * @GSERIAL_BYTE_SIZE_SEVENBIT: 7 data bits
 * @GSERIAL_BYTE_SIZE_EIGHTBIT: 8 data bits
 *
 * Enumeration for data bit length settings on a serial port.
 */
typedef enum {
    GSERIAL_BYTE_SIZE_FIVEBIT,
    GSERIAL_BYTE_SIZE_SIXBIT,
    GSERIAL_BYTE_SIZE_SEVENBIT,
    GSERIAL_BYTE_SIZE_EIGHTBIT
} GSerialBYTE_SIZE;

/**
 * gserial_byte_size_get_type:
 *
 * Returns: the #GType for #GSerialBYTE_SIZE
 */
#define GSERIAL_TYPE_BYTE_SIZE (gserial_byte_size_get_type())

/**
 * GSerialPARITY:
 * @GSERIAL_PARITY_NONE: No parity
 * @GSERIAL_PARITY_ODD: Odd parity
 * @GSERIAL_PARITY_EVEN: Even parity
 *
 * Enumeration for parity bit settings on a serial port.
 */
typedef enum {
    GSERIAL_PARITY_NONE,
    GSERIAL_PARITY_ODD,
    GSERIAL_PARITY_EVEN
} GSerialPARITY;

/**
 * gserial_parity_get_type:
 *
 * Returns: the #GType for #GSerialPARITY
 */
#define GSERIAL_TYPE_PARITY (gserial_parity_get_type())

/**
 * GSerialSTOP_BITS:
 * @GSERIAL_STOP_BITS_ONE: One stop bit
 * @GSERIAL_STOP_BITS_TWO: Two stop bits
 * @GSERIAL_STOP_BITS_ONEHALF: One and a half stop bits (usually for 5-bit)
 *
 * Enumeration for stop bit settings on a serial port.
 */
typedef enum {
    GSERIAL_STOP_BITS_ONE,
    GSERIAL_STOP_BITS_TWO,
    GSERIAL_STOP_BITS_ONEHALF
} GSerialSTOP_BITS;

/**
 * gserial_stop_bits_get_type:
 *
 * Returns: the #GType for #GSerialSTOP_BITS
 */
#define GSERIAL_TYPE_STOP_BITS (gserial_stop_bits_get_type())

#define GSERIAL_TYPE_PORT (gserial_port_get_type())

/**
 * GSerialPort:
 *
 * Represents a serial port instance.
 */
G_DECLARE_FINAL_TYPE(GSerialPort, gserial_port, GSerial, Port, GObject)

/**
 * gserial_port_new:
 *
 * Creates a new #GSerialPort instance with default settings.
 *
 * Returns: (transfer full): a new #GSerialPort instance
 */
GSerialPort* gserial_port_new(void);

/**
 * gserial_port_new_with_params:
 * @byte_size: the byte size to use
 * @parity: the parity setting
 * @timeout: read/write timeout in milliseconds
 * @baud_rate: the baud rate
 *
 * Creates a new #GSerialPort instance with custom parameters.
 *
 * Returns: (transfer full): a new #GSerialPort instance
 */
GSerialPort* gserial_port_new_with_params(
        GSerialBYTE_SIZE byte_size, 
        GSerialPARITY parity,
        guint timeout,
        guint baud_rate
        );

/**
 * gserial_port_open:
 * @self: a #GSerialPort
 * @path: the device path to open (e.g., "/dev/ttyUSB0")
 *
 * Opens the serial port.
 *
 * Returns: %TRUE if the port was opened successfully, %FALSE otherwise
 */
gboolean gserial_port_open(GSerialPort* self, gchar* path);

/**
 * gserial_port_close:
 * @self: a #GSerialPort
 *
 * Closes the serial port.
 */
void gserial_port_close(GSerialPort* self);

/**
 * gserial_port_is_open:
 * @self: a #GSerialPort
 *
 * Checks if the serial port is currently open.
 *
 * Returns: %TRUE if open, %FALSE otherwise
 */
gboolean gserial_port_is_open(GSerialPort* self);

/**
 * gserial_port_bytes_available:
 * @self: a #GSerialPort
 *
 * Gets the number of bytes available to read.
 *
 * Returns: number of bytes available
 */
guint gserial_port_bytes_available(GSerialPort* self);

/**
 * gserial_port_read_bytes:
 * @self: a #GSerialPort
 * @len: maximum number of bytes to read
 * @result_length: (out): actual number of bytes read
 *
 * Reads up to @len bytes from the serial port.
 *
 * Returns: (transfer full) (array length=result_length): a byte array of received data
 */
guint8* gserial_port_read_bytes(GSerialPort* self, guint len, gint* result_length);

/**
 * gserial_port_read_string:
 * @self: a #GSerialPort
 * @len: maximum string length to read
 *
 * Reads a string from the serial port.
 *
 * Returns: (transfer full): a newly allocated string
 */
gchar* gserial_port_read_string(GSerialPort* self, guint len);

/**
 * gserial_port_read_line:
 * @self: a #GSerialPort
 *
 * Reads a line of text terminated by a newline character.
 *
 * Returns: (transfer full): a newly allocated line
 */
gchar* gserial_port_read_line(GSerialPort* self);

/**
 * gserial_port_write_bytes:
 * @self: a #GSerialPort
 * @bytes: (array length=length): the bytes to write
 * @length: number of bytes to write
 *
 * Writes raw bytes to the serial port.
 *
 * Returns: number of bytes actually written
 */
gint gserial_port_write_bytes(GSerialPort* self, guint8* bytes, guint length);

/**
 * gserial_port_write_string:
 * @self: a #GSerialPort
 * @s: string to write
 *
 * Writes a string to the serial port.
 *
 * Returns: number of bytes actually written
 */
gint gserial_port_write_string(GSerialPort* self, const gchar* s);

/**
 * gserial_port_get_baud:
 * @self: a #GSerialPort
 *
 * Gets the baud rate.
 *
 * Returns: the baud rate
 */
guint gserial_port_get_baud(GSerialPort* self);

/**
 * gserial_port_set_baud:
 * @self: a #GSerialPort
 * @baud: baud rate to set
 *
 * Sets the baud rate.
 */
void gserial_port_set_baud(GSerialPort* self, guint baud);

/**
 * gserial_port_get_byte_size:
 * @self: a #GSerialPort
 *
 * Gets the byte size.
 *
 * Returns: a #GSerialBYTE_SIZE
 */
GSerialBYTE_SIZE gserial_port_get_byte_size(GSerialPort* self);

/**
 * gserial_port_set_byte_size:
 * @self: a #GSerialPort
 * @byte_size: new byte size
 *
 * Sets the byte size.
 */
void gserial_port_set_byte_size(GSerialPort* self, GSerialBYTE_SIZE byte_size);

/**
 * gserial_port_get_parity:
 * @self: a #GSerialPort
 *
 * Gets the parity setting.
 *
 * Returns: a #GSerialPARITY
 */
GSerialPARITY gserial_port_get_parity(GSerialPort* self);

/**
 * gserial_port_set_parity:
 * @self: a #GSerialPort
 * @parity: new parity setting
 *
 * Sets the parity.
 */
void gserial_port_set_parity(GSerialPort* self, GSerialPARITY parity);

/**
 * gserial_port_get_stop_bits:
 * @self: a #GSerialPort
 *
 * Gets the stop bit setting.
 *
 * Returns: a #GSerialSTOP_BITS
 */
GSerialSTOP_BITS gserial_port_get_stop_bits(GSerialPort* self);

/**
 * gserial_port_set_stop_bits:
 * @self: a #GSerialPort
 * @stop_bits: new stop bit setting
 *
 * Sets the stop bits.
 */
void gserial_port_set_stop_bits(GSerialPort* self, GSerialSTOP_BITS stop_bits);

/**
 * gserial_port_get_timeout:
 * @self: a #GSerialPort
 *
 * Gets the current timeout value in milliseconds.
 *
 * Returns: the timeout
 */
guint gserial_port_get_timeout(GSerialPort* self);

/**
 * gserial_port_set_timeout:
 * @self: a #GSerialPort
 * @timeout: timeout in milliseconds
 *
 * Sets the timeout value for reads and writes.
 */
void gserial_port_set_timeout(GSerialPort* self, guint timeout);

G_END_DECLS

#endif


