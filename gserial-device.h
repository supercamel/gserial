#ifndef GSERIAL_DEVICE_H
#define GSERIAL_DEVICE_H

#include <glib-object.h>

G_BEGIN_DECLS

#define GSERIAL_TYPE_DEVICE (gserial_device_get_type())
/**
 * GSerialDevice:
 *
 * An immutable snapshot of a serial port. Enumerating never opens the port.
 * USB metadata describes the adapter, not the radio or MCU attached to its UART.
 */
G_DECLARE_FINAL_TYPE(GSerialDevice, gserial_device, GSERIAL, DEVICE, GObject)

/**
 * gserial_device_get_path:
 * @self: a #GSerialDevice
 * Returns: (transfer none): the path or USB URI accepted by gserial_port_open()
 */
const gchar *gserial_device_get_path(GSerialDevice *self);
/**
 * gserial_device_get_name:
 * @self: a #GSerialDevice
 * Returns: (transfer none): a display name
 */
const gchar *gserial_device_get_name(GSerialDevice *self);
/**
 * gserial_device_get_serial_number:
 * @self: a #GSerialDevice
 * Returns: (transfer none): the USB serial number, or an empty string if unavailable
 */
const gchar *gserial_device_get_serial_number(GSerialDevice *self);
/**
 * gserial_device_get_vendor_id:
 * @self: a #GSerialDevice
 * Returns: the USB vendor ID, or -1 if unavailable
 */
gint gserial_device_get_vendor_id(GSerialDevice *self);
/**
 * gserial_device_get_product_id:
 * @self: a #GSerialDevice
 * Returns: the USB product ID, or -1 if unavailable
 */
gint gserial_device_get_product_id(GSerialDevice *self);
/**
 * gserial_device_get_port_number:
 * @self: a #GSerialDevice
 * Returns: the port/interface number on a USB adapter, or -1 if unavailable
 */
gint gserial_device_get_port_number(GSerialDevice *self);
/**
 * gserial_device_get_has_permission:
 * @self: a #GSerialDevice
 * Returns: whether Android USB permission was granted when this snapshot was made
 *
 * Always true on desktop; opening can still fail due to OS permissions or a busy port.
 */
gboolean gserial_device_get_has_permission(GSerialDevice *self);

#define GSERIAL_TYPE_DEVICE_MANAGER (gserial_device_manager_get_type())
/**
 * GSerialDeviceManager:
 *
 * Serial device discovery and asynchronous Android USB permissions. Use from
 * its creating thread, whose thread-default GLib context delivers permission
 * results. Keep the manager alive until a request finishes, or cancel it.
 */
G_DECLARE_FINAL_TYPE(GSerialDeviceManager, gserial_device_manager, GSERIAL, DEVICE_MANAGER, GObject)
/**
 * gserial_device_manager_new:
 * Returns: (transfer full): a new manager
 */
GSerialDeviceManager *gserial_device_manager_new(void);
/**
 * gserial_device_manager_list_devices:
 * @self: a #GSerialDeviceManager
 * @error: return location for an error
 *
 * Refreshes the device list. Android lists supported USB serial adapters;
 * Linux and Windows list installed serial ports. No permission prompts occur.
 * Android USB serial numbers become available only after permission is granted.
 *
 * Returns: (transfer full) (element-type GSerialDevice): device snapshots
 */
GList *gserial_device_manager_list_devices(GSerialDeviceManager *self, GError **error);
/**
 * gserial_device_manager_lookup:
 * @self: a #GSerialDeviceManager
 * @path: a device path or USB URI
 * @error: return location for an error
 *
 * Refreshes one device. After Android permission is granted, its path uses
 * the USB serial number if unique, allowing reconnect after re-enumeration.
 * A missing or ambiguous device is an error; it is never replaced by another.
 *
 * Returns: (transfer full) (nullable): a device snapshot
 */
GSerialDevice *gserial_device_manager_lookup(GSerialDeviceManager *self, const gchar *path, GError **error);
/**
 * gserial_device_manager_request_permission:
 * @self: a #GSerialDeviceManager
 * @path: the path returned by enumeration
 * @error: return location for an error
 *
 * Starts one permission request and emits permission-result when it completes.
 * Already-authorized and desktop requests also complete asynchronously. A
 * second pending request is rejected. Permission does not open the port.
 *
 * Returns: whether the request was started
 */
gboolean gserial_device_manager_request_permission(GSerialDeviceManager *self, const gchar *path, GError **error);
/**
 * gserial_device_manager_cancel_permission:
 * @self: a #GSerialDeviceManager
 *
 * Cancels observation of the pending request without emitting a result. Android
 * may still display its system dialog; cancellation does not revoke permission.
 */
void gserial_device_manager_cancel_permission(GSerialDeviceManager *self);

G_END_DECLS
#endif
