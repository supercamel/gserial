#include "gserial-private.h"

struct _GSerialDevice {
    GObject parent_instance;
    gchar *path, *name, *serial;
    gint vid, pid, port;
    gboolean permission;
};
G_DEFINE_TYPE(GSerialDevice, gserial_device, G_TYPE_OBJECT)

static void gserial_device_finalize(GObject *object)
{
    GSerialDevice *self = GSERIAL_DEVICE(object);
    g_free(self->path); g_free(self->name); g_free(self->serial);
    G_OBJECT_CLASS(gserial_device_parent_class)->finalize(object);
}
static void gserial_device_class_init(GSerialDeviceClass *klass)
{
    G_OBJECT_CLASS(klass)->finalize = gserial_device_finalize;
}
static void gserial_device_init(GSerialDevice *self) { (void)self; }
GSerialDevice *gserial_device_new_info(const gchar *path, const gchar *name,
    gint vid, gint pid, const gchar *serial, gint port, gboolean permission)
{
    GSerialDevice *self = g_object_new(GSERIAL_TYPE_DEVICE, NULL);
    self->path = g_strdup(path); self->name = g_strdup(name ? name : path);
    self->serial = g_strdup(serial ? serial : "");
    self->vid = vid; self->pid = pid; self->port = port; self->permission = permission;
    return self;
}
const gchar *gserial_device_get_path(GSerialDevice *self) { return self->path; }
const gchar *gserial_device_get_name(GSerialDevice *self) { return self->name; }
const gchar *gserial_device_get_serial_number(GSerialDevice *self) { return self->serial; }
gint gserial_device_get_vendor_id(GSerialDevice *self) { return self->vid; }
gint gserial_device_get_product_id(GSerialDevice *self) { return self->pid; }
gint gserial_device_get_port_number(GSerialDevice *self) { return self->port; }
gboolean gserial_device_get_has_permission(GSerialDevice *self) { return self->permission; }

struct _GSerialDeviceManager {
    GObject parent_instance;
    GMainContext *context;
    GSource *source;
    gchar *pending_path;
#ifdef __ANDROID__
    GSerialAndroidObject *pending;
#endif
};
G_DEFINE_TYPE(GSerialDeviceManager, gserial_device_manager, G_TYPE_OBJECT)
static guint permission_signal;

void gserial_device_manager_cancel_permission(GSerialDeviceManager *self)
{
    g_return_if_fail(GSERIAL_IS_DEVICE_MANAGER(self));
    if (self->source) {
        g_source_destroy(self->source);
        g_source_unref(self->source);
        self->source = NULL;
    }
#ifdef __ANDROID__
    if (self->pending) gserial_android_object_close(self->pending);
    self->pending = NULL;
#endif
    g_clear_pointer(&self->pending_path, g_free);
}
static void manager_dispose(GObject *object)
{
    gserial_device_manager_cancel_permission(GSERIAL_DEVICE_MANAGER(object));
    G_OBJECT_CLASS(gserial_device_manager_parent_class)->dispose(object);
}
static void manager_finalize(GObject *object)
{
    g_main_context_unref(GSERIAL_DEVICE_MANAGER(object)->context);
    G_OBJECT_CLASS(gserial_device_manager_parent_class)->finalize(object);
}
static void gserial_device_manager_class_init(GSerialDeviceManagerClass *klass)
{
    G_OBJECT_CLASS(klass)->dispose = manager_dispose;
    G_OBJECT_CLASS(klass)->finalize = manager_finalize;
    /**
     * GSerialDeviceManager::permission-result:
     * @self: the manager
     * @path: the requested path
     * @granted: whether access was granted
     * @message: an error description, or an empty string on success
     *
     * Emitted on the manager's GLib context. Call lookup() to refresh USB
     * metadata and obtain a serial-number-based path before saving settings.
     */
    permission_signal = g_signal_new("permission-result", G_TYPE_FROM_CLASS(klass),
        G_SIGNAL_RUN_LAST, 0, NULL, NULL, NULL, G_TYPE_NONE, 3,
        G_TYPE_STRING, G_TYPE_BOOLEAN, G_TYPE_STRING);
}
static void gserial_device_manager_init(GSerialDeviceManager *self)
{
    self->context = g_main_context_ref_thread_default();
}
GSerialDeviceManager *gserial_device_manager_new(void)
{
    return g_object_new(GSERIAL_TYPE_DEVICE_MANAGER, NULL);
}
static gint compare_devices(gconstpointer a, gconstpointer b)
{
    return g_strcmp0(gserial_device_get_path((GSerialDevice *)a),
                    gserial_device_get_path((GSerialDevice *)b));
}
GList *gserial_device_manager_list_devices(GSerialDeviceManager *self, GError **error)
{
    g_return_val_if_fail(GSERIAL_IS_DEVICE_MANAGER(self), NULL);
#ifdef __ANDROID__
    GList *devices = gserial_android_list_devices(error);
#else
    GList *devices = gserial_platform_list_devices(error);
#endif
    return g_list_sort(devices, compare_devices);
}
GSerialDevice *gserial_device_manager_lookup(GSerialDeviceManager *self, const gchar *path, GError **error)
{
    g_return_val_if_fail(GSERIAL_IS_DEVICE_MANAGER(self), NULL);
    g_return_val_if_fail(path != NULL, NULL);
#ifdef __ANDROID__
    return gserial_android_lookup(path, error);
#else
    GList *devices = gserial_device_manager_list_devices(self, error);
    GSerialDevice *found = NULL;
    for (GList *item = devices; item; item = item->next) {
        if (g_str_equal(gserial_device_get_path(item->data), path)) {
            found = g_object_ref(item->data);
            break;
        }
    }
    g_list_free_full(devices, g_object_unref);
    if (!found && (!error || !*error))
        g_set_error(error, g_quark_from_static_string("gserial-device-error"), 1,
                    "Serial device is no longer available: %s", path);
    return found;
#endif
}
static gboolean permission_poll(gpointer data)
{
    GSerialDeviceManager *self = g_object_ref(data);
    GError *error = NULL;
    gint state = 1;
#ifdef __ANDROID__
    state = gserial_android_permission_state(self->pending, &error);
#endif
    if (state == 0) { g_object_unref(self); return G_SOURCE_CONTINUE; }
    gchar *path = g_strdup(self->pending_path);
    gserial_device_manager_cancel_permission(self);
    g_signal_emit(self, permission_signal, 0, path, state > 0,
        error ? error->message : state > 0 ? "" : "USB permission was denied or the device was detached");
    g_clear_error(&error);
    g_free(path);
    g_object_unref(self);
    return G_SOURCE_REMOVE;
}
gboolean gserial_device_manager_request_permission(GSerialDeviceManager *self, const gchar *path, GError **error)
{
    g_return_val_if_fail(GSERIAL_IS_DEVICE_MANAGER(self), FALSE);
    g_return_val_if_fail(path != NULL, FALSE);
    if (self->source) {
        g_set_error_literal(error, g_quark_from_static_string("gserial-device-error"), 2,
                            "A USB permission request is already pending");
        return FALSE;
    }
#ifdef __ANDROID__
    self->pending = gserial_android_request_permission(path, error);
    if (!self->pending) return FALSE;
#endif
    self->pending_path = g_strdup(path);
    self->source = g_timeout_source_new(25);
    g_source_set_callback(self->source, permission_poll, self, NULL);
    g_source_attach(self->source, self->context);
    return TRUE;
}
