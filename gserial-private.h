#ifndef GSERIAL_PRIVATE_H
#define GSERIAL_PRIVATE_H
#include "gserial-device.h"

GSerialDevice *gserial_device_new_info(const gchar *path, const gchar *name,
    gint vid, gint pid, const gchar *serial, gint port, gboolean permission);
GList *gserial_platform_list_devices(GError **error);

#ifdef __ANDROID__
typedef struct _GSerialAndroidObject GSerialAndroidObject;
GList *gserial_android_list_devices(GError **error);
GSerialDevice *gserial_android_lookup(const gchar *path, GError **error);
GSerialAndroidObject *gserial_android_request_permission(const gchar *path, GError **error);
gint gserial_android_permission_state(GSerialAndroidObject *request, GError **error);
void gserial_android_object_close(GSerialAndroidObject *object);
GSerialAndroidObject *gserial_android_open(const gchar *path, guint baud, guint bits,
    guint parity, guint stops, GError **error);
gint gserial_android_available(GSerialAndroidObject *session, GError **error);
guint8 *gserial_android_read(GSerialAndroidObject *session, guint length, guint timeout,
    gint *count, GError **error);
gint gserial_android_write(GSerialAndroidObject *session, const guint8 *bytes,
    guint length, guint timeout, GError **error);
gboolean gserial_android_set_control(GSerialAndroidObject *session, gboolean dtr,
    gboolean rts, GError **error);
#endif
#endif
