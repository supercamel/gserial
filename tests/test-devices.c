#define _DEFAULT_SOURCE
#include "GSerial-1.0.h"
#include <glib/gstdio.h>
#include <unistd.h>

GList *gserial_linux_list_devices(const gchar *, const gchar *, GError **);

static void put(const gchar *root, const gchar *relative, const gchar *contents)
{
    gchar *path = g_build_filename(root, relative, NULL), *parent = g_path_get_dirname(path);
    g_assert_cmpint(g_mkdir_with_parents(parent, 0700), ==, 0);
    g_assert_true(g_file_set_contents(path, contents, -1, NULL));
    g_free(parent); g_free(path);
}
static void link_to(const gchar *root, const gchar *relative, const gchar *target)
{
    gchar *path = g_build_filename(root, relative, NULL), *parent = g_path_get_dirname(path);
    g_assert_cmpint(g_mkdir_with_parents(parent, 0700), ==, 0);
    g_assert_cmpint(symlink(target, path), ==, 0);
    g_free(parent); g_free(path);
}
static void remove_tree(const gchar *path)
{
    if (!g_file_test(path, G_FILE_TEST_IS_SYMLINK) && g_file_test(path, G_FILE_TEST_IS_DIR)) {
        GDir *dir = g_dir_open(path, 0, NULL); const gchar *name;
        while ((name = g_dir_read_name(dir))) {
            gchar *child = g_build_filename(path, name, NULL); remove_tree(child); g_free(child);
        }
        g_dir_close(dir); g_rmdir(path);
    } else g_remove(path);
}
static void test_enumeration(void)
{
    gchar *root = g_dir_make_tmp("gserial-devices-XXXXXX", NULL);
    g_assert_nonnull(root);
    put(root, "hardware/usb/idVendor", "0403\n");
    put(root, "hardware/usb/idProduct", "6001\n");
    put(root, "hardware/usb/serial", "RFD-A\n");
    put(root, "hardware/usb/product", "FTDI radio adapter\n");
    put(root, "hardware/usb/interface/bInterfaceNumber", "00\n");
    put(root, "dev/ttyUSB0", "");
    link_to(root, "sys/ttyUSB0/device", "../../hardware/usb/interface");
    link_to(root, "dev/serial/by-id/usb-FTDI_RFD-A", "../../ttyUSB0");
    /* Disconnected and virtual entries must not become selectable devices. */
    link_to(root, "sys/ttyUSB1/device", "../../hardware/usb/interface");
    put(root, "dev/tty0", ""); put(root, "sys/tty0/name", "virtual");
    gchar *sys = g_build_filename(root, "sys", NULL), *dev = g_build_filename(root, "dev", NULL);
    GError *error = NULL;
    GList *list = gserial_linux_list_devices(sys, dev, &error);
    g_assert_no_error(error); g_assert_cmpuint(g_list_length(list), ==, 1);
    GSerialDevice *device = list->data;
    gchar *expected = g_build_filename(dev, "serial/by-id/usb-FTDI_RFD-A", NULL);
    g_assert_cmpstr(gserial_device_get_path(device), ==, expected);
    g_assert_cmpstr(gserial_device_get_serial_number(device), ==, "RFD-A");
    g_assert_cmpint(gserial_device_get_vendor_id(device), ==, 0x0403);
    g_assert_cmpint(gserial_device_get_product_id(device), ==, 0x6001);
    g_assert_cmpint(gserial_device_get_port_number(device), ==, 0);
    g_assert_true(gserial_device_get_has_permission(device));
    g_assert_nonnull(strstr(gserial_device_get_name(device), "FTDI radio adapter"));
    g_list_free_full(list, g_object_unref);
    g_assert_cmpint(g_unlink(expected), ==, 0);
    list = gserial_linux_list_devices(sys, dev, &error); g_assert_no_error(error);
    gchar *node = g_build_filename(dev, "ttyUSB0", NULL);
    g_assert_cmpstr(gserial_device_get_path(list->data), ==, node);
    g_list_free_full(list, g_object_unref);
    g_free(node); g_free(expected); g_free(sys); g_free(dev); remove_tree(root); g_free(root);
}
typedef struct { GSerialDeviceManager *manager; guint calls; } Permission;
static void permission_result(GSerialDeviceManager *manager, const gchar *path,
                              gboolean granted, const gchar *message, Permission *state)
{
    g_assert_true(granted); g_assert_cmpstr(message, ==, "");
    state->calls++;
    if (state->calls == 1) {
        g_assert_cmpstr(path, ==, "/dev/first");
        g_assert_true(gserial_device_manager_request_permission(manager, "/dev/second", NULL));
    } else {
        g_assert_cmpstr(path, ==, "/dev/second");
        g_object_unref(manager); state->manager = NULL;
    }
}
static void test_permission_lifetime(void)
{
    GMainContext *context = g_main_context_new(); g_main_context_push_thread_default(context);
    Permission state = {gserial_device_manager_new(), 0};
    g_signal_connect(state.manager, "permission-result", G_CALLBACK(permission_result), &state);
    g_assert_true(gserial_device_manager_request_permission(state.manager, "/dev/cancelled", NULL));
    GError *error = NULL;
    g_assert_false(gserial_device_manager_request_permission(state.manager, "/dev/busy", &error));
    g_assert_nonnull(error); g_clear_error(&error);
    gserial_device_manager_cancel_permission(state.manager);
    g_assert_true(gserial_device_manager_request_permission(state.manager, "/dev/first", NULL));
    gint64 deadline = g_get_monotonic_time() + G_TIME_SPAN_SECOND;
    while (state.calls < 2 && g_get_monotonic_time() < deadline) {
        g_main_context_iteration(context, FALSE); g_usleep(1000);
    }
    g_assert_cmpuint(state.calls, ==, 2); g_assert_null(state.manager);
    g_main_context_pop_thread_default(context); g_main_context_unref(context);
}
static void test_open_error(void)
{
    GSerialPort *port = gserial_port_new(); GError *error = NULL;
    g_assert_false(gserial_port_open_full(port, "/gserial/nonexistent", &error));
    g_assert_nonnull(error); g_clear_error(&error);
    g_assert_false(gserial_port_set_control_lines(port, FALSE, FALSE, &error));
    g_assert_nonnull(error); g_clear_error(&error); g_object_unref(port);
}
int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/gserial/devices/linux-fixture", test_enumeration);
    g_test_add_func("/gserial/devices/permission-lifetime", test_permission_lifetime);
    g_test_add_func("/gserial/devices/open-errors", test_open_error);
    return g_test_run();
}
