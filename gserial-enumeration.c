#include "gserial-private.h"
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#include <setupapi.h>
#include <cfgmgr32.h>

static gchar *utf8(const WCHAR *value)
{
    return g_utf16_to_utf8((const gunichar2 *)value, -1, NULL, NULL, NULL);
}
static gchar *usb_serial(DEVINST node)
{
    /* FTDI's child instance ID can append a channel letter. Read the USB
     * ancestor instead so a channel suffix is not reported as the serial. */
    for (guint depth = 0; depth < 16; ++depth) {
        WCHAR value[MAX_DEVICE_ID_LEN] = {0};
        if (CM_Get_Device_IDW(node, value, G_N_ELEMENTS(value), 0) == CR_SUCCESS) {
            gchar *id = utf8(value);
            if (id && g_str_has_prefix(id, "USB\\")) {
                gchar *last = strrchr(id, '\\');
                if (last && last[1] && !strchr(last + 1, '&')) {
                    gchar *result = g_strdup(last + 1); g_free(id); return result;
                }
            }
            g_free(id);
        }
        DEVINST parent;
        if (CM_Get_Parent(&parent, node, 0) != CR_SUCCESS) break;
        node = parent;
    }
    return NULL;
}

GList *gserial_platform_list_devices(GError **error)
{
    static const GUID ports_class = {0x4d36e978, 0xe325, 0x11ce,
                                    {0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18}};
    HDEVINFO info = SetupDiGetClassDevsW(&ports_class, NULL, NULL, DIGCF_PRESENT);
    if (info == INVALID_HANDLE_VALUE) {
        g_set_error(error, g_quark_from_static_string("gserial-device-error"), 1,
                    "Cannot enumerate Windows serial ports (error %lu)", GetLastError());
        return NULL;
    }
    GList *devices = NULL;
    SP_DEVINFO_DATA device = {0};
    device.cbSize = sizeof(device);
    for (DWORD index = 0; SetupDiEnumDeviceInfo(info, index, &device); ++index) {
        HKEY key = SetupDiOpenDevRegKey(info, &device, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (key == INVALID_HANDLE_VALUE) continue;
        WCHAR port[256] = {0}, friendly[1024] = {0}, hardware[2048] = {0};
        DWORD bytes = sizeof(port) - sizeof(WCHAR), type = 0;
        LONG status = RegQueryValueExW(key, L"PortName", NULL, &type, (BYTE *)port, &bytes);
        RegCloseKey(key);
        if (status != ERROR_SUCCESS || type != REG_SZ || wcsncmp(port, L"COM", 3)) continue;
        SetupDiGetDeviceRegistryPropertyW(info, &device, SPDRP_FRIENDLYNAME, NULL,
            (BYTE *)friendly, sizeof(friendly) - sizeof(WCHAR), NULL);
        SetupDiGetDeviceRegistryPropertyW(info, &device, SPDRP_HARDWAREID, NULL,
            (BYTE *)hardware, sizeof(hardware) - sizeof(WCHAR), NULL);
        gchar *path = utf8(port), *name = utf8(friendly), *id = utf8(hardware);
        gchar *vendor = id ? strstr(id, "VID_") : NULL, *product = id ? strstr(id, "PID_") : NULL;
        gint vid = vendor ? (gint)g_ascii_strtoull(vendor + 4, NULL, 16) & 0xffff : -1;
        gint pid = product ? (gint)g_ascii_strtoull(product + 4, NULL, 16) & 0xffff : -1;
        gchar *serial = usb_serial(device.DevInst);
        if (path) devices = g_list_prepend(devices, gserial_device_new_info(path,
            name && *name ? name : path, vid, pid, serial, -1, TRUE));
        g_free(path); g_free(name); g_free(id); g_free(serial);
    }
    SetupDiDestroyDeviceInfoList(info);
    return devices;
}

#elif !defined(__ANDROID__)
#include <stdlib.h>

static gchar *attribute(const gchar *directory, const gchar *name)
{
    gchar *path = g_build_filename(directory, name, NULL), *text = NULL;
    if (g_file_get_contents(path, &text, NULL, NULL)) g_strstrip(text);
    g_free(path);
    return text;
}
static gchar *stable_path(const gchar *devroot, const gchar *node)
{
    gchar *directory = g_build_filename(devroot, "serial", "by-id", NULL);
    GDir *dir = g_dir_open(directory, 0, NULL);
    gchar *result = NULL;
    if (dir) {
        const gchar *name;
        while ((name = g_dir_read_name(dir))) {
            gchar *path = g_build_filename(directory, name, NULL);
            char *resolved = realpath(path, NULL);
            if (resolved && g_str_equal(resolved, node) && (!result || strcmp(path, result) < 0)) {
                g_free(result); result = g_strdup(path);
            }
            free(resolved); g_free(path);
        }
        g_dir_close(dir);
    }
    g_free(directory);
    return result ? result : g_strdup(node);
}

/* Separate roots let regression tests use a private sysfs/device fixture. */
GList *gserial_linux_list_devices(const gchar *sysroot, const gchar *devroot, GError **error)
{
    GDir *dir = g_dir_open(sysroot, 0, error);
    if (!dir) return NULL;
    GList *devices = NULL;
    const gchar *name;
    while ((name = g_dir_read_name(dir))) {
        gchar *node = g_build_filename(devroot, name, NULL);
        gchar *link = g_build_filename(sysroot, name, "device", NULL);
        char *resolved = realpath(link, NULL);
        g_free(link);
        if (!resolved || !g_file_test(node, G_FILE_TEST_EXISTS)) {
            free(resolved); g_free(node); continue;
        }
        gchar *current = g_strdup(resolved), *serial = NULL, *product = NULL;
        gint vid = -1, pid = -1, port = -1;
        free(resolved);
        while (strcmp(current, "/") != 0) {
            gchar *v = attribute(current, "idVendor"), *p = attribute(current, "idProduct");
            gchar *number = attribute(current, "bInterfaceNumber");
            if (number && port < 0) port = (gint)g_ascii_strtoull(number, NULL, 16);
            g_free(number);
            if (v && p) {
                vid = (gint)g_ascii_strtoull(v, NULL, 16);
                pid = (gint)g_ascii_strtoull(p, NULL, 16);
                serial = attribute(current, "serial"); product = attribute(current, "product");
            }
            g_free(v); g_free(p);
            if (vid >= 0) break;
            gchar *parent = g_path_get_dirname(current);
            g_free(current); current = parent;
        }
        gchar *path = stable_path(devroot, node);
        gchar *label = product ? g_strdup_printf("%s%s%s (%s)", product,
            serial && *serial ? " · " : "", serial ? serial : "", name) : g_strdup(name);
        devices = g_list_prepend(devices, gserial_device_new_info(path, label, vid, pid, serial, port, TRUE));
        g_free(current); g_free(serial); g_free(product); g_free(path); g_free(label); g_free(node);
    }
    g_dir_close(dir);
    return devices;
}
GList *gserial_platform_list_devices(GError **error)
{
    return gserial_linux_list_devices("/sys/class/tty", "/dev", error);
}
#endif
