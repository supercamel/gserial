#include "gserial-private.h"
#include "gserial-android.h"
#include <dlfcn.h>

struct _GSerialAndroidObject { JavaVM *vm; jobject value; };
typedef struct {
    JavaVM *vm;
    JNIEnv *env;
    jobject context;
    jclass bridge;
    gboolean attached, frame;
} Bridge;
static GMutex runtime_lock;
static JavaVM *runtime_vm;
static jobject runtime_context, runtime_loader;

static void fail(GError **error, const gchar *message)
{
    g_set_error_literal(error, g_quark_from_static_string("gserial-android-error"), 1, message);
}
static gchar *from_java(JNIEnv *env, jstring value)
{
    if (!value) return NULL;
    const jchar *text = (*env)->GetStringChars(env, value, NULL);
    if (!text) return NULL;
    gchar *result = g_utf16_to_utf8((const gunichar2 *)text,
        (*env)->GetStringLength(env, value), NULL, NULL, NULL);
    (*env)->ReleaseStringChars(env, value, text);
    return result;
}
static jstring to_java(JNIEnv *env, const gchar *value)
{
    glong length;
    gunichar2 *text = g_utf8_to_utf16(value, -1, NULL, &length, NULL);
    if (!text) return NULL;
    jstring result = (*env)->NewString(env, (jchar *)text, (jsize)length);
    g_free(text);
    return result;
}
static gboolean exception(Bridge *b, GError **error)
{
    JNIEnv *env = b->env;
    jthrowable thrown = (*env)->ExceptionOccurred(env);
    if (!thrown) return FALSE;
    (*env)->ExceptionClear(env);
    jclass type = (*env)->GetObjectClass(env, thrown);
    jmethodID method = type ? (*env)->GetMethodID(env, type, "toString", "()Ljava/lang/String;") : NULL;
    jstring value = method ? (*env)->CallObjectMethod(env, thrown, method) : NULL;
    gchar *message = !(*env)->ExceptionCheck(env) && value ? from_java(env, value) : NULL;
    (*env)->ExceptionClear(env);
    fail(error, message ? message : "Android USB operation failed");
    g_free(message);
    return TRUE;
}
gboolean gserial_android_initialize(JNIEnv *env, jobject context, GError **error)
{
    g_return_val_if_fail(env != NULL && context != NULL, FALSE);
    g_mutex_lock(&runtime_lock);
    if (runtime_vm) { g_mutex_unlock(&runtime_lock); return TRUE; }
    Bridge b = {.env = env};
    if ((*env)->PushLocalFrame(env, 16) != JNI_OK) {
        (*env)->ExceptionClear(env); fail(error, "Cannot allocate an Android JNI frame");
        g_mutex_unlock(&runtime_lock); return FALSE;
    }
    jclass type = (*env)->GetObjectClass(env, context);
    jmethodID app_method = type ? (*env)->GetMethodID(env, type, "getApplicationContext", "()Landroid/content/Context;") : NULL;
    jobject app = app_method ? (*env)->CallObjectMethod(env, context, app_method) : NULL;
    gboolean ok = !exception(&b, error) && app != NULL;
    if (ok) {
        type = (*env)->GetObjectClass(env, app);
        jmethodID load = type ? (*env)->GetMethodID(env, type, "getClassLoader", "()Ljava/lang/ClassLoader;") : NULL;
        jobject loader = load ? (*env)->CallObjectMethod(env, app, load) : NULL;
        ok = !exception(&b, error) && loader != NULL;
        if (ok) {
            runtime_context = (*env)->NewGlobalRef(env, app);
            if (!exception(&b, error) && runtime_context) {
                runtime_loader = (*env)->NewGlobalRef(env, loader);
                ok = !exception(&b, error) && runtime_loader != NULL;
            } else ok = FALSE;
        }
    }
    if (ok) ok = (*env)->GetJavaVM(env, &runtime_vm) == JNI_OK;
    if (!ok) {
        if (runtime_context) (*env)->DeleteGlobalRef(env, runtime_context);
        if (runtime_loader) (*env)->DeleteGlobalRef(env, runtime_loader);
        runtime_vm = NULL; runtime_context = runtime_loader = NULL;
        if (!error || !*error) fail(error, "Cannot retain the Android application context");
    }
    (*env)->PopLocalFrame(env, NULL);
    g_mutex_unlock(&runtime_lock);
    return ok;
}
static void end(Bridge *b)
{
    if (b->frame) (*b->env)->PopLocalFrame(b->env, NULL);
    if (b->attached) (*b->vm)->DetachCurrentThread(b->vm);
}
static gboolean begin(Bridge *b, GSerialAndroidObject *object, GError **error)
{
    jobject loader = NULL;
    if (object) b->vm = object->vm;
    else {
        g_mutex_lock(&runtime_lock);
        b->vm = runtime_vm; b->context = runtime_context; loader = runtime_loader;
        g_mutex_unlock(&runtime_lock);
    }
    if (!b->vm) {
        void *platform = dlopen("libsqgi-platform.so", RTLD_NOW | RTLD_LOCAL);
        JavaVM *(*vm)(void) = platform ? dlsym(platform, "sqgi_android_get_java_vm") : NULL;
        jobject (*context)(void) = platform ? dlsym(platform, "sqgi_android_get_application_context") : NULL;
        /* Older SQGI platform builds expose the same application context via
         * their GStreamer integration hook. */
        if (!context && platform) context = dlsym(platform, "gst_android_get_application_context");
        jobject (*class_loader)(void) = platform ? dlsym(platform, "sqgi_android_get_application_class_loader") : NULL;
        b->vm = vm ? vm() : NULL;
        b->context = context ? context() : NULL;
        loader = class_loader ? class_loader() : NULL;
        if (platform) dlclose(platform);
        if (!b->vm || !b->context || !loader) {
            fail(error, "SQGI Android application services are unavailable");
            return FALSE;
        }
    }
    if ((*b->vm)->GetEnv(b->vm, (void **)&b->env, JNI_VERSION_1_6) != JNI_OK) {
        if ((*b->vm)->AttachCurrentThread(b->vm, &b->env, NULL) != JNI_OK) {
            fail(error, "Cannot attach the Android USB thread"); return FALSE;
        }
        b->attached = TRUE;
    }
    if ((*b->env)->PushLocalFrame(b->env, 32) != JNI_OK) {
        (*b->env)->ExceptionClear(b->env);
        fail(error, "Cannot allocate an Android USB JNI frame"); end(b); return FALSE;
    }
    b->frame = TRUE;
    if (!object) {
        JNIEnv *env = b->env;
        jclass type = (*env)->GetObjectClass(env, loader);
        jmethodID load = type ? (*env)->GetMethodID(env, type, "loadClass", "(Ljava/lang/String;)Ljava/lang/Class;") : NULL;
        jstring name = (*env)->NewStringUTF(env, "org.gserial.UsbSerialBridge");
        if (load && name) b->bridge = (*env)->CallObjectMethod(env, loader, load, name);
        if (exception(b, error)) { end(b); return FALSE; }
        if (!b->bridge) { fail(error, "GSerial Android Java sources are missing from the APK"); end(b); return FALSE; }
    }
    return TRUE;
}
static GSerialAndroidObject *retain(Bridge *b, jobject value, GError **error)
{
    if (exception(b, error)) return NULL;
    if (!value) { fail(error, "Android USB operation returned no object"); return NULL; }
    jobject global = (*b->env)->NewGlobalRef(b->env, value);
    if (exception(b, error)) return NULL;
    if (!global) { fail(error, "Cannot retain the Android USB object"); return NULL; }
    GSerialAndroidObject *object = g_new0(GSerialAndroidObject, 1);
    object->vm = b->vm; object->value = global;
    return object;
}
static GList *devices(Bridge *b, jobjectArray values, GError **error)
{
    if (exception(b, error)) return NULL;
    if (!values) { fail(error, "Invalid Android USB device list"); return NULL; }
    JNIEnv *env = b->env;
    jsize length = (*env)->GetArrayLength(env, values);
    if (length % 8) { fail(error, "Invalid Android USB device metadata"); return NULL; }
    GList *result = NULL;
    for (jsize i = 0; i < length; i += 8) {
        gchar *row[8] = {0};
        gboolean valid = TRUE;
        for (gint j = 0; j < 8; ++j) {
            jstring value = (*env)->GetObjectArrayElement(env, values, i + j);
            if (!(*env)->ExceptionCheck(env) && value) row[j] = from_java(env, value);
            if (value) (*env)->DeleteLocalRef(env, value);
            if (!row[j] || (*env)->ExceptionCheck(env)) { valid = FALSE; break; }
        }
        if (valid) result = g_list_append(result, gserial_device_new_info(row[0], row[1],
            (gint)g_ascii_strtoll(row[2], NULL, 10), (gint)g_ascii_strtoll(row[3], NULL, 10),
            row[4], (gint)g_ascii_strtoll(row[5], NULL, 10), g_str_equal(row[6], "true")));
        for (gint j = 0; j < 8; ++j) g_free(row[j]);
        if (!valid) {
            if (!exception(b, error)) fail(error, "Invalid Android USB device string");
            g_list_free_full(result, g_object_unref); return NULL;
        }
    }
    return result;
}
GList *gserial_android_list_devices(GError **error)
{
    Bridge b = {0};
    if (!begin(&b, NULL, error)) return NULL;
    jmethodID method = (*b.env)->GetStaticMethodID(b.env, b.bridge, "listDevices", "(Landroid/content/Context;)[Ljava/lang/String;");
    jobjectArray value = method ? (*b.env)->CallStaticObjectMethod(b.env, b.bridge, method, b.context) : NULL;
    GList *result = devices(&b, value, error);
    end(&b); return result;
}
GSerialDevice *gserial_android_lookup(const gchar *path, GError **error)
{
    Bridge b = {0};
    if (!begin(&b, NULL, error)) return NULL;
    jmethodID method = (*b.env)->GetStaticMethodID(b.env, b.bridge, "lookup", "(Landroid/content/Context;Ljava/lang/String;)[Ljava/lang/String;");
    jstring name = to_java(b.env, path);
    jobjectArray value = method && name ? (*b.env)->CallStaticObjectMethod(b.env, b.bridge, method, b.context, name) : NULL;
    GList *list = devices(&b, value, error);
    GSerialDevice *result = list ? g_object_ref(list->data) : NULL;
    g_list_free_full(list, g_object_unref);
    end(&b); return result;
}
GSerialAndroidObject *gserial_android_request_permission(const gchar *path, GError **error)
{
    Bridge b = {0};
    if (!begin(&b, NULL, error)) return NULL;
    jmethodID method = (*b.env)->GetStaticMethodID(b.env, b.bridge, "requestPermission",
        "(Landroid/content/Context;Ljava/lang/String;)Lorg/gserial/UsbSerialBridge$Permission;");
    jstring name = to_java(b.env, path);
    jobject value = method && name ? (*b.env)->CallStaticObjectMethod(b.env, b.bridge, method, b.context, name) : NULL;
    GSerialAndroidObject *result = retain(&b, value, error);
    end(&b); return result;
}
static gint integer_method(GSerialAndroidObject *object, const char *name, GError **error)
{
    Bridge b = {0};
    if (!begin(&b, object, error)) return -1;
    jclass type = (*b.env)->GetObjectClass(b.env, object->value);
    jmethodID method = type ? (*b.env)->GetMethodID(b.env, type, name, "()I") : NULL;
    gint value = method ? (*b.env)->CallIntMethod(b.env, object->value, method) : -1;
    if (exception(&b, error)) value = -1;
    end(&b); return value;
}
gint gserial_android_permission_state(GSerialAndroidObject *object, GError **error)
{
    return integer_method(object, "state", error);
}
gint gserial_android_available(GSerialAndroidObject *object, GError **error)
{
    return integer_method(object, "available", error);
}
void gserial_android_object_close(GSerialAndroidObject *object)
{
    if (!object) return;
    Bridge b = {0};
    if (begin(&b, object, NULL)) {
        jclass type = (*b.env)->GetObjectClass(b.env, object->value);
        jmethodID method = type ? (*b.env)->GetMethodID(b.env, type, "close", "()V") : NULL;
        if (method) (*b.env)->CallVoidMethod(b.env, object->value, method);
        exception(&b, NULL);
        (*b.env)->DeleteGlobalRef(b.env, object->value);
        end(&b);
    }
    g_free(object);
}
GSerialAndroidObject *gserial_android_open(const gchar *path, guint baud, guint bits,
    guint parity, guint stops, GError **error)
{
    Bridge b = {0};
    if (!begin(&b, NULL, error)) return NULL;
    jmethodID method = (*b.env)->GetStaticMethodID(b.env, b.bridge, "open",
        "(Landroid/content/Context;Ljava/lang/String;IIII)Lorg/gserial/UsbSerialBridge$Session;");
    jstring name = to_java(b.env, path);
    jobject value = method && name ? (*b.env)->CallStaticObjectMethod(b.env, b.bridge, method,
        b.context, name, (jint)baud, (jint)bits, (jint)parity, (jint)stops) : NULL;
    GSerialAndroidObject *result = retain(&b, value, error);
    end(&b); return result;
}
guint8 *gserial_android_read(GSerialAndroidObject *object, guint length, guint timeout,
    gint *count, GError **error)
{
    *count = 0;
    Bridge b = {0};
    if (!begin(&b, object, error)) return NULL;
    jclass type = (*b.env)->GetObjectClass(b.env, object->value);
    jmethodID method = type ? (*b.env)->GetMethodID(b.env, type, "read", "(II)[B") : NULL;
    jbyteArray value = method ? (*b.env)->CallObjectMethod(b.env, object->value, method,
        (jint)MIN(length, G_MAXINT), (jint)MIN(timeout, G_MAXINT)) : NULL;
    guint8 *result = NULL;
    if (!exception(&b, error) && value) {
        jsize size = (*b.env)->GetArrayLength(b.env, value);
        result = g_malloc((gsize)size + 1);
        (*b.env)->GetByteArrayRegion(b.env, value, 0, size, (jbyte *)result);
        if (exception(&b, error)) g_clear_pointer(&result, g_free);
        else { *count = size; result[size] = 0; }
    }
    end(&b); return result;
}
gint gserial_android_write(GSerialAndroidObject *object, const guint8 *bytes,
    guint length, guint timeout, GError **error)
{
    Bridge b = {0};
    if (!begin(&b, object, error)) return -1;
    /* Bound each JNI copy; callers already support partial write counts. */
    length = MIN(length, 65536u);
    jclass type = (*b.env)->GetObjectClass(b.env, object->value);
    jmethodID method = type ? (*b.env)->GetMethodID(b.env, type, "write", "([BI)I") : NULL;
    jbyteArray buffer = (*b.env)->NewByteArray(b.env, (jsize)length);
    if (buffer && length) (*b.env)->SetByteArrayRegion(b.env, buffer, 0, (jsize)length, (const jbyte *)bytes);
    gint result = -1;
    if (!exception(&b, error)) {
        if (method && buffer) {
            result = (*b.env)->CallIntMethod(b.env, object->value, method, buffer, (jint)MIN(timeout, G_MAXINT));
            if (exception(&b, error)) result = -1;
        } else fail(error, "Cannot allocate an Android USB write buffer");
    }
    end(&b); return result;
}
gboolean gserial_android_set_control(GSerialAndroidObject *object, gboolean dtr, gboolean rts, GError **error)
{
    Bridge b = {0};
    if (!begin(&b, object, error)) return FALSE;
    jclass type = (*b.env)->GetObjectClass(b.env, object->value);
    jmethodID method = type ? (*b.env)->GetMethodID(b.env, type, "setControl", "(ZZ)V") : NULL;
    if (method) (*b.env)->CallVoidMethod(b.env, object->value, method, (jboolean)dtr, (jboolean)rts);
    gboolean ok = !exception(&b, error) && method != NULL;
    end(&b); return ok;
}
