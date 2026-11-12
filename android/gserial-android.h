#ifndef GSERIAL_ANDROID_H
#define GSERIAL_ANDROID_H
#include <glib.h>
#include <jni.h>
G_BEGIN_DECLS
/* Call once from an application's JNI entry point, before other GSerial calls.
 * Retains the application context/class loader for the lifetime of the process.
 * SQGI applications initialize automatically and do not need this function.
 * Package the installed share/sqgi-android/java sources in the application. */
gboolean gserial_android_initialize(JNIEnv *env, jobject context, GError **error);
G_END_DECLS
#endif
