#define _DEFAULT_SOURCE
#include "GSerial-1.0.h"
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

typedef struct {
    int master;
    int slave;
    char path[128];
    GSerialPort *port;
    guint source_id;
    guint callbacks;
} Fixture;

static void fixture_init(Fixture *f, gboolean nonblocking, guint timeout)
{
    memset(f, 0, sizeof(*f));
    g_assert_cmpint(openpty(&f->master, &f->slave, f->path, NULL, NULL), ==, 0);
    f->port = gserial_port_new();
    g_object_add_weak_pointer(G_OBJECT(f->port), (gpointer *)&f->port);
    gserial_port_set_timeout(f->port, timeout);
    gserial_port_set_nonblocking(f->port, nonblocking);
    g_assert_true(gserial_port_open(f->port, f->path));
}

static void fixture_clear(Fixture *f)
{
    if (f->port) {
        gserial_port_close(f->port);
        g_object_unref(f->port);
    }
    close(f->master);
    close(f->slave);
}

static void wait_for_callbacks(Fixture *f, guint count)
{
    gint64 deadline = g_get_monotonic_time() + G_TIME_SPAN_SECOND;
    while (f->callbacks < count && g_get_monotonic_time() < deadline) {
        g_main_context_iteration(NULL, FALSE);
        g_usleep(1000);
    }
    g_assert_cmpuint(f->callbacks, ==, count);
}

static void release_on_data(GSerialPort *port, gint available, Fixture *f)
{
    g_assert_cmpint(available, >, 0);
    f->source_id = g_source_get_id(g_main_current_source());
    f->callbacks++;
    g_object_unref(port);
}

static void test_callback_release(void)
{
    Fixture f;
    fixture_init(&f, FALSE, 100);
    g_signal_connect(f.port, "on_data", G_CALLBACK(release_on_data), &f);
    g_assert_cmpint(write(f.master, "x", 1), ==, 1);
    wait_for_callbacks(&f, 1);
    g_assert_null(f.port);
    g_assert_null(g_main_context_find_source_by_id(NULL, f.source_id));
    fixture_clear(&f);
}

static void reopen_on_data(GSerialPort *port, gint available, Fixture *f)
{
    f->callbacks++;
    if (f->callbacks == 1) {
        f->source_id = g_source_get_id(g_main_current_source());
        gserial_port_close(port);
        g_assert_true(gserial_port_open(port, f->path));
    } else {
        gint count;
        guint8 *bytes = gserial_port_read_bytes(port, available, &count);
        g_assert_cmpint(count, ==, available);
        g_free(bytes);
        gserial_port_close(port);
    }
}

static void test_callback_reopen(void)
{
    Fixture f;
    fixture_init(&f, FALSE, 100);
    g_signal_connect(f.port, "on_data", G_CALLBACK(reopen_on_data), &f);
    g_assert_cmpint(write(f.master, "x", 1), ==, 1);
    wait_for_callbacks(&f, 1);
    g_assert_null(g_main_context_find_source_by_id(NULL, f.source_id));
    g_assert_true(gserial_port_is_open(f.port));
    g_assert_cmpint(write(f.master, "y", 1), ==, 1);
    wait_for_callbacks(&f, 2);
    g_assert_false(gserial_port_is_open(f.port));
    fixture_clear(&f);
    g_usleep(10000);
    g_main_context_iteration(NULL, FALSE);
}

static void reopen_during_dispose(GSerialPort *port, Fixture *f)
{
    f->callbacks++;
    g_assert_false(gserial_port_open(port, f->path));
}

static void test_dispose_reopen(void)
{
    Fixture f;
    fixture_init(&f, FALSE, 100);
    g_signal_connect(f.port, "disconnected", G_CALLBACK(reopen_during_dispose), &f);
    g_object_unref(f.port);
    g_assert_null(f.port);
    g_assert_cmpuint(f.callbacks, ==, 1);
    fixture_clear(&f);
    g_usleep(10000);
    g_main_context_iteration(NULL, FALSE);
}

static void test_binary_io(void)
{
    Fixture f;
    fixture_init(&f, FALSE, 100);
    guint8 expected[] = {0, 0xff, '\r', '\n', 'a'};
    g_assert_cmpint(write(f.master, expected, sizeof expected), ==, sizeof expected);
    gint count;
    guint8 *received = gserial_port_read_bytes(f.port, sizeof expected, &count);
    g_assert_cmpmem(received, count, expected, sizeof expected);
    g_free(received);
    g_assert_cmpint(gserial_port_write_bytes(f.port, expected, sizeof expected), ==, sizeof expected);
    guint8 output[sizeof expected];
    g_assert_cmpint(read(f.master, output, sizeof output), ==, sizeof expected);
    g_assert_cmpmem(output, sizeof output, expected, sizeof expected);
    gserial_port_close(f.port);
    gserial_port_close(f.port);
    g_assert_true(gserial_port_open(f.port, f.path));
    fixture_clear(&f);
}

static void test_write_timeout(gconstpointer use_string)
{
    Fixture f;
    fixture_init(&f, FALSE, 100);
    guint length = 1024 * 1024;
    guint8 *buffer = g_malloc(length + 1);
    memset(buffer, 'x', length);
    buffer[length] = 0;
    gint64 start = g_get_monotonic_time();
    gint written = use_string
        ? gserial_port_write_string(f.port, (const gchar *)buffer)
        : gserial_port_write_bytes(f.port, buffer, length);
    gint64 elapsed = g_get_monotonic_time() - start;
    g_assert_cmpint(written, >, 0);
    g_assert_cmpint(written, <, length);
    g_assert_cmpint(elapsed, >=, 90000);
    g_assert_cmpint(elapsed, <, 2 * G_TIME_SPAN_SECOND);
    g_assert_true(gserial_port_is_open(f.port));

    /* A partial result must match what reached the peer. */
    g_assert_cmpint(fcntl(f.master, F_SETFL, O_NONBLOCK), ==, 0);
    gint drained = 0;
    guint8 output[4096];
    ssize_t count;
    while ((count = read(f.master, output, sizeof output)) > 0) {
        g_assert_cmpmem(output, count, buffer + drained, count);
        drained += count;
    }
    g_assert_cmpint(drained, ==, written);

    /* The temporary nonblocking write must not change subsequent reads. */
    gint received;
    start = g_get_monotonic_time();
    guint8 *bytes = gserial_port_read_bytes(f.port, 1, &received);
    elapsed = g_get_monotonic_time() - start;
    g_assert_cmpint(received, ==, 0);
    g_assert_cmpint(elapsed, >=, 90000);
    g_free(bytes);
    g_free(buffer);
    fixture_clear(&f);
}

typedef struct {
    int fd;
    guint8 *buffer;
    guint length;
    guint count;
} Reader;

static gpointer drain_peer(gpointer data)
{
    Reader *reader = data;
    g_usleep(20000);
    while (reader->count < reader->length) {
        struct pollfd fd = {reader->fd, POLLIN, 0};
        if (poll(&fd, 1, 2000) <= 0)
            break;
        ssize_t count = read(reader->fd, reader->buffer + reader->count,
                            reader->length - reader->count);
        if (count <= 0)
            break;
        reader->count += count;
    }
    return NULL;
}

static void test_write_resumes(void)
{
    Fixture f;
    fixture_init(&f, FALSE, 1000);
    guint length = 256 * 1024;
    guint8 *input = g_malloc(length);
    for (guint i = 0; i < length; ++i)
        input[i] = (guint8)i;
    Reader reader = {f.master, g_malloc(length), length, 0};
    GThread *thread = g_thread_new("serial-reader", drain_peer, &reader);
    gint written = gserial_port_write_bytes(f.port, input, length);
    g_thread_join(thread);
    g_assert_cmpint(written, ==, length);
    g_assert_cmpmem(reader.buffer, reader.count, input, length);
    g_free(input);
    g_free(reader.buffer);
    fixture_clear(&f);
}

static void test_immediate_io(gconstpointer nonblocking)
{
    Fixture f;
    fixture_init(&f, nonblocking != NULL, nonblocking ? 1000 : 0);
    guint length = 1024 * 1024;
    guint8 *buffer = g_malloc0(length);
    gint64 start = g_get_monotonic_time();
    gint written = gserial_port_write_bytes(f.port, buffer, length);
    g_assert_cmpint(written, >=, 0);
    g_assert_cmpint(written, <, length);
    /* Saturation must return zero without closing the port. */
    for (int i = 0; i < 16 && written > 0; ++i)
        written = gserial_port_write_bytes(f.port, buffer, length);
    g_assert_cmpint(written, ==, 0);
    gint received;
    guint8 *bytes = gserial_port_read_bytes(f.port, 1, &received);
    g_assert_cmpint(received, ==, 0);
    g_assert_cmpint(g_get_monotonic_time() - start, <, G_TIME_SPAN_SECOND);
    g_assert_true(gserial_port_is_open(f.port));
    g_free(bytes);
    g_free(buffer);
    fixture_clear(&f);
}

static void test_configuration(void)
{
    Fixture f;
    fixture_init(&f, FALSE, 100);
    const gchar *names[] = {"baud", "timeout", "byte-size", "parity", "stop-bits"};
    const guint values[] = {9600, 1000, GSERIAL_BYTE_SIZE_SEVENBIT,
                           GSERIAL_PARITY_ODD, GSERIAL_STOP_BITS_TWO};
    for (guint i = 0; i < G_N_ELEMENTS(names); ++i) {
        guint original, actual;
        g_object_get(f.port, names[i], &original, NULL);
        g_test_expect_message(NULL, G_LOG_LEVEL_CRITICAL, "*!gserial_port_is_open(self)*");
        g_object_set(f.port, names[i], values[i], NULL);
        g_test_assert_expected_messages();
        g_object_get(f.port, names[i], &actual, NULL);
        g_assert_cmpuint(actual, ==, original);
    }
    gserial_port_close(f.port);
    gserial_port_set_baud(f.port, 9600);
    gserial_port_set_timeout(f.port, 200);
    gserial_port_set_stop_bits(f.port, GSERIAL_STOP_BITS_TWO);
    g_assert_true(gserial_port_open(f.port, f.path));
    struct termios attr;
    g_assert_cmpint(tcgetattr(f.slave, &attr), ==, 0);
    g_assert_cmpuint(cfgetospeed(&attr), ==, B9600);
    g_assert_cmpuint(attr.c_cc[VTIME], ==, 2);
    g_assert_true((attr.c_cflag & CSTOPB) != 0);
    fixture_clear(&f);
}

static void test_enum_types(void)
{
    GType types[] = {GSERIAL_TYPE_BYTE_SIZE, GSERIAL_TYPE_PARITY, GSERIAL_TYPE_STOP_BITS};
    const gchar *names[] = {"GSERIAL_BYTE_SIZE_EIGHTBIT", "GSERIAL_PARITY_EVEN", "GSERIAL_STOP_BITS_ONEHALF"};
    const gint values[] = {GSERIAL_BYTE_SIZE_EIGHTBIT, GSERIAL_PARITY_EVEN, GSERIAL_STOP_BITS_ONEHALF};
    for (guint i = 0; i < G_N_ELEMENTS(types); ++i) {
        g_assert_true(G_TYPE_IS_ENUM(types[i]));
        GEnumClass *klass = g_type_class_ref(types[i]);
        GEnumValue *value = g_enum_get_value_by_name(klass, names[i]);
        g_assert_nonnull(value);
        g_assert_cmpint(value->value, ==, values[i]);
        g_type_class_unref(klass);
    }
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/gserial/lifetime/release-in-callback", test_callback_release);
    g_test_add_func("/gserial/lifetime/reopen-in-callback", test_callback_reopen);
    g_test_add_func("/gserial/lifetime/reopen-during-dispose", test_dispose_reopen);
    g_test_add_func("/gserial/io/binary", test_binary_io);
    g_test_add_data_func("/gserial/io/write-bytes-timeout", NULL, test_write_timeout);
    g_test_add_data_func("/gserial/io/write-string-timeout", GINT_TO_POINTER(1), test_write_timeout);
    g_test_add_func("/gserial/io/write-resumes", test_write_resumes);
    g_test_add_data_func("/gserial/io/nonblocking", GINT_TO_POINTER(1), test_immediate_io);
    g_test_add_data_func("/gserial/io/zero-timeout", NULL, test_immediate_io);
    g_test_add_func("/gserial/configuration", test_configuration);
    g_test_add_func("/gserial/enum-types", test_enum_types);
    return g_test_run();
}
