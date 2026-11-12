package org.gserial;

import android.os.SystemClock;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.concurrent.ConcurrentHashMap;

/** Installed only with -Dandroid_test_backend=true. Never emulates physical USB. */
public final class TestUsbBackend implements UsbSerialBridge.TestBackend {
    private static final ConcurrentHashMap<String, Boolean> grants = new ConcurrentHashMap<>();
    private static final String LOOP = "gserial-test://loopback", DENY = "gserial-test://denied";
    public String[] listDevices() {
        String[] a = row(LOOP), b = row(DENY), result = new String[16];
        System.arraycopy(a, 0, result, 0, 8); System.arraycopy(b, 0, result, 8, 8);
        return result;
    }
    private String[] row(String path) {
        return new String[]{path, "TEST ONLY · USB bridge loopback", "1027", "24577", "TEST-RFD",
            "0", Boolean.toString(Boolean.TRUE.equals(grants.get(path))), "TestEndpoint"};
    }
    public String[] lookup(String path) throws IOException {
        if (!path.equals(LOOP) && !path.equals(DENY)) throw new IOException("Unknown USB test port");
        return row(path);
    }
    public UsbSerialBridge.Permission requestPermission(String path) throws IOException {
        lookup(path);
        return new UsbSerialBridge.Permission(0) {
            final long ready = SystemClock.elapsedRealtime() + 75;
            boolean cancelled;
            @Override public int state() {
                if (cancelled) return -1;
                if (SystemClock.elapsedRealtime() < ready) return 0;
                boolean granted = path.equals(LOOP);
                grants.put(path, granted);
                return granted ? 1 : -1;
            }
            @Override public void close() { cancelled = true; }
        };
    }
    public UsbSerialBridge.Session open(String path, int baud, int bits, int parity, int stops) throws IOException {
        lookup(path);
        if (!Boolean.TRUE.equals(grants.get(path))) throw new IOException("Test USB permission required");
        if (baud <= 0 || bits < 5 || bits > 8 || parity < 0 || parity > 2 || stops < 1 || stops > 3)
            throw new IOException("Invalid serial configuration");
        return new UsbSerialBridge.Session(new UsbSerialBridge.Endpoint() {
            final ArrayDeque<Byte> queue = new ArrayDeque<>();
            boolean closed;
            public synchronized int read(byte[] bytes, int timeout) throws IOException {
                if (queue.isEmpty() && !closed) {
                    try { wait(timeout); } catch (InterruptedException error) { Thread.currentThread().interrupt(); }
                }
                if (closed) throw new IOException("Test adapter detached");
                int count = 0;
                while (count < bytes.length && !queue.isEmpty()) bytes[count++] = queue.removeFirst();
                return count;
            }
            public synchronized int write(byte[] bytes, int timeout) throws IOException {
                if (closed) throw new IOException("Test adapter is closed");
                if (bytes.length == 1 && bytes[0] == 0x04) { closed = true; notifyAll(); return 1; }
                int count = Math.min(bytes.length, 3); // exercise caller retries
                for (int i = 0; i < count; ++i) queue.addLast(bytes[i]);
                notifyAll();
                return count;
            }
            public void setControl(boolean dtr, boolean rts) {}
            public synchronized void close() { closed = true; notifyAll(); }
        });
    }
}
