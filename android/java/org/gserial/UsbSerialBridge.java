package org.gserial;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.net.Uri;
import android.os.Build;
import android.os.SystemClock;
import com.hoho.android.usbserial.driver.SerialTimeoutException;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.UUID;

/** JNI entry points. No native pointer is retained by Java or a BroadcastReceiver. */
public final class UsbSerialBridge {
    private UsbSerialBridge() {}
    private static final Set<String> openPorts = new HashSet<>();

    /* The optional test class is absent from production builds and packages. */
    public interface TestBackend {
        String[] listDevices();
        String[] lookup(String path) throws IOException;
        Permission requestPermission(String path) throws IOException;
        Session open(String path, int baud, int bits, int parity, int stops) throws IOException;
    }
    private static TestBackend testBackend() {
        try {
            return (TestBackend)Class.forName("org.gserial.TestUsbBackend").getDeclaredConstructor().newInstance();
        } catch (ClassNotFoundException absent) {
            return null;
        } catch (ReflectiveOperationException error) {
            throw new IllegalStateException("Cannot initialize the USB test fixture", error);
        }
    }
    private static UsbManager manager(Context context) throws IOException {
        UsbManager manager = (UsbManager)context.getSystemService(Context.USB_SERVICE);
        if (manager == null) throw new IOException("Android USB host service is unavailable");
        return manager;
    }
    private static void register(Context context, BroadcastReceiver receiver, IntentFilter filter) {
        if (Build.VERSION.SDK_INT >= 33) context.registerReceiver(receiver, filter, Context.RECEIVER_NOT_EXPORTED);
        else context.registerReceiver(receiver, filter);
    }
    private static String serial(UsbManager manager, UsbDevice device) {
        if (!manager.hasPermission(device)) return "";
        try {
            String value = device.getSerialNumber();
            return value == null ? "" : value;
        } catch (SecurityException unavailable) { return ""; }
    }
    private static String authority(UsbDevice device) {
        return String.format(Locale.ROOT, "%04x:%04x", device.getVendorId(), device.getProductId());
    }
    private static String path(UsbManager manager, UsbSerialPort port, List<UsbSerialDriver> drivers) {
        UsbDevice device = port.getDevice();
        String serial = serial(manager, device);
        if (!serial.isEmpty()) {
            int matches = 0;
            for (UsbSerialDriver driver : drivers) {
                UsbDevice candidate = driver.getDevice();
                if (authority(candidate).equals(authority(device)) && serial.equals(serial(manager, candidate))) matches++;
            }
            if (matches != 1) serial = "";
        }
        return new Uri.Builder().scheme("usb").encodedAuthority(authority(device))
            .appendPath(serial.isEmpty() ? "@" + device.getDeviceId() : "s" + serial)
            .appendPath(Integer.toString(port.getPortNumber())).build().toString();
    }
    private static String[] row(UsbManager manager, UsbSerialPort port, List<UsbSerialDriver> drivers) {
        UsbDevice device = port.getDevice();
        String driver = port.getDriver().getClass().getSimpleName().replace("SerialDriver", "");
        String product = device.getProductName(), serial = serial(manager, device);
        String name = (product == null || product.isEmpty() ? driver : product) +
            (serial.isEmpty() ? "" : " · " + serial) + " · port " + port.getPortNumber() + " (" + driver + ")";
        return new String[]{path(manager, port, drivers), name,
            Integer.toString(device.getVendorId()), Integer.toString(device.getProductId()),
            serial, Integer.toString(port.getPortNumber()), Boolean.toString(manager.hasPermission(device)), driver};
    }
    private static UsbSerialPort find(UsbManager manager, String path, List<UsbSerialDriver> drivers) throws IOException {
        Uri uri = Uri.parse(path);
        List<String> parts = uri.getPathSegments();
        if (!"usb".equals(uri.getScheme()) || parts.size() != 2 || uri.getQuery() != null || uri.getFragment() != null)
            throw new IOException("Invalid USB serial URI");
        int number;
        try { number = Integer.parseInt(parts.get(1)); }
        catch (NumberFormatException invalid) { throw new IOException("Invalid USB port number"); }
        if (number < 0 || !Integer.toString(number).equals(parts.get(1))) throw new IOException("Invalid USB port number");
        String selector = parts.get(0);
        UsbSerialPort found = null;
        for (UsbSerialDriver driver : drivers) {
            UsbDevice device = driver.getDevice();
            if (!authority(device).equals(uri.getEncodedAuthority()) || number >= driver.getPorts().size()) continue;
            boolean matches = selector.equals("@" + device.getDeviceId()) ||
                (selector.startsWith("s") && selector.length() > 1 && selector.substring(1).equals(serial(manager, device)));
            if (matches) {
                if (found != null) throw new IOException("USB identity is ambiguous; select the adapter again");
                found = driver.getPorts().get(number);
            }
        }
        if (found == null) throw new IOException("USB adapter is unavailable or needs permission; refresh the device list");
        return found;
    }
    public static String[] listDevices(Context context) throws IOException {
        UsbManager manager = manager(context);
        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager);
        ArrayList<String> result = new ArrayList<>();
        for (UsbSerialDriver driver : drivers)
            for (UsbSerialPort port : driver.getPorts()) result.addAll(Arrays.asList(row(manager, port, drivers)));
        TestBackend test = testBackend();
        if (test != null) result.addAll(Arrays.asList(test.listDevices()));
        return result.toArray(new String[0]);
    }
    public static String[] lookup(Context context, String path) throws IOException {
        TestBackend test = path.startsWith("gserial-test://") ? testBackend() : null;
        if (test != null) return test.lookup(path);
        UsbManager manager = manager(context);
        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager);
        return row(manager, find(manager, path, drivers), drivers);
    }

    public static class Permission {
        private volatile int result;
        private Context context;
        private UsbManager manager;
        private UsbDevice device;
        private BroadcastReceiver receiver;
        private PendingIntent intent;
        public Permission(int result) { this.result = result; }
        Permission(Context context, UsbManager manager, UsbDevice device) {
            this.context = context.getApplicationContext(); this.manager = manager; this.device = device;
            if (manager.hasPermission(device)) { result = 1; return; }
            String action = "org.gserial.USB_PERMISSION." + UUID.randomUUID();
            receiver = new BroadcastReceiver() {
                @Override public void onReceive(Context ignored, Intent value) {
                    UsbDevice received = value.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    if (received == null || received.getDeviceId() != device.getDeviceId()) return;
                    if (action.equals(value.getAction())) finish(manager.hasPermission(device) ? 1 : -1);
                    else if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(value.getAction())) finish(-1);
                }
            };
            IntentFilter filter = new IntentFilter(action);
            filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
            try {
                register(this.context, receiver, filter);
                intent = PendingIntent.getBroadcast(this.context, 0,
                    new Intent(action).setPackage(this.context.getPackageName()),
                    PendingIntent.FLAG_MUTABLE | PendingIntent.FLAG_CANCEL_CURRENT);
                manager.requestPermission(device, intent);
            } catch (RuntimeException error) { close(); throw error; }
        }
        private synchronized void finish(int value) { result = value; close(); }
        public int state() {
            if (result == 0 && device != null && !manager.getDeviceList().containsKey(device.getDeviceName())) finish(-1);
            return result;
        }
        public synchronized void close() {
            if (receiver != null) {
                try { context.unregisterReceiver(receiver); } catch (IllegalArgumentException ignored) {}
                receiver = null;
            }
            if (intent != null) { intent.cancel(); intent = null; }
        }
    }
    public static Permission requestPermission(Context context, String path) throws IOException {
        TestBackend test = path.startsWith("gserial-test://") ? testBackend() : null;
        if (test != null) return test.requestPermission(path);
        UsbManager manager = manager(context);
        UsbSerialPort port = find(manager, path, UsbSerialProber.getDefaultProber().findAllDrivers(manager));
        return new Permission(context, manager, port.getDevice());
    }

    public interface Endpoint {
        int read(byte[] bytes, int timeout) throws IOException;
        int write(byte[] bytes, int timeout) throws IOException;
        void setControl(boolean dtr, boolean rts) throws IOException;
        void close() throws IOException;
    }
    public static class Session {
        private final Endpoint endpoint;
        private final byte[] buffer = new byte[65536];
        private final Object writeLock = new Object();
        private int start, size;
        private volatile boolean closed;
        private IOException failure;
        private final Thread reader;
        private Context context;
        private BroadcastReceiver detach;
        private String key;
        public Session(Endpoint endpoint) {
            this.endpoint = endpoint;
            reader = new Thread(this::receive, "GSerial USB reader");
            reader.setDaemon(true);
            reader.start();
        }
        private void receive() {
            byte[] incoming = new byte[4096];
            try {
                while (!closed) {
                    int count = endpoint.read(incoming, 200);
                    synchronized (this) {
                        if (closed) break;
                        if (count < 0 || count > incoming.length) throw new IOException("Invalid USB read count");
                        if (count > buffer.length - size) throw new IOException("USB receive buffer overflow");
                        for (int i = 0; i < count; ++i) buffer[(start + size + i) % buffer.length] = incoming[i];
                        size += count;
                        if (count > 0) notifyAll();
                    }
                }
            } catch (IOException error) { failed(error); }
            catch (RuntimeException error) { failed(new IOException("USB reader failed", error)); }
        }
        private synchronized void failed(IOException error) { if (!closed) failure = error; notifyAll(); }
        private void check() throws IOException {
            if (failure != null) throw failure;
            if (closed) throw new IOException("USB port is closed");
        }
        public synchronized int available() throws IOException { check(); return size; }
        public synchronized byte[] read(int maximum, int timeout) throws IOException {
            check();
            long deadline = SystemClock.elapsedRealtime() + timeout;
            while (size == 0 && maximum > 0 && timeout > 0) {
                long remaining = deadline - SystemClock.elapsedRealtime();
                if (remaining <= 0) break;
                try { wait(remaining); }
                catch (InterruptedException interrupted) { Thread.currentThread().interrupt(); return new byte[0]; }
                check();
            }
            int count = Math.min(maximum, size);
            byte[] bytes = new byte[count];
            for (int i = 0; i < count; ++i) bytes[i] = buffer[(start + i) % buffer.length];
            start = (start + count) % buffer.length; size -= count;
            return bytes;
        }
        public int write(byte[] bytes, int timeout) throws IOException {
            synchronized (this) { check(); }
            synchronized (writeLock) { return endpoint.write(bytes, Math.max(timeout, 1)); }
        }
        public void setControl(boolean dtr, boolean rts) throws IOException {
            synchronized (this) { check(); }
            synchronized (writeLock) { endpoint.setControl(dtr, rts); }
        }
        public void close() {
            synchronized (this) { if (closed) return; closed = true; notifyAll(); }
            if (detach != null) {
                try { context.unregisterReceiver(detach); } catch (IllegalArgumentException ignored) {}
                detach = null;
            }
            try { endpoint.close(); } catch (IOException ignored) {}
            try { reader.join(500); } catch (InterruptedException ignored) { Thread.currentThread().interrupt(); }
            if (key != null) synchronized (openPorts) { openPorts.remove(key); }
        }
    }
    public static Session open(Context context, String path, int baud, int bits, int parity, int stops) throws IOException {
        TestBackend test = path.startsWith("gserial-test://") ? testBackend() : null;
        if (test != null) return test.open(path, baud, bits, parity, stops);
        UsbManager manager = manager(context);
        UsbSerialPort port = find(manager, path, UsbSerialProber.getDefaultProber().findAllDrivers(manager));
        if (!manager.hasPermission(port.getDevice())) throw new IOException("USB permission is required before opening the adapter");
        String key = port.getDevice().getDeviceName() + ":" + port.getPortNumber();
        synchronized (openPorts) { if (!openPorts.add(key)) throw new IOException("USB port is already open"); }
        UsbDeviceConnection connection = null;
        Session session = null;
        try {
            connection = manager.openDevice(port.getDevice());
            if (connection == null) throw new IOException("Cannot open the USB adapter");
            port.open(connection);
            port.setParameters(baud, bits, stops, parity);
            session = new Session(new Endpoint() {
                public int read(byte[] bytes, int timeout) throws IOException { return port.read(bytes, timeout); }
                public int write(byte[] bytes, int timeout) throws IOException {
                    try { port.write(bytes, timeout); return bytes.length; }
                    catch (SerialTimeoutException partial) {
                        if (partial.bytesTransferred < 0 || partial.bytesTransferred > bytes.length) throw partial;
                        return partial.bytesTransferred;
                    }
                }
                public void setControl(boolean dtr, boolean rts) throws IOException { port.setDTR(dtr); port.setRTS(rts); }
                public void close() throws IOException { port.close(); }
            });
            session.key = key; session.context = context.getApplicationContext();
            Session current = session;
            session.detach = new BroadcastReceiver() {
                @Override public void onReceive(Context ignored, Intent intent) {
                    UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    if (device != null && device.getDeviceId() == port.getDevice().getDeviceId())
                        current.failed(new IOException("USB adapter detached"));
                }
            };
            register(session.context, session.detach, new IntentFilter(UsbManager.ACTION_USB_DEVICE_DETACHED));
            return session;
        } catch (IOException | RuntimeException error) {
            if (session != null) session.close();
            else {
                try { port.close(); } catch (IOException ignored) {}
                if (connection != null) connection.close();
            }
            synchronized (openPorts) { openPorts.remove(key); }
            throw error;
        }
    }
}
