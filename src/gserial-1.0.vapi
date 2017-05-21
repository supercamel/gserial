/* gserial-1.0.vapi generated by vapigen-0.36, do not modify. */

[CCode (cprefix = "gserial", gir_namespace = "gserial", gir_version = "1.0", lower_case_cprefix = "gserial_")]
namespace gserial {
	[CCode (cheader_filename = "gserial.h", type_id = "gserial_port_get_type ()")]
	public class Port : GLib.Object {
		[CCode (has_construct_function = false)]
		public Port ();
		public uint available ();
		public void close ();
		public void get_stuff (out GLib.Array<char> garray);
		public bool is_open ();
		public void open (string path);
		public void read_bytes (uint len_in, out GLib.Array<char> garray);
		public void read_bytes_until (char stop_char, out GLib.Array<char> garray);
		public void read_string (uint len_in, out string str);
		public void set_baud (int baud);
		public void set_byte_size (gserial.ByteSize sz);
		public void set_parity (gserial.Parity p);
		public void set_timeout (int timeout);
		[CCode (has_construct_function = false)]
		public Port.with_params (gserial.ByteSize sz, gserial.Parity p, int timeout, int baud);
		public int write (GLib.Array<char> garray);
		[NoAccessorMethod]
		public int baud { get; set construct; }
		[NoAccessorMethod]
		public int byte_size { get; set construct; }
		[NoAccessorMethod]
		public int parity { get; set construct; }
		[NoAccessorMethod]
		public int timeout { get; set construct; }
	}
	[CCode (cheader_filename = "gserial.h", has_type_id = false)]
	public struct PortInfo {
		public weak string path;
	}
	[CCode (cheader_filename = "gserial.h", cprefix = "GSERIAL_BYTE_SIZE_", has_type_id = false)]
	public enum ByteSize {
		FIVEBITS,
		SIXBITS,
		SEVENBITS,
		EIGHTBITS
	}
	[CCode (cheader_filename = "gserial.h", cprefix = "GSERIAL_PARITY_", has_type_id = false)]
	public enum Parity {
		NONE,
		ODD,
		EVEN
	}
	[CCode (cheader_filename = "gserial.h")]
	public static GLib.Array<gserial.PortInfo> enumerate_ports ();
}
