using gserial_os;

namespace gserial {
	public enum BYTE_SIZE {
		FIVEBIT,
		SIXBIT,
		SEVENBIT,
		EIGHTBIT
	}

	public enum PARITY {
		NONE,
		ODD,
		EVEN
	}

	public enum STOP_BITS {
		ONE,
		TWO,
		ONEHALF
	}

	public errordomain ComError {
		OPEN_ERROR,
		READ_ERROR,
		WRITE_ERROR
	}

	public class Port {
		public Port() {

		}

		public Port.with_params(BYTE_SIZE bs, PARITY p, uint t, uint b) {
			byte_size = bs;
			parity = p;
			timeout = t;
			baud = b;
		}

		~Port() {
			if(is_open()) {
				close();
			}
		}

		public void open(string path) throws ComError {
			handle = gserial_os.open(path);
			gserial_os.set_interface_attr(handle, (int)baud, byte_size, parity);
		}

		public void close() {
			gserial_os.close(handle);
		}

		public bool is_open() {
			return (gserial_os.is_open(handle) == 1);
		}

		public uint available() {
			return gserial_os.available(handle);
		}

		public uint8[] read_bytes(uint len) throws ComError {
			uint8[] r = new uint8[len];
			if(gserial_os.read(handle, r, (int)len) < 0) {
				throw new ComError.READ_ERROR("Error reading");
			}
			return r;
		}

		public string read_string(uint len) throws ComError {
			char[] s = new char[len];
			if(gserial_os.read(handle, s, (int)len) < 0) {
				throw new ComError.READ_ERROR("Error reading");
			}
			return (string)s;
		}

		public void write_bytes(uint8[] b, uint len) throws ComError {
			if(gserial_os.write(handle, b, (int)len) < 0) {
				throw new ComError.WRITE_ERROR("Error writing");
			}
		}


		public void write_string(string s) throws ComError {
			if(gserial_os.write(handle, (uint8[])s, s.length) < 0) {
				throw new ComError.WRITE_ERROR("Error writing");
			}
		}

		public uint baud { set; get; }
		public BYTE_SIZE byte_size { set; get; }
		public PARITY parity { set; get; }
		public STOP_BITS stop_bits { set; get; }
		public uint timeout { set; get; }

		private gserial_os.HandleType* handle;
	}

}


static int main() {
	return 0;
}

