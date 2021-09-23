[CCode (cheader_filename = "gserial_os.h")]

namespace gserial_os {
	[CCode (cname = "gserialOsHandleType", has_type_id = false)]
    public struct HandleType {
	}
	public void set_interface_attr(HandleType* t, int baud, int bytesize, int parity);
	public HandleType* open(char* path);
	public void close(HandleType* t);
	public int is_open(HandleType* t);
	public int available(HandleType* t);
	public int read(HandleType* t, char* c, int len);
	public int write(HandleType* t, char* c, int len);
}


