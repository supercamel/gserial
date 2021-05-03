#include <girepository.h>
#include <stdio.h>
#include "gserial.h"

int
main (int argc, char *argv[])
{
	GOptionContext *ctx;
    GError *error = NULL;

    ctx = g_option_context_new (NULL);
    g_option_context_add_group (ctx, g_irepository_get_option_group ());  

    if (!g_option_context_parse (ctx, &argc, &argv, &error)) {
        g_print ("greeter: %s\n", error->message);
        return 1;
    }
    
    /* Create our object */
    
    gserialPort *port = g_object_new (GSERIAL_TYPE_PORT, NULL);

    //gserial_port_set_byte_size(port, GSERIAL_BYTE_SIZE_EIGHTBITS);
    GValue value = G_VALUE_INIT;
    g_value_init(&value, G_TYPE_INT);
    g_value_set_int(&value, GSERIAL_BYTE_SIZE_EIGHTBITS);
    g_object_set_property((GObject*)port, "byte_size", &value);

	gserial_port_open(port, "/dev/ttyUSB0");
	if(gserial_port_is_open(port)) 
	{
		printf("port is open!\n");
	}
	
    return 0;
}

