#include "mcu_interface_new/i2c.h"
#include <cstring>


int bus;

int main(){

/* Open i2c bus /dev/i2c-0 */
	if (bus = i2c_open("/dev/i2c-8") == -1) {

	/* Error process */
	}
	
	I2CDevice device;
	memset(&device, 0, sizeof(device));
	
	/* 24C04 */
	device.bus = bus;	/* Bus 0 */
	device.addr = 0x8;	/* Slave address is 0x50, 7-bit */
	device.iaddr_bytes = 1;	/* Device internal address is 1 byte */
	device.page_bytes = 16; /* Device are capable of 16 bytes per page */

	unsigned char buffer[256];
	ssize_t size = sizeof(buffer);
	memset(buffer, 0, sizeof(buffer));

	/* From i2c 0x0 address read 256 bytes data to buffer */
	if ((i2c_write(&device, 0x8, buffer, size)) != size) {

	/* Error process */
	}

	i2c_close(bus);

	return 0;

}
