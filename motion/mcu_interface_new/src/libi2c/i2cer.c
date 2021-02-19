#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "i2c.h"

int main(){
  int bus;
while(1){
if ((bus = i2c_open("/dev/i2c-8")) == -1) {

}
I2CDevice device;
memset(&device, 0, sizeof(device));


device.bus = bus;
device.addr = 0x8;
device.iaddr_bytes = 1;
device.page_bytes = 16;

unsigned char buffer[256];
ssize_t size = sizeof(buffer);
memset(buffer, 0, sizeof(buffer));
int fd = open("/dev/urandom", O_RDONLY);
read(fd, buffer, sizeof(buffer));

if ((i2c_ioctl_write(&device, 0x8, buffer, size)) != size) {

	/* Error process */
}
i2c_close(bus);
}

return 0;
}
