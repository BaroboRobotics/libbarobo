#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

int main() {
  uint8_t data[4] = {0x35, 0xee, 0xaa, 0x33};
  int fh;
  int i;
  size_t size;

  for(i = 0; i < 10; i++) {
    fh = open("/dev/i2c-3", O_RDWR);

    if(fh < 0) {
      perror("open error");
    }

    /* Set slave address */
    ioctl(fh, I2C_SLAVE, 0x55);

    /* Write data */
    size = write(fh, data, 2);
    printf("Wrote %d bytes.\n", size);
    if(size) {perror("write failed");}
    close(fh);
    sleep(2);
  }
  printf("\n");
}
