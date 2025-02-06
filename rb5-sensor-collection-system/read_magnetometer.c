#include <stdio.h> 
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#define I2C_BUS 1

int i2c_open(unsigned char bus, unsigned char addr)
{
  int file;
  char filename[16];
  sprintf(filename,"/dev/i2c-%d", bus);
  if ((file = open(filename,O_RDWR)) < 0)
  {
    fprintf(stderr, "i2c_open open error: %s\n", strerror(errno));
    return(file);
  }
  if (ioctl(file,I2C_SLAVE,addr) < 0)
  {
    fprintf(stderr, "i2c_open ioctl error: %s\n", strerror(errno));
    return(-1);
  }
  return(file);
}

int i2c_read(int handle, unsigned char* buf, unsigned int length)
{

  if (read(handle, buf, length) != length)
  {
    fprintf(stderr, "i2c_read error: %s\n", strerror(errno));
    return(-1);
  }
  return(length);
}

int i2c_close(int handle)
{
  if ((close(handle)) != 0)
  {
    fprintf(stderr, "i2c_close error: %s\n", strerror(errno));
    return(-1);
  }
  return(0);
}

int main(int argc, char *argv[]){
    int handle, opt;
    uint8_t buf[6];
    uint16_t magData[3];
    float range_scale = 0.003f;

    FILE* logging_path;
    int logging = 0;
    int duration = 10; //default logging duration
    int rate = 500; //sample collection rate

    // Parse all command line options
    while ((opt = getopt(argc, argv, "p:t:r:")) != -1) {
        switch (opt) {
        case 'p':
            logging_path = fopen(optarg, "w");
            fprintf(logging_path,"Timestamp,MagX,Magy,MaxZ\n");
            logging = 1;
            //printf("Path is recorded\n");
            break;
        case 't':
            duration = atoi(optarg);
            //printf("time is set\n");
            break;
        case 'r':
            rate = atoi(optarg);
            break;
        case ':':
            fprintf(stderr, "Error - option %c needs a value\n\n", optopt);
            return -1;
        }
    }

    handle = i2c_open(I2C_BUS, 0x0E); // open I2C file handle
    i2c_read(handle, buf, 6);

    unsigned int i = 0;

    struct timespec tp, curr, start;
    int time_elapsed = 0;
    clockid_t clk_id = CLOCK_REALTIME;

    clock_gettime(clk_id, &start);
    while(logging){
      if (i2c_read(handle, buf, 6) == -1){
        printf("Error reading from i2c");
      } 
      else {
        magData[0] = (((int16_t)buf[1] << 8) | ((int16_t)buf[0]));
        magData[1] = (((int16_t)buf[3] << 8) | ((int16_t)buf[2]));
        magData[2] = (((int16_t)buf[5] << 8) | ((int16_t)buf[4]));

        clock_gettime(clk_id, &tp);
        fprintf(logging_path, "%lld.%ld,", tp.tv_sec, tp.tv_nsec);

        fprintf(logging_path, "%d,%d,%d\n", magData[0], magData[1], magData[2]);
      }
        // grab time and exit if past duration
        clock_gettime(clk_id, &curr);
        time_elapsed = curr.tv_sec - start.tv_sec;

        if (time_elapsed >= duration){
            fclose(logging_path);
            break;      
        }
    }

    i2c_close(handle);
    return 0;
}
