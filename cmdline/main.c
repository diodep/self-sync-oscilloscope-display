#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "bad.h"


void disp(int fd, const uint8_t *fb)
{
	uint8_t frame_start = 'U';
	uint8_t frame[1024];
	uint16_t sample;
	int x, y;
	int d = 0;
	/*for(y = 0; y < 64; y++)
	{
		frame[d++] = 256;
		frame[d++] = 0x00;
		frame[d++] = 256;
		for(x = 0; x < 13; x ++)
		{
			if(fb[(y / 8) * 128 + x ] & (1 << (y % 8)))
				sample = 2048 - 16 * y;
			else
				sample = 256;

			frame[d++] = sample;
		}
	}*/
	for(x = 0; x < 1024; x++)
	{
		frame[x] = ~fb[x];
	}
	write(fd, &frame_start, 1);
	write(fd, frame, 1024);
}
int main()
{
	int fd;
	fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	if(fd == -1)
	{
		fprintf(stderr, "Can't open serial port!\r\n");
		exit(1);
	}
	printf("Bad apple header size: %d\n", sizeof(badapple));
	//disp(fd, badapple);
	//return 0;
	int i;
	for(i = 0; i < sizeof(badapple) / (128 * 64 / 8); i++)
	{
		disp(fd, badapple + i * (128 * 64 / 8));
		usleep(66.3 * 1000);
	}

}
