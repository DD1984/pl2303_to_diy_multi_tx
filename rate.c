#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <err.h>
#include <linux/serial.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <endian.h>

#define TTY_DEV "/dev/ttyUSB0"

#define MAX_CH 16
#define RESOLUTION_BIT 11
#define MB (MAX_CH * RESOLUTION_BIT / 8)

#define CUR_PROTO         4 //Hisky
#define CUR_SUB_PROTO     1 //HK310
#define CUR_RX_NUM        3 // кол-во передаваемых каналов

typedef struct {
	//0
	uint8_t hdr;
	//1
	uint8_t proto:5;
	uint8_t range:1;     //0x20	1=Yes /0=No
	uint8_t autobind:1;  //0x40	1=Yes /0=No
	uint8_t bind:1;      //0x80	1=Bind/0=No
	//2
	uint8_t rx_num:4;
	uint8_t sub_proto:3;
	uint8_t low_power:1;     //0x80	0=High/1=Low
	//3
	uint8_t opt;
	//4
	uint8_t data[MB];
} __attribute__((packed)) serial_t;

int serial_open(const char *device, int rate)
{
	int fd;
	struct termios2 options;

	/* Open and configure serial port */
	if ((fd = open(device,O_RDWR|O_NOCTTY)) == -1) {
		printf("open() err\n");
		return -1;
	}

	if (ioctl(fd, TCGETS2, &options) < 0) {
		printf("ioctl TCGETS2 err\n");
		close(fd);
		return -1;
	}

	options.c_cflag &= ~CBAUD; //Remove current BAUD rate
	options.c_cflag |= BOTHER; //Allow custom BAUD rate using int input

	options.c_ispeed = rate; //Set the input BAUD rate
	options.c_ospeed = rate; //Set the output BAUD rate

	options.c_cflag |= CS8;
	options.c_cflag |= CSTOPB; // CSTOPB = 2 Stop bits
	options.c_cflag |= PARENB;

	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~CRTSCTS;


	if (ioctl(fd, TCSETS2, &options) < 0) {
		printf("ioctl TCSETS2 err\n");
		close(fd);
		return -1;
	}

	return fd;
}

void set_ch(char *buf, int num, uint16_t val)
{
	int byte_num = (RESOLUTION_BIT * num) / 8;
	int off_32bit = ((RESOLUTION_BIT * num) % 8);

	uint32_t *p = (uint32_t *)(buf + byte_num);
	uint32_t mask = (1 << RESOLUTION_BIT) - 1;

	*p &= ~(mask << off_32bit);
	*p |= val << off_32bit;
}

void usage(void)
{
	printf("usage:\n");
	printf("\t-b              - bind\n");
	printf("\t-c CH1 ... CH16 - channels values (0..%u)\n", (1 << RESOLUTION_BIT) - 1);
}

void main(int argc, char *argv[])
{
	typedef enum { UNKNOWN, BIND = 1, TX } mode_t;
	mode_t mode = UNKNOWN;
	
	if (argv[1] == NULL) {
		usage();
		return;
	}

	uint16_t channels[16];
	memset(channels, 0, sizeof(channels));

	if (strcmp(argv[1], "-b") == 0) {
		mode = BIND;
	}
	else if (strcmp(argv[1], "-c") == 0) {
		mode = TX;

		char **p = &argv[2];
		int cnt = 0;
		while (*p != NULL) {
			unsigned int c;
			if (sscanf(*p, "%u", &c) == 1 && (c < (1 << RESOLUTION_BIT))) {
				channels[cnt] = c;
				cnt++;
			}
			else {
				usage();
				return;
			}
			p++;
			if (cnt >= 16)
				break;
		}
	}

	if (mode == UNKNOWN) {
		usage();
		return;
	}

	int fd = serial_open(TTY_DEV, 100000);
	if ( fd < 0) {
		printf("serial_open() err\n");
		return;
	}

	char buf[sizeof(serial_t)];

	memset(buf, 0, sizeof(buf));

	serial_t *pkt = (serial_t *)buf;

	pkt->hdr = 0x55;
	pkt->proto = CUR_PROTO;
	pkt->sub_proto = CUR_SUB_PROTO;
	pkt->rx_num = CUR_RX_NUM;
	pkt->opt = 0;

	if (mode == BIND) {
		pkt->bind = 1;
		pkt->low_power = 1;
	}
	else {
		pkt->bind = 0;
		pkt->low_power = 0;
	}

	int i;
	for (i = 0; i < 16; i++)
		set_ch(pkt->data, i, channels[i]);

	while (1) {
		write(fd, buf, sizeof(serial_t));
		usleep(50000);
	}

	close(fd);
}
