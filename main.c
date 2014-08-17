#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/time.h>
#include <fcntl.h>

#define I2C_DEV_0 "/dev/i2c-0"
#define I2C_ADDR 0x20
#define MSEC_TO_USEC(msec) (msec * 1000)
#define POLL_TIMEOUT_MS 3
#define SIZE 2

/*LEDs are connected to port B*/

#define PORT_A 0x00
#define PORT_B 0x01
#define PORT_A_LATCH 0x14
#define PORT_B_LATCH 0x15
#define FALSE 0
#define READ 0x12
#define PORT_DIR 0x07 /*PINS ONE TWO AND THREE AS INPUT*/

#define MIDDLE_BUT_PRESSED(c) (c & 0x20)
#define RIGHT_BUT_PRESSED(c) (c & 0x40)
#define LEFT_BUT_PRESSED(c) (c & 0x80)

#define TRUE 1
#define FALSE 0

#define ENC_DIR_CW 0
#define ENC_DIR_CCW 1
#define ENC_PORTA 0x08
#define ENC_PORTB 0x0F


struct front_panel
{
	int fd;					//File descriptor for the front panel
	void (*on_but_press) (char *data); 	//Call back handler for the buttons
	void (*on_enc_rot) (char *data);	//Call back handler for the encoder
};

/*
	Initialize the direction of the ports.

*/
int init_port(const int fd,const unsigned char port, const unsigned char dir, unsigned char *latch)
{
	char data[2];

	data[0] = port;
	data[1] = dir;

	if (port == PORT_A) {
		*latch = PORT_A_LATCH;
	} else {
		*latch = PORT_B_LATCH;
	}

	return write(fd,data,2);
}

int init_device(int *fd)
{
	if ((*fd = open(I2C_DEV_0,O_RDWR)) == -1) {
		printf("Unable to open device\n");
		return EXIT_FAILURE;
	}

	if (ioctl(*fd,I2C_SLAVE,I2C_ADDR) < 0) {
		printf("Unable to obtain address.\n");
		close(*fd);
	}
}


void write_register(const int fd, unsigned char addr, unsigned char reg, unsigned char value)
{
	unsigned char buf[2] = {reg,value};
	struct i2c_msg msg = {addr, 0, sizeof(buf), buf };
    	struct i2c_rdwr_ioctl_data rdwr = { &msg, 1 };

	if ( ioctl( fd, I2C_RDWR, &rdwr ) < 0 ){
		printf("Write error\n");
	}
}

void read_register(const int fd, unsigned char address, unsigned char reg, unsigned char *out)
{
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[2];
	unsigned char inbuf,outbuf;

	outbuf = reg;

	messages[0].addr = address;
	messages[0].flags = 0;
	messages[0].len = sizeof(inbuf);
	messages[0].buf = &outbuf;

	messages[1].addr = address;
	messages[1].flags = I2C_M_RD;
	messages[1].len = sizeof(outbuf);
	messages[1].buf = &inbuf;

	packets.msgs = messages;
	packets.nmsgs = 2;

	if (ioctl(fd,I2C_RDWR,&packets) < 0) {
		printf("Unable to read.\n");
	}

	*out = inbuf;
}


void print_binary(unsigned char ch, unsigned char *str)
{
	int i;

	printf("\n");
	for (i=0; i < 8; i++) {
		if ((0x01 << i) & ch) {
			printf("1");
		} else {
			printf("0");
		}
	}
	printf("\n%s = %d\n",str,ch);
}

double get_milli_seconds(void) {
	struct timeval  tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000;
	//return time(NULL);
}

int main(void)
{
	int fd;

	init_device(&fd);
	unsigned char latch;

	if (init_port(fd,PORT_A,PORT_DIR,&latch) != 2) {
		printf("Write error: Unable to configure device\n");
	}

	unsigned char data,prev_data,enc_dir;
	unsigned char pressed = FALSE;

	//Confgiure io and pullups
	write_register(fd, I2C_ADDR, 0x00, 0xFF);
	write_register(fd, I2C_ADDR, 0x0C, 0xFF);


	while (1){
		read_register(fd, I2C_ADDR, READ, &data);
		data =~ data;

		if ( MIDDLE_BUT_PRESSED(data) && pressed == FALSE ) {
			pressed = TRUE;
			printf("Middle Button Pressed\n");
		} else if ( LEFT_BUT_PRESSED(data) && pressed == FALSE ) {
			printf("Left Button Pressed\n");
			pressed = TRUE;
		} else if ( RIGHT_BUT_PRESSED(data) && pressed == FALSE ) {
			pressed = TRUE;
			printf("Right Button Pressed\n");
		} else if (pressed == TRUE && data == 0){ //Released
			pressed = FALSE;
		}

		if (prev_data != data) {
			if (prev_data == ENC_PORTA & ENC_PORTB &&
				data & ENC_PORTB) {
				enc_dir = ENC_DIR_CW;
				printf("CW\n");
			} else if ( prev_data & ENC_PORTB && data == ENC_PORTA & ENC_PORTB ){
				enc_dir = ENC_DIR_CCW;
				printf("CCW\n");
			}
		}

		prev_data = data;
		usleep(MSEC_TO_USEC(POLL_TIMEOUT_MS));
	}
	close(fd);
	return EXIT_SUCCESS;
}
