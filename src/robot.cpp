#include "robot/robot.h"
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <malloc.h>
#include <string.h>

robot::robot()
{

}
robot::~robot()
{

}
bool robot::com_start(char * portname)
{
    serial_port = open(portname, O_RDWR);
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));

    // Create new termios struc, we call it 'tty' for convention
    memset(&tty, 0, sizeof tty);

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
    
    return false;
}
bool robot::com_end()
{
    close(serial_port);
    return false;
}
void robot::robot_write(int value)
{
    char buf[20]="1"; //write is 1
    char ch_value[10]={0};
    //debug //cast from integer to byte array
    sprintf(ch_value,"%d", value);
    //printf("position target value");
    //printf(ch_value);
    //snprintf(ch_value, "%d", value);
    strcat(buf,ch_value);
    //printf(buf);
    //write(serial_port, buf, strlen(buf)+1);
	//std::cout << "command" << buf << "is sent" << std::endl;
    //printf("Write sent");
    write(serial_port,buf, strlen(ch_value) + 1);

}
int robot::robot_read()
{
    char buf[20]="2"; //read is 2;
	//sleep(2);
    write(serial_port, buf, strlen(buf)+1);
    usleep(700000);
	//debug //std::cout << "command" << buf << "is sent"<< std::endl;
	//usleep(0);
    //usleep(10);
    int num_bytes = read(serial_port, read_buf, sizeof(read_buf));
	//std::cout << read_buf << std::endl;
	//std::cout << strlen((char *)read_buf) << std::endl;
    if (num_bytes < 0) {
        printf("Error reading: %s", strerror(errno));
    }
    //printf("Read %i bytes. Received message: %s\n", num_bytes, read_buf);
    //read_result=atoi(read_buf);
    read_result=byteToInt(read_buf,strlen((char *)read_buf));
    //printf("after change to INT %i\n",read_result);

    for (int i=0;i<malloc_usable_size(read_buf);i++)
    {
        read_buf[i]=NULL;
    }

	//byte buffer cast to integer
    return read_result;
}

int robot::byteToInt(byte * buffer, int buffer_size)
{
    int minus_sign = -1;
	int total=0;
	int exponential = 0;
	for (int i = buffer_size - 1; i >= 0; i--) 
	{
		if (static_cast<int>(read_buf[i]) >= 48)
		{
			total = total + (static_cast<int>(read_buf[i]) - 48)*pow(10, exponential);
			//printf("%i",total);
            exponential++;
		}
		else if (static_cast<int>(read_buf[i]) == 45)
		{
			total = total*minus_sign;
		}
		else
		{
			return total;
		}
	}
	return total;
}
