// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

typedef unsigned char byte;

class robot{
    public: 
        robot();
        ~robot();
        bool com_start(char * portname);
        bool com_end();
        bool init();
        int robot_read();
        void robot_write(int value);
        int byteToInt(byte * buffer, int length_buffer);

    private:
        struct termios tty;
        int serial_port;
        byte * read_buf= new byte[10]{0};
        byte * write_buf = new byte[256]{0};

        int read_result;
    
};
