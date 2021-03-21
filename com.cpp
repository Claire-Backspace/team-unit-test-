
#include "com.h"

int open_port(int com_port)
{
    int fd;
    char *dev[] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};

    printf("open dev [%s]\n",dev[com_port]);

    fd = open(dev[com_port], O_RDWR|O_NOCTTY);
    if (fd < 0)
       {
           perror("open serial port");
           return(-1);
       }

       //恢复串口为阻塞状态
       //非阻塞：fcntl(fd,F_SETFL,FNDELAY)
       //阻塞：fcntl(fd,F_SETFL,0)
       if (fcntl(fd, F_SETFL, 0) < 0)
       {
           perror("fcntl F_SETFL\n");
       }
       /*测试是否为终端设备*/
       if (isatty(STDIN_FILENO) == 0)
       {
           perror("standard input is not a terminal device");
       }

       return fd;
   }

/************串口配置***************/
int set_com_config(int fd,int baud_rate,int data_bits, char parity, int stop_bits)
{
    baud_rate = 115200;
    stop_bits = 1;
    data_bits = 8;
    parity = 'N';
    struct termios opt;
    int speed;
    if(tcgetattr(fd, &opt) != 0)
    {
        perror("tcgetattr");
        return -1;
    }
    /*处理未接收字符*/
        tcflush(fd, TCIFLUSH);

        /*设置等待时间和最小接收字符*/
        opt.c_cc[VTIME]  = 11;
        opt.c_cc[VMIN] = 0;

        /*关闭串口回显*/
        opt.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHOK|ECHONL|NOFLSH);

        /*激活新配置*/
        if((tcsetattr(fd, TCSANOW, &opt)) != 0)
        {
            perror("tcsetattr");
            return -1;
        }
        return 0;
}
