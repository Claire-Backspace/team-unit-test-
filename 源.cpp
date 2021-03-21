
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <stdio.h>
#include <fcntl.h>  //文件控制定义
#include <termios.h>   //POSIX终端控制定义
#include <unistd.h>    //UNIX标准定义
#include <errno.h>     //ERROR数字定义
#include <sys/select.h>
using namespace cv;
using namespace std;
int capture_Value = 0;      //相机的默认值
int t1, t2, t3, FPS;
VideoCapture capture(1);
int main()
{
    //串口部分
    int fd;
    struct termios options, newstate;
    char* buf = new char[8];//分配内存空间
    sprintf(buf, "%s%d%s%d", "S", x, ",", y);
    //sprintf(buf2,"%s%d%s%d", "Y", ":", y);
    fd = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);    //打开串口
    if (fd == -1)
        printf("can not open the COM!\n");
    else
        printf("open COM ok!\n");
    /*判断是否是终端设备
    if(isatty(STDIN_FILENO) == 0)
      printf("不是终端设备\n");
    else
      printf("是终端设备\n");
    */
    if (fcntl(fd, F_SETFL, 0) < 0) //改为阻塞模式
        printf("fcntl failed\n");
    else
        printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
    tcgetattr(fd, &options);
    //设置波特率
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    //获取波特率
   // tcgetattr(fd, &newstate);
    //    baud_rate_i=cfgetispeed(&newstate);
    //    baud_rate_o=cfgetospeed(&newstate);
        //串口设置
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;//设置无奇偶校验位，N
    options.c_cflag &= ~CSTOPB; //设置停止位1
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; //设置数据位
    options.c_cc[VTIME] = 0;//阻塞模式的设置
    options.c_cc[VMIN] = 1;
    //激活新配置
    tcsetattr(fd, TCSANOW, &options);
    //输出波特率
    //printf("输入波特率为%d，输出波特率为%d\n" , baud_rate_i, baud_rate_o);
   //配置其他选项
//    SerialPortSettings.c_cflag &= ~CRTSCTS;     //关闭基于硬件的流量控制
//    SerialPortSettings.c_cflag |= CREAD | CLOCAL;   //打开串口（CREAD）的接收器
//    SerialPortSettings.c_iflag &= ~(IXON|IXOFF|IXANY);  //关闭基于软件的流量控制（XON / XOFF）
//    SerialPortSettings.c_iflag &= ~(ICANON|ECHO|ECHOE|ISIG);    //设置操作模式
    ```
        if (fcntl(fd, F_SETFL, 0) < 0) //改为阻塞模式
            printf("fcntl failed\\n");
        else
            printf("fcntl=%d\\n", fcntl(fd, F_SETFL, 0));
    //获取串口
    tcgetattr(fd, &options);
    //设置波特率
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    //串口设置
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;//设置无奇偶校验位，N
    options.c_cflag &= ~CSTOPB; //设置停止位1
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; //设置数据位
    options.c_cc[VTIME] = 0;//阻塞模式的设置
    options.c_cc[VMIN] = 1;
    //激活新配置
    tcsetattr(fd, TCSANOW, &options);

    cout << buf[0] << endl;
    sprintf(buf, "%c", op);
    write(fd, buf, sizeof(buf));
    tcflush(fd, TCIOFLUSH);//清除所有正在发送的I/O数据

    cd = read(fd, buff, sizeof(buff));
    if (cd == -1)
        cout << cd << endl;
    sleep(0.5);
    return cd;

}


