#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <string.h>
using namespace std;
using namespace cv;
int port(char op)
{
//串口部分代码串口部分代码串口部分代码串口部分代码串口部分代码串口部分代码
int fd;
int cd;
struct termios options, newstate;
char buff[1];
char * buf = new char[5];//分配内存空间
fd=open("/dev/ttyUSB0", O_RDWR|O_NONBLOCK|O_NOCTTY|O_NDELAY);    //打开串口
if(fd==-1)
printf("can not open the COM!\n");
else
printf("open COMß ok!\n");
     if(fcntl(fd, F_SETFL, 0) <0 ) //改为阻塞模式
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
 options.c_cc[VTIME]=0;//阻塞模式的设置
 options.c_cc[VMIN]=1;
 //激活新配置
 tcsetattr(fd, TCSANOW, &options);

 cout<<buf[0]<<endl;
 sprintf(buf, "%c",op);
  write(fd, buf, sizeof(buf));
  tcflush(fd, TCIOFLUSH);//清除所有正在发送的I/O数据

     cd= read(fd,buff,sizeof(buff));
      if(cd==-1)
      cout<<cd<<endl;
      sleep(0.5);

  return cd;
}
