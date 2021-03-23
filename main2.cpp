#include"com2.h"

//#include <stdio.h>
//#include <string.h>
//#include <sys/types.h>
//#include <errno.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <unistd.h>
//#include <termios.h>
//#include <stdlib.h>
#include <iostream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;



int main()
{
    int fd;
       int nread,i;
       char buff[]="Hello\n";
       if((fd=open_port(fd,1)) == 0)
       {
           perror("open_port error");
           return (fd);
       }
       if((i=set_opt(fd,115200,8,'N',1)) == 0)
       {
           perror("set_opt error");
           return fd;
       }
       printf("fd=%d\n",fd);
   //    fd=3;
       nread=read(fd,buff,8);
       printf("nread=%d,%s\n",nread,buff);
       close(fd);
       return 0;
}
