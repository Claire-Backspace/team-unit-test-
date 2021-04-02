#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


 //write(fd,buff,sizeof(buff));

using namespace std;
#define COM_NAME "/dev/ttyUSB0"

int set_opt(int,int,int,char,int);
int main(){
    int fd;
    char len;
    char buffer[512];
     unsigned char buff[1] ;
    //buff[0] = 0x10;
    buff[0] = '8';

    char *uart_out = "Please input,waiting\n";
    int x,y;
    fd = open(COM_NAME, O_RDWR | O_NOCTTY |O_NDELAY);
    if(fd < 0){
        perror(COM_NAME);
                cout<<"!!!"<<endl;
                return -1;
            }
            memset(buffer,0,sizeof(buffer));
            set_opt(fd, 115200, 8, 'N', 1);
            int n = 0;//
            while(n < 1)
            {
                read(fd, buff,sizeof(buffer));

                write(fd, buff, sizeof(buffer));
//                if(read != 0)
                cout<<buff<<endl;
                //return(fd, sign, sizeof(sign));
                n++;
            }
            //return(fd, buff, sizeof(buff))

        //    write(fd,uart_out, 1);
            while(1){
        //             while((len = read(fd, buffer, 512))>0){
        //            buffer[len+1] = '\0';
        //              write(fd,buffer,strlen(buffer));
                      write(fd,buff,sizeof(buff));
                      x=sizeof(buff);
                    len=read(fd,buffer,sizeof(buffer));
                    y=strlen(buffer);
                    memset(buffer,0,strlen(buffer));
                    len = 0;
////                    ssize_t read(int fd, void *buf, size_t count);
////                    返回值：成功返回读取的字节数，出错返回-1并设置errno，如果在调read之前已到达文件末尾，则这次read返回0
//                    //ssize_t write(int fd, const void *buf, size_t count);
//                    //返回值：成功返回写入的字节数，出错返回-1并设置errno

                }
}

int set_opt(int fd,int comspeed,int comBits,char comEvent,int comStop){
    struct termios newtio,oldtio;
    if( tcgetattr(fd,&oldtio) !=0){
       perror("SetupSerial 1");
       return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch(comBits){
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch (comEvent) {
    case 'N':
        newtio.c_cflag &= PARENB;
               break;
           }
           switch (comspeed) {
           case 115200:
               cfsetispeed(&newtio, B115200);
               cfsetospeed(&newtio, B115200);
               break;
           default:
               cfsetispeed(&newtio, B9600);
               cfsetospeed(&newtio, B9600);
               break;
           }
              if(comStop == 1){
                  newtio.c_cflag &= ~CSTOPB;
                  newtio.c_cc[VTIME] = 100;
                  newtio.c_cc[VMIN] = 0;
                  tcflush(fd,TCIFLUSH);
              }
              if((tcsetattr(fd,TCSANOW,&newtio))!=0){
                  perror("comset error");
                  return -1;
              }else{
                  cout<<"set done!"<<endl;
              }
              return 0;
          }
