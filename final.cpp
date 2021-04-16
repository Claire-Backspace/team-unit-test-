#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

 //write(fd,buff,sizeof(buff));
using namespace cv;
using namespace std;
#define COM_NAME "/dev/ttyUSB0"

int set_opt(int,int,int,char,int);
int in = 0;

int print_px_value(Mat& im)
{
    int counter = 0;
    int rowNumber = im.rows;  //行数
    int colNumber = im.cols * im.channels();  //列数 x 通道数=每一行元素的个数

    //双重循环，遍历所有的像素值
    for (int i = 0; i < rowNumber; i++)  //行循环
    {
        uchar* data = im.ptr<uchar>(i);  //获取第i行的首地址
        for (int j = 0; j < colNumber; j++)   //列循环
        {
            //data[j] = data[j] / div * div + div / 2;
            //cout << (int)data[j] << endl;
            if( data[j] == 255) counter += 1;

        }  //行处理结束
    }
    //cout<<counter<<endl;
    return counter;//white
}



int main(){
    int fd;
    char len;
    char buffer1[512];
    char bufferget[512];
     unsigned char buff1[1];
char get[1];
    //buff[0] = 0x10;
    buff1[0] = '8';

    char *uart_out = "Please input,waiting\n";
    //int x,y;
    fd = open(COM_NAME, O_RDWR | O_NOCTTY |O_NDELAY);
    if(fd < 0){
        perror(COM_NAME);
                cout<<"!!!"<<endl;
                return -1;
            }
            memset(buffer1,0,sizeof(buffer1));
            set_opt(fd, 115200, 8, 'N', 1);
            int n = 0;//
             //read(fd, buff,sizeof(buffer));
            int in = 0;

            if(in == 0)
            {
                int  readByte = read(fd ,get, sizeof(bufferget));
                if(readByte != 0)
                {
                    in = 1;
                    //cout<<"in:"<<in<<endl;
                }

            }
            Mat src, templ, templgray;
            VideoCapture capture(2);

            templ = imread("/home/guoye/images/using/1ld.png");
            int counter = 0;

            while(in == 1)
            {
                capture >> src;
                Mat gray;
                //cvtCOLOR(src, gray, COLOR_BGR2GRAY);
                //cvtCOLOR(templ, templgray, COLOR_BGR2GRAY);

                namedWindow("modle", WINDOW_AUTOSIZE);
                //imshow("modle", templ);
                int h = src.rows - templ.rows + 1;
                int w = src.cols - templ.cols + 1;
                Mat result;
                result.create(w, h, CV_32FC1);
                matchTemplate(src, templ, result, TM_SQDIFF_NORMED);
                normalize(result, result, 1, 0, NORM_MINMAX, -1, Mat());
                double minValue = -1;
                double maxValue;
                Point minLocation;
                Point maxLocation;
                Point matchLocation;
                minMaxLoc(result, &minValue, &maxValue, &minLocation, &maxLocation, Mat());

                matchLocation = minLocation;

                rectangle(src, matchLocation, Point(matchLocation.x + templ.cols, matchLocation.y + templ.rows), Scalar(0, 255, 0), 2, 8, 0);
                Mat roi = src(Rect(matchLocation, Point(matchLocation.x + templ.cols, matchLocation.y + templ.rows)));
                Mat roihsv;
                cvtColor(roi, roihsv, COLOR_BGR2HSV);
                inRange(roihsv, Scalar(24, 67, 254), Scalar(73, 120, 255), roihsv);
                print_px_value(roihsv);

//                imshow("roi", roihsv);
//                imshow("原始图", src);
//                waitKey(1);

                if(counter == 0)
                {
                    if(print_px_value(roihsv) != 0)
                    {
                        for(int n = 0; n < 3; n++)
                        {
                            write(fd, buff1, sizeof(buffer1));
                            cout<<buff1<<endl;
                            sleep(1.75);
                            counter++;
                        }

                    }


                }

            }


//            while(1){
//        //             while((len = read(fd, buffer, 512))>0){
//        //            buffer[len+1] = '\0';
//        //              write(fd,buffer,strlen(buffer));
//                      write(fd,buff,sizeof(buff));
//                      x=sizeof(buff);
//                    len=read(fd,buffer,sizeof(buffer));
//                    y=strlen(buffer);
//                    memset(buffer,0,strlen(buffer));
//                    len = 0;
//                }
            return 0 ;
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
