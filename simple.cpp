#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
using namespace std;
using namespace cv;
#define COM_NAME "/dev/ttyUSB0"
int count = 0;
int set_opt(int,int,int,char,int);
Point2f gc[1];
float gr[1];
int cc;
int ss;
char buff[1];
int counter = 0;


int jugment1()
{
    VideoCapture capture (0);
    Mat src, hsv, thr, thred;
    Mat dst_green, dst_yellow;
    //while(counter < 6)
    //{ counter ++;
    while(counter < 30)
    {
        counter ++;

        capture >> src;
        //cvtColor(src, thr, COLOR_BGR2GRAY);
        cvtColor(src, hsv, COLOR_BGR2HSV);
        //inRange(hsv, Scalar(55, 184, 170), Scalar(89, 245, 255), dst_green);
        inRange(hsv, Scalar(55, 183, 50), Scalar(70, 255, 170), dst_green);//green2
        //inRange(hsv, Scalar(17, 125, 132), Scalar(32, 245, 255), dst_yellow);

        vector<Vec3f>circles;
        HoughCircles(dst_green, circles, HOUGH_GRADIENT, 1.55, 15, 100, 70, 10, 40);
        for (size_t i = 0; i < circles.size(); i++)
        {
            Vec3f c = circles[i];
            //yuan
            //circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

//            while(i < 1)
//            {
                gc[i] = center;
                gr[i] = radius;
        }

        if(gr[0] == 0)
        {
            cc = 1;
            cout<<"cc:"<< cc <<endl;
        }
        else if(gr[0] != 0)
        {
            ss = 1;
            cout<<"ss:"<< ss <<endl;
        }
}
//}
    return cc;
}


int jugment2()
{
    VideoCapture capture (0);
    Mat src, hsv, thr, thred;
    Mat dst_green, dst_yellow;
    int m = 0;
    //while(counter < 6)
    //{ counter ++;
    while(counter < 30)
    {
        counter ++;

        capture >> src;
        //cvtColor(src, thr, COLOR_BGR2GRAY);
        cvtColor(src, hsv, COLOR_BGR2HSV);
        //inRange(hsv, Scalar(55, 184, 170), Scalar(89, 245, 255), dst_green);
        inRange(hsv, Scalar(55, 183, 50), Scalar(70, 255, 170), dst_green);//green2
        inRange(hsv, Scalar(17, 125, 132), Scalar(32, 245, 255), dst_yellow);
        //threshold(thr, thred, 30, 255, 0);
        //threshold(thr, thred, 100, 255, 1);
        //blur(thred, thred, Size(15, 15));
        //imshow("thred", thred);


//        Point2f gc[1];
//        float gr[1];
//        Point2f yc[1];
//        float yr[1];
        vector<Vec3f>circles;
        HoughCircles(dst_green, circles, HOUGH_GRADIENT, 1.55, 15, 100, 70, 10, 40);
        for (size_t i = 0; i < circles.size(); i++)
        {
            Vec3f c = circles[i];
            //yuan
            //circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

//            while(i < 1)
//            {
                gc[i] = center;
                gr[i] = radius;





//            }
        }

        if(gr[0] == 0)
        {
            cc = 1;
            cout<<"cc:"<< cc <<endl;
        }
        else if(gr[0] != 0)
        {
            ss = 1;
            cout<<"ss:"<< ss <<endl;
        }
        cout<<"gr:"<< gr[0] <<endl;
        cout<<"ss:"<< ss <<endl;
        cout<<"cc:"<< cc <<endl;
//        if(gr[0] > 0)
//        {
//            ss = 1;
//        }
//        else if(gr[0] == 0)
//        {
//            cc = 1;
//        }

//        imshow("capture",src);
//        //imshow("hsv", tog);
//        char(key)=(char)waitKey(1);


//        if(key==27)
//            break;
                //return 0;
    }
    //return cc;



    //}
                return ss;
}

//////////////////////////////////////////////////////////////////////////////

int main()
{
    int fd;
    char len;
    char buffer1[512];
    char buffer2[512];
     unsigned char buff1[1];
      unsigned char buff2[1];
    //buff[0] = 0x10;
    buff1[0] = '8';//go
    buff2[0] = '9';//stop


    char *uart_out = "Please input,waiting\n";
    int x,y;
    fd = open(COM_NAME, O_RDWR | O_NOCTTY |O_NDELAY);
    if(fd < 0){
        perror(COM_NAME);
                cout<<"!!!"<<endl;
                return -1;
            }
            memset(buffer1,0,sizeof(buffer1));
            memset(buffer2,0,sizeof(buffer2));
            set_opt(fd, 115200, 8, 'N', 1);
            //buff[0] = '9';
            //write(fd, buff, sizeof(buffer));
            jugment1();
            jugment2();
            //int n = 0;//
//            while(n < 1)
//            {
//                read(fd, buff,sizeof(buffer));

//                write(fd, buff, sizeof(buffer));
////                if(read != 0)
//                cout<<buff<<endl;
//                //return(fd, sign, sizeof(sign));
//                n++;
//            }
            //return(fd, buff, sizeof(buff))

        //    write(fd,uart_out, 1);


            cout<<"jugment1"<<jugment1()<<endl;
            cout<<"jugment2"<<jugment2()<<endl;
            cout<<"cc:"<<cc<<endl;
            cout<<"ss:"<<ss<<endl;
            //while(){
            for(int n = 0; n < 1; n++)
            {
                if(jugment1() == 1 && jugment2() == 1)//cc = 1, ss = 1
                {
                    write(fd, buff1, sizeof(buffer1));
                    cout<<buff1<<endl;
                }
                else if(jugment1() == 0 && jugment2() == 0)//cc = 0, ss = 0
                {
                    write(fd, buff1, sizeof(buffer1));
                    cout<<buff1<<endl;
                }
                else if(jugment1() == 1 && jugment2() == 0)//cc = 0, ss = 0
                {
                    write(fd, buff1, sizeof(buffer1));
                    cout<<buff1<<endl;
                }
                else if(jugment1() == 0 && jugment2() == 1)//cc = 0, ss = 0
                {
                    write(fd, buff2, sizeof(buffer2));
                    cout<<buff2<<endl;
                }
            }
            //}


            while(1){
        //             while((len = read(fd, buffer, 512))>0){
        //            buffer[len+1] = '\0';
        //              write(fd,buffer,strlen(buffer));
                      write(fd,buff1,sizeof(buff1));
                      x=sizeof(buff1);
                    len=read(fd,buffer1,sizeof(buffer1));
                    y=strlen(buffer1);
                    memset(buffer1,0,strlen(buffer1));
                    len = 0;

                    write(fd,buff2,sizeof(buff2));
                    x=sizeof(buff2);
                  len=read(fd,buffer2,sizeof(buffer2));
                  y=strlen(buffer2);
                  memset(buffer2,0,strlen(buffer2));
                  len = 0;
//////                    ssize_t read(int fd, void *buf, size_t count);
//////                    返回值：成功返回读取的字节数，出错返回-1并设置errno，如果在调read之前已到达文件末尾，则这次read返回0
////                    //ssize_t write(int fd, const void *buf, size_t count);
////                    //返回值：成功返回写入的字节数，出错返回-1并设置errno

//                }


        //cout<<gr[0]<<endl;

        //if(radius[0] > 1)
//        vector<Vec3f>circles1;
//        HoughCircles(dst_yellow, circles1, HOUGH_GRADIENT, 1.55, 15, 100, 42, 10, 30);
//        for (size_t i = 0; i < circles1.size(); i++)
//        {
//            Vec3f d = circles1[i];
//            //yuan
//            //circle(src, Point(c[0], c[1]), c[2], Scalar(77, 255, 255), 2, LINE_AA);
//            circle(src, Point(d[0], d[1]), d[2], Scalar(0, 255, 0), 2, LINE_AA);
//            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//            int radius = cvRound(circles[i][2]);
//            //yc[i] = center;
//            //yr[i] = radius;
//        }
        //cout<<src.cols<<endl;//lie640
        //cout<<src.rows<<endl;//hang360

        //cv::Mat combine = cv::Mat::zeros(w, h, src.type());
        //Mat ROI = src(Range(gc[0].x - gr[0], gc[0].y - gr[0]), Range(gc[0].x + gr[0], gc[0].y + gr[0]));
        //Mat ROI = src(Range(1, 360), Range(1, 640));

        //int rowNumber = src.rows;  //行数
          //int colNumber = src.cols * im.channels();  //列数 x 通道数=每一行元素的个数

//        int i = gc[0].y - gr[0];
//        int rowNumber = gc[0].y + gr[0];
//        int j = gc[0].x - gr[0];
//        int colNumber = gc[0].x + gr[0];
//          //双重循环，遍历像素值
//        int counter = 0;
//          for ( ; i < rowNumber; i++)  //行循环
//          {
//              uchar* data = src.ptr<uchar>(i);  //获取第i行的首地址
//              for ( ; j < colNumber; j++)   //列循环
//              {
//                  //data[j] = data[j] / div * div + div / 2;
//                  //cout << (int)data[j] << endl;
//                  if (data[j] == 0) counter += 1;

//              }  //行处理结束
//          }
//          cout<<counter<<endl;
        //Mat tog = dst_green&&dst_yellow;
    return 0;
}
            return 0;
}


int set_opt(int fd,int comspeed,int comBits,char comEvent,int comStop)
{
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



//#include<iostream>
//#include<opencv2/opencv.hpp>
//#include<opencv2/highgui/highgui.hpp>
//#include<opencv2/imgproc/imgproc.hpp>
//#include<opencv2/core/core.hpp>
//#include<cmath>
//vector<vector<Point>>contours;
//vector<Vec4i>hierarchy;
//using namespace cv;
//using namespace std;

//const int max_value_H = 360 / 2;
//const int max_value = 255;
//int low_H = 0, low_L = 0, low_S = 0;
//int high_H = max_value_H, high_L = max_value, high_S = max_value;

//String HLSname = "HLS";
//Mat clo_HLS, clo_threshold;

//static void on_low_H_thresh_trackbar(int, void*)
//{
//    low_H = min(high_H - 1, low_H);
//    setTrackbarPos("Low H", HLSname, low_H);
//}
//static void on_high_H_thresh_trackbar(int, void*)
//{
//    high_H = max(high_H, low_H + 1);
//    setTrackbarPos("High H", HLSname, high_H);
//}
//static void on_low_L_thresh_trackbar(int, void*)
//{
//    low_L = min(high_L - 1, low_L);
//    setTrackbarPos("Low L", HLSname, low_L);
//}
//static void on_high_L_thresh_trackbar(int, void*)
//{
//    high_L = max(high_L, low_L + 1);
//    setTrackbarPos("High L", HLSname, high_L);
//}
//static void on_low_S_thresh_trackbar(int, void*)
//{
//    low_S = min(high_S - 1, low_S);
//    setTrackbarPos("Low V", HLSname, low_S);
//}
//static void on_high_S_thresh_trackbar(int, void*)
//{
//    high_S = max(high_S, low_S + 1);
//    setTrackbarPos("High S", HLSname, high_S);
//}

//int thresholdvalue = 100;
//int thresholdtype = 3;

//Mat dst;
//void threshold( int, void*)
//{
//    Mat gray;

//        threshold(gray, dst, thresholdvalue, 255, thresholdtype);

//}

//int main()
//{
//    VideoCapture capture(2);
//    Mat src, gray;
//    while (1)
//    {
//        capture >> src;
//        cvtColor(src, gray, COLOR_BGR2GRAY);
//        namedWindow("WINDOW_NAME", WINDOW_AUTOSIZE);
//        createTrackbar("modle", "WINDOW_NAME", &thresholdtype, 4, threshold);
//        createTrackbar("number", "WINDOW_NAME", &thresholdvalue, 255, threshold);
//        threshold(gray, dst, thresholdvalue, 255, thresholdtype);
//        imshow("capture", src);
//        imshow("WINDOW_NAME", dst);
//            char(key) = (char)waitKey(1);
//        if (key == 27)
//            break;
//    }
//    return 0;
//}
