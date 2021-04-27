// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

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

int main() {
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/ttyUSB0", O_RDWR);

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
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
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  // Write to serial port
  //unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
  //write(serial_port, "Hello, world!", sizeof(msg));

  // Allocate memory for read buffer, set size according to your needs
  char read_buf [256];
  unsigned char buff1[] = "8";//cross
  unsigned char buff2[] = "9";//stop

  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
      return 1;
  }

  int in = 0;
int sec = 0;
  int thir = 0;
  int fourth = 0;


  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
  // print it to the screen like this!)
  switch(read_buf[0])
  {
  case'1':
      in = 1;
      cout<<"in:"<<in<<endl;
      break;
  }



  Mat src, templ;
  Mat templ2, templ3, templ4;


  VideoCapture capture(0);

  templ = imread("/home/guoye/images/using/1ld.png");
  templ2 = imread("/home/guoye/images/using/2ld.png");
  templ3 = imread("/home/guoye/images/using/3ld.png");
  templ4 = imread("/home/guoye/images/using/4ld.png");

  while(in)
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

          if(print_px_value(roihsv) != 0)
          {
              write(serial_port, buff1, sizeof(1));
              cout<<"1:"<<buff1[0]<<endl;
              sleep(2);
              write(serial_port, buff2, sizeof(1));
              cout<<"1:"<<buff2[0]<<endl;
              in--;
              sec++;
          }
  }

  while(sec)
  {
      capture >> src;
      Mat gray;
      //cvtCOLOR(src, gray, COLOR_BGR2GRAY);
      //cvtCOLOR(templ, templgray, COLOR_BGR2GRAY);

      namedWindow("modle", WINDOW_AUTOSIZE);
      //imshow("modle", templ);
      int h = src.rows - templ2.rows + 1;
      int w = src.cols - templ2.cols + 1;
      Mat result2;
      result2.create(w, h, CV_32FC1);
      matchTemplate(src, templ2, result2, TM_SQDIFF_NORMED);
      normalize(result2, result2, 1, 0, NORM_MINMAX, -1, Mat());
      double minValue = -1;
      double maxValue;
      Point minLocation;
      Point maxLocation;
      Point matchLocation;
      minMaxLoc(result2, &minValue, &maxValue, &minLocation, &maxLocation, Mat());

      matchLocation = minLocation;

      rectangle(src, matchLocation, Point(matchLocation.x + templ2.cols, matchLocation.y + templ2.rows), Scalar(0, 255, 0), 2, 8, 0);
      Mat roi = src(Rect(matchLocation, Point(matchLocation.x + templ2.cols, matchLocation.y + templ2.rows)));
      Mat roihsv;
      cvtColor(roi, roihsv, COLOR_BGR2HSV);
      inRange(roihsv, Scalar(24, 67, 254), Scalar(73, 120, 255), roihsv);
      print_px_value(roihsv);

////                imshow("roi", roihsv);
////                imshow("原始图", src);
////                waitKey(1);

          if(print_px_value(roihsv) != 0)
          {
              write(serial_port, buff1, sizeof(1));
              cout<<"2:"<<buff1<<endl;
              sleep(2);
              write(serial_port, buff2, sizeof(1));
              cout<<"2:"<<buff2<<endl;
              sec--;
              thir++;
              break;
          }

  }


  while(thir)
  {
      capture >> src;
      Mat gray;
      //cvtCOLOR(src, gray, COLOR_BGR2GRAY);
      //cvtCOLOR(templ, templgray, COLOR_BGR2GRAY);

      namedWindow("modle", WINDOW_AUTOSIZE);
      //imshow("modle", templ);
      int h = src.rows - templ3.rows + 1;
      int w = src.cols - templ3.cols + 1;
      Mat result3;
      result3.create(w, h, CV_32FC1);
      matchTemplate(src, templ3, result3, TM_SQDIFF_NORMED);
      normalize(result3, result3, 1, 0, NORM_MINMAX, -1, Mat());
      double minValue = -1;
      double maxValue;
      Point minLocation;
      Point maxLocation;
      Point matchLocation;
      minMaxLoc(result3, &minValue, &maxValue, &minLocation, &maxLocation, Mat());

      matchLocation = minLocation;

      rectangle(src, matchLocation, Point(matchLocation.x + templ3.cols, matchLocation.y + templ3.rows), Scalar(0, 255, 0), 2, 8, 0);
      Mat roi = src(Rect(matchLocation, Point(matchLocation.x + templ3.cols, matchLocation.y + templ3.rows)));
      Mat roihsv;
      cvtColor(roi, roihsv, COLOR_BGR2HSV);
      inRange(roihsv, Scalar(24, 67, 254), Scalar(73, 120, 255), roihsv);
      print_px_value(roihsv);

//                imshow("roi", roihsv);
//                imshow("原始图", src);
//                waitKey(1);

          if(print_px_value(roihsv) != 0)
          {
              write(serial_port, buff1, sizeof(1));
              cout<<"3:"<<buff1<<endl;
              sleep(1);
              write(serial_port, buff2, sizeof(1));
              cout<<"3:"<<buff2<<endl;
              thir--;
              fourth++;
              break;
          }
  }

  while(fourth)
  {
      capture >> src;
      Mat gray;
      //cvtCOLOR(src, gray, COLOR_BGR2GRAY);
      //cvtCOLOR(templ, templgray, COLOR_BGR2GRAY);

      namedWindow("modle", WINDOW_AUTOSIZE);
      //imshow("modle", templ);
      int h = src.rows - templ4.rows + 1;
      int w = src.cols - templ4.cols + 1;
      Mat result4;
      result4.create(w, h, CV_32FC1);
      matchTemplate(src, templ4, result4, TM_SQDIFF_NORMED);
      normalize(result4, result4, 1, 0, NORM_MINMAX, -1, Mat());
      double minValue = -1;
      double maxValue;
      Point minLocation;
      Point maxLocation;
      Point matchLocation;
      minMaxLoc(result4, &minValue, &maxValue, &minLocation, &maxLocation, Mat());

      matchLocation = minLocation;

      rectangle(src, matchLocation, Point(matchLocation.x + templ4.cols, matchLocation.y + templ4.rows), Scalar(0, 255, 0), 2, 8, 0);
      Mat roi = src(Rect(matchLocation, Point(matchLocation.x + templ4.cols, matchLocation.y + templ4.rows)));
      Mat roihsv;
      cvtColor(roi, roihsv, COLOR_BGR2HSV);
      inRange(roihsv, Scalar(24, 67, 254), Scalar(73, 120, 255), roihsv);
      print_px_value(roihsv);

//                imshow("roi", roihsv);
//                imshow("原始图", src);
//                waitKey(1);

          if(print_px_value(roihsv) != 0)
          {
              write(serial_port, buff1, sizeof(1));
              cout<<"4:"<<buff1<<endl;
              sleep(1);
              write(serial_port, buff2, sizeof(1));
              cout<<"4:"<<buff2<<endl;
              fourth--;
              break;
          }

  }

  close(serial_port);
  return main(); // success
}
