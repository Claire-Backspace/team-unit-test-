#include <iostream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "com.h"

#define BUFFER_SIZE 30

using namespace std;
using namespace cv;
char *pstr[]={"NO1\n","NO2\n","NO3\n","NO4\n"};


int main(int argc, char *argv[])
{
    int fd;
    int i;
    char read_buffer[BUFFER_SIZE];
    int read_buffer_size;
    fd = open_port(0);
    if(set_com_config(fd, 115200, 8, 'N', 1) < 0) /* 配置串口 */
       {
           perror("set_com_config");
           return 1;
       }
    while(1)
    {
        for(i = 0; i < 4; i++)
        {
            write(fd, pstr[i], strlen(pstr[i]));
            sleep(1);
            while(!read_buffer_size);
            printf("read[%d][%s]\n",  read_buffer_size,read_buffer);
            //write(fd, 1, 8 );
        }
    }
    close(fd);
    return 0;
}
