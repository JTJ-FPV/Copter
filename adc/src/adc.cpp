// 加载 打开串口并读取显示 用到的库
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// 加载 实现显示16进制 用到的库
#include <sstream>
#include <iomanip>
#include<cstdio>


// 加载 使用延时时会 用到的库
#include <thread>
#include <chrono>


int main(int argc, char  **argv)
{   
    ros::init(argc, argv, "adc");

    ros::NodeHandle nh;
    
    ros::Publisher adc_pub = nh.advertise<std_msgs::Float64>("/adc/value", 1);

    ros::Rate rate(10);

    int fd;
    struct termios tty;
    char buffer[1024];
    ssize_t bytesRead;

    // 打开 /dev/ttyACM0 串口端口
    fd = open("/dev/ttyACM0", O_RDONLY | O_NOCTTY);

    if (fd == -1)
    {
        std::cerr << "Error opening serial port." << std::endl;
        return 1;
    }

    // 配置串口参数
    if (tcgetattr(fd, &tty) != 0)
    {
        std::cerr << "Error getting serial port attributes." << std::endl;
        close(fd);
        return 1;
    }

    // 配置波特率
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(INPCK | ISTRIP);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error setting serial port attributes." << std::endl;
        close(fd);
        return 1;
    }

    std::stringstream sstream;
    double num;
    std_msgs::Float64 receive_num;
    // 读取串口数据并输出到控制台
    while (ros::ok())
    {
        bytesRead = read(fd, buffer, sizeof(buffer));

        if (bytesRead > 0)
        {
            for(int i=0; i<bytesRead; i++) {
              sstream << buffer[i];
            //   std::cout << buffer[i];  // 原始数据显示
            }
            std::cout << std::endl;
            // 转数字
            // sstream >> num;
            num = atof(buffer);
            // std::cout << "the num is " << num << std::endl;
            num /= 200.0;
            if(num <= 0.3)
                ROS_ERROR("num is low 0.03");
            // ROS_INFO_STREAM("the receive number is " << num);
            receive_num.data = num;
            adc_pub.publish(receive_num);
        }
        rate.sleep();
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ros::spinOnce();
    }

    // 关闭串口
    close(fd);
    return 0;
}