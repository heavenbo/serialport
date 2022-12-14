#include <ros/ros.h>
#include <serial/serial.h> //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
serial::Serial ser; //声明串口对象
//回调函数
void write_callback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data); //发送串口数据
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_example_node");
    //声明节点句柄
    ros::NodeHandle nh;
    //订阅主题，并配置回调函数
    // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    // //发布主题
    // ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyACM1");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    //检测串口是否已经打开，并给出提示信息
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    //指定循环的频率
    ros::Rate loop_rate(50);
    uint8_t buffer_read[1];
    uint8_t buffer[1];
    buffer[0] = 2;
    int i = 0;
    while (ros::ok())
    {
        int n = ser.read(buffer_read, 1);
        if (n != 0)
        {
            ROS_INFO("%d", buffer_read[0] & 0xff);
        }
        if (i < 1)
        {
            ser.write(buffer, 1);
            ROS_INFO("%d  write", i);
            i++;
        }
        //处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();
        loop_rate.sleep();
    }
}
