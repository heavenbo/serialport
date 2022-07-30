#include <serial/serial.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include "../lib/EDLib.h"
#include "../lib/identify.hpp"

bool isread = false;
bool isdetect = false;
int main(int argv, char **argc)
{
    std::string yaml_path = "/home/boyheaven/Vscode/work/serialPort/src/serialPort/config/point.yaml";
    std::string mp3_path = "/home/boyheaven/Vscode/work/serialPort/src/serialPort/mp3/mplay.sh ";

    cv::VideoCapture cap;
    YAML::Node point_yaml = YAML::LoadFile(yaml_path);
//     ros::init(argv, argc, "fir");
//     ros::NodeHandle n;
//     ros::Rate rate(20);
    serial::Serial ser;
    identify::color a = identify::nocolor;
    identify::shape b = identify::none;
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
    uint8_t buffer_read[1];
    int state = 2;
    int nopoint = 0;
    int i = 0;
    buffer_read[0]=3;
    ser.write(buffer_read,1);
    buffer_read[0]=0;
    while (1)
    {
        int n = ser.read(buffer_read, 1);
        if (buffer_read[0] == 100)
        {
            state = 0;
            nopoint = 0;
            buffer_read[0] = 0;
            ROS_INFO("Restart!");
        }
        if (buffer_read[0] == 99)
        {
            state = 1;
            ROS_INFO("set state:");
        }
        if (state == 0)
        {
            if (buffer_read[0] < 13 && buffer_read[0] > 0 && nopoint == 0)
            {
                int i = buffer_read[0];
                point_yaml["first_point"] = i;
                std::ofstream fot(yaml_path);
                fot << point_yaml;
                fot.close();
                nopoint = 1;
                isread = true;
            }
            if (isread)
            {
                buffer_read[0] = 0;
                isread = false;
            }
            if (buffer_read[0] < 13 && buffer_read[0] > 0 && nopoint == 1)
            {
                int i = buffer_read[0];
                point_yaml["second_point"] = i;
                std::ofstream fot(yaml_path);
                fot << point_yaml;
                fot.close();
                nopoint = 2;
            }
        }
        if (state == 1)
        {
            if (buffer_read[0] < 5 && buffer_read[0] > 0)
            {
                i = buffer_read[0];
                if (i == 1)
                {
                    ser.close();
                    std::system("~/Vscode/work/write/bin/write");
                    std::cout << "1 over" << std::endl;
                    return 0;
                }
                if (i == 2)
                {
                    while (!isdetect)
                    {
                        cap.open(2, cv::CAP_V4L2);
                        cv::Mat img;
                        cap >> img;
                        identify::color a = identify::nocolor;
                        identify::shape b = identify::none;
                        cap >> img;
                        b = identify::shape_kind(img);
                        a = identify::color_kind(img, 0.1);
                        if (a == identify::red && b == identify::square)
                        {
                            isdetect = true;
                            point_yaml["first_point"] = 9;
                            point_yaml["second_point"] = 10;
                            std::ofstream fot(yaml_path);
                            fot << point_yaml;
                            fot.close();
                            cap.release();
                            int mp3_num = 5;
                            mp3_path = mp3_path + std::to_string(mp3_num);
                            char *mp3_path_ = const_cast<char *>(mp3_path.c_str());
                            system(mp3_path_);
                            break;
                        }
                        if (a == identify::blue && b == identify::square)
                        {
                            isdetect = true;
                            point_yaml["first_point"] = 11;
                            point_yaml["second_point"] = 12;
                            std::ofstream fot(yaml_path);
                            fot << point_yaml;
                            fot.close();
                            cap.release();
                            int mp3_num = 4;
                            mp3_path = mp3_path + std::to_string(mp3_num);
                            char *mp3_path_ = const_cast<char *>(mp3_path.c_str());
                            system(mp3_path_);
                            break;
                        }
                        if (a == identify::red && b == identify::circle)
                        {
                            isdetect = true;
                            point_yaml["first_point"] = 6;
                            point_yaml["second_point"] = 5;
                            std::ofstream fot(yaml_path);
                            fot << point_yaml;
                            fot.close();
                            cap.release();
                            int mp3_num = 3;
                            mp3_path = mp3_path + std::to_string(mp3_num);
                            char *mp3_path_ = const_cast<char *>(mp3_path.c_str());
                            system(mp3_path_);
                            break;
                        }
                        if (a == identify::blue && b == identify::circle)
                        {
                            isdetect = true;
                            point_yaml["first_point"] = 7;
                            point_yaml["second_point"] = 8;
                            std::ofstream fot(yaml_path);
                            fot << point_yaml;
                            fot.close();
                            cap.release();
                            int mp3_num = 2;
                            mp3_path = mp3_path + std::to_string(mp3_num);
                            char *mp3_path_ = const_cast<char *>(mp3_path.c_str());
                            system(mp3_path_);
                            break;
                        }
                        if (a == identify::red && b == identify::triangle)
                        {
                            isdetect = true;
                            point_yaml["first_point"] = 2;
                            point_yaml["second_point"] = 1;
                            std::ofstream fot(yaml_path);
                            fot << point_yaml;
                            fot.close();
                            cap.release();
                            int mp3_num = 7;
                            mp3_path = mp3_path + std::to_string(mp3_num);
                            char *mp3_path_ = const_cast<char *>(mp3_path.c_str());
                            system(mp3_path_);
                            break;
                        }
                        if (a == identify::blue && b == identify::triangle)
                        {
                            isdetect = true;
                            point_yaml["first_point"] = 3;
                            point_yaml["second_point"] = 4;
                            std::ofstream fot(yaml_path);
                            fot << point_yaml;
                            fot.close();
                            cap.release();
                            int mp3_num = 6;
                            mp3_path = mp3_path + std::to_string(mp3_num);
                            char *mp3_path_ = const_cast<char *>(mp3_path.c_str());
                            system(mp3_path_);
                            break;
                        }
                        cap.release();
                    }
                    break;
                }
                if (i == 3)
                {
                    point_yaml["three_state"] = 1;
                    std::ofstream fot(yaml_path);
                    fot << point_yaml;
                    fot.close();
                    ser.close();
                    // std::system("");
                    std::cout << "three_state:\t" << point_yaml["three_state"] << std::endl;
                    std::cout << "over" << std::endl;
                    break;
                }
                if (i == 4)
                {
                    point_yaml["three_state"] = 2;
                    std::ofstream fot(yaml_path);
                    fot << point_yaml;
                    fot.close();
                    ser.close();
                    // std::system("");
                    std::cout << "three_state:\t" << point_yaml["three_state"] << std::endl;
                    std::cout << "over" << std::endl;
                }
            }
            buffer_read[0] = 0;
        }
    }
    return 0;
}
