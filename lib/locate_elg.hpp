#include "EDLib.h"
#include <opencv2/opencv.hpp>
#include "depth.hpp"
// struct vec_ax
// {
//     double x;
//     double y;
//     double z;
//     vec_ax(double _x, double _y, double _z)
//     {
//         x = _x;
//         y = _y;
//         z = _z;
//     }
// };
const int area = 1500;
const int s = 300;
// in_Img为二值图像
bool get_elp(const ::cv::Mat &in_Img, mEllipse &c)
{
    cv::Mat img_two;
    cv::cvtColor(in_Img, img_two, cv::COLOR_BGR2GRAY);
    bool isappeared = false;
    EDPF testEDPF = EDPF(img_two);
    EDCircles testEDCircles = EDCircles(testEDPF);

    // cv::Mat edgePFImage = testEDPF.getEdgeImage();
    // cv::imshow("Edge Image Parameter Free", edgePFImage);

    // Get circle information as [cx, cy, r]
    std::vector<mCircle> circles = testEDCircles.getCircles();
    // Get ellipse information as [cx, cy, a, b, theta]
    std::vector<mEllipse> ellipses = testEDCircles.getEllipses();

    int nocircle = circles.capacity();
    int noellipse = ellipses.capacity();
    int find_max_circle = 0;
    int find_max_ellipses = 0;
    if (nocircle != 0)
    {
        for (int i = 1; i < nocircle; i++)
        {
            if (circles[find_max_circle].r < circles[i].r)
                find_max_circle = i;
        }
    }
    if (noellipse != 0)
    {
        for (int i = 1; i < find_max_ellipses; i++)
        {
            if (ellipses[find_max_ellipses].axes.area() < ellipses[i].axes.area())
                find_max_ellipses = i;
        }
    }
    if (noellipse != 0 && nocircle != 0)
    {
        if (pow(circles[find_max_circle].r, 2) > ellipses[find_max_ellipses].axes.area())
        {
            if (pow(circles[find_max_circle].r, 2) > area)
            {
                cv::Size r = cv::Size((int)circles[find_max_circle].r, (int)circles[find_max_circle].r);
                c = mEllipse(circles[find_max_circle].center, r, 0);
                isappeared = true;
            }
            else
            {
                isappeared = false;
            }
        }
        else
        {
            if (ellipses[find_max_ellipses].axes.area() > area)
            {
                isappeared = true;
                c = ellipses[find_max_ellipses];
            }
            else
            {
                isappeared = false;
            }
        }
    }
    if (nocircle != 0 && noellipse == 0)
    {
        if (pow(circles[find_max_circle].r, 2) > area)
        {
            cv::Size r = cv::Size((int)circles[find_max_circle].r, (int)circles[find_max_circle].r);
            c = mEllipse(circles[find_max_circle].center, r, 0);
            isappeared = true;
        }
        else
        {
            isappeared = false;
        }
    }
    if (nocircle == 0 && noellipse != 0)
    {
        if (ellipses[find_max_ellipses].axes.area() > area)
        {
            isappeared = true;
            c = ellipses[find_max_ellipses];
        }
        else
        {
            isappeared = false;
        }
    }
    if (nocircle == 0 && noellipse == 0)
    {
        isappeared = false;
        c = mEllipse(cv::Point2d(0, 0), cv::Size(0, 0), 0);
    }
    return isappeared;
}
bool cal_elp(cv::Mat &depth_img, mEllipse &elp, double r, depth::axis &r_ax, depth::axis &r_vec)
{
    bool iscal = false;
    cv::Point2d p_on((int)elp.center.x, (int)elp.center.y - elp.axes.height);
    cv::Point2d p_down((int)elp.center.x, (int)elp.center.y + elp.axes.height);
    cv::Point2d p_left((int)elp.center.x - elp.axes.width, (int)elp.center.y);
    cv::Point2d p_right((int)elp.center.x + elp.axes.width, (int)elp.center.y);
    uint16_t p_on_z = depth_img.at<uint16_t>(p_on.x, p_on.y);
    uint16_t p_down_z = depth_img.at<uint16_t>(p_down.x, p_down.y);
    uint16_t p_left_z = depth_img.at<uint16_t>(p_left.x, p_left.y);
    uint16_t p_right_z = depth_img.at<uint16_t>(p_right.x, p_right.y);
    if (p_on_z > s && p_down_z > s && (p_left_z > s || p_right_z > s))
    {
        depth::axis p_on_axis = depth::Cal_axis(p_on_z, p_on);
        depth::axis p_down_axis = depth::Cal_axis(p_down_z, p_down);
        // if (abs(p_down_axis.real_y - p_on_axis.real_y - 2 * r) < 0.1)
        if (abs(p_down_axis.real_z - p_on_axis.real_z) < 0.05)
        {
            iscal = true;
            r_ax = depth::axis(p_on_axis.real_x, p_on_axis.real_y + r, (p_on_axis.real_z + p_down_axis.real_z) / 2);
        }
        else
        {
            iscal = false;
            r_ax = depth::axis();
        }
    }
    else
    {
        if (p_on_z > s && p_left_z > s)
        {
            depth::axis p_on_axis = depth::Cal_axis(p_on_z, p_on);
            r_ax = depth::axis(p_on_axis.real_x, p_on_axis.real_y + r, p_on_axis.real_z);
            iscal = true;
        }
        if (p_on_z > s && p_right_z > s)
        {
            depth::axis p_on_axis = depth::Cal_axis(p_on_z, p_on);
            r_ax = depth::axis(p_on_axis.real_x, p_on_axis.real_y + r, p_on_axis.real_z);
            iscal = true;
        }
        if (p_down_z > s && p_left_z > s)
        {
            depth::axis p_down_axis = depth::Cal_axis(p_down_z, p_down);
            r_ax = depth::axis(p_down_axis.real_x, p_down_axis.real_y - r, p_down_axis.real_z);
            iscal = true;
        }
        if (p_down_z > s && p_left_z > s)
        {
            depth::axis p_down_axis = depth::Cal_axis(p_down_z, p_down);
            r_ax = depth::axis(p_down_axis.real_x, p_down_axis.real_y - r, p_down_axis.real_z);
            iscal = true;
        }
    }
    return iscal;
}