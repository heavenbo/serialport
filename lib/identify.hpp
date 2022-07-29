#include <opencv2/opencv.hpp>
#include <math.h>
#include "EDLib.h"
#include "locate_elg.hpp"

namespace identify
{
    const double thresh = 150;
    const int RATIO = 3;
    const int kernel_size = 3;
    const int lowThreshold = 50;
    const double real_x=0.00296;
    const double real_y=0.00317;
    enum color
    {
        blue,
        white,
        gray,
        purple,
        yellow,
        green,
        red,
        black,
        cyan,
        orange,
        nocolor,
        test
    };
    enum shape
    {
        triangle,
        square,
        circle,
        none
    };
    void color_Range(cv::InputArray img, cv::OutputArray imgThresholded, const color &ctrl)
    {
        cv::Mat imghsv;
        cv::cvtColor(img, imghsv, cv::COLOR_BGR2HSV);
        switch (ctrl)
        {
        case blue:
        {
            cv::inRange(imghsv, cv::Scalar(100, 43, 46), cv::Scalar(150, 255, 255), imgThresholded); //蓝色
            break;
        }
        case white:
        {
            cv::inRange(imghsv, cv::Scalar(0, 0, 221), cv::Scalar(180, 30, 255), imgThresholded); //白色
            break;
        }
        case gray:
        {
            cv::inRange(imghsv, cv::Scalar(0, 0, 46), cv::Scalar(180, 43, 220), imgThresholded); //灰色
            break;
        }
        case purple:
        {
            cv::inRange(imghsv, cv::Scalar(125, 43, 46), cv::Scalar(155, 255, 255), imgThresholded); //紫色
            break;
        }
        case yellow:
        {
            cv::inRange(imghsv, cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255), imgThresholded); //黄色
            break;
        }
        case green:
        {
            cv::inRange(imghsv, cv::Scalar(40, 43, 110), cv::Scalar(77, 150, 255), imgThresholded); //绿色
            break;
        }
        case red:
        {
            cv::Mat imgThresholded1, imgThresholded2;
            cv::inRange(imghsv, cv::Scalar(0, 100, 150), cv::Scalar(15, 200, 200), imgThresholded1); //红色
            cv::inRange(imghsv, cv::Scalar(150, 100, 100), cv::Scalar(180, 255, 255), imgThresholded2);
            cv::add(imgThresholded1, imgThresholded2, imgThresholded);

            break;
        }
        case black:
        {
            cv::inRange(imghsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 90), imgThresholded); //黑色
            break;
        }
        case cyan:
        {
            cv::inRange(imghsv, cv::Scalar(78, 43, 46), cv::Scalar(99, 255, 255), imgThresholded); //青色
            break;
        }
        case orange:
        {
            cv::inRange(imghsv, cv::Scalar(11, 43, 46), cv::Scalar(25, 255, 255), imgThresholded); //橙色
            break;
        }
        case test:
        {
            cv::inRange(imghsv, cv::Scalar(0, 0, 0), cv::Scalar(40, 255, 255), imgThresholded); // test
            break;
        }
        default:
        {
            cv::inRange(imghsv, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255), imgThresholded);
            break;
        }
        }
    }
    bool color_center(cv::InputArray img_two, cv::InputOutputArray result_img,
                      cv::Point2f &photo_center)
    {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarcy;
        findContours(img_two, contours, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty())
        {
            return false;
        }
        else
        {
            std::vector<cv::Point> contour;
            unsigned long max_contour = 0;
            double max_area = cv::contourArea(contours[0]);
            for (unsigned long i = 1; i < contours.size(); i++)
            {
                double temp_area = cv::contourArea(contours[i]);
                if (max_area < temp_area)
                {
                    max_area = temp_area;
                    max_contour = i;
                }
            }
            contour = contours[max_contour];
            if (max_area < 50.0)
            {
                photo_center.x = 0;
                photo_center.y = 0;
                return false;
            }
            else
            {
                std::vector<cv::RotatedRect> box(contour.size()); //最小外接矩形
                cv::Point2f rect[4];

                box[0] = cv::minAreaRect(cv::Mat(contour));
                box[0].points(rect); //最小外接矩形的4个端点
                int x = 0, y = 0;
                for (int j = 0; j < 4; j++)
                {
                    cv::line(result_img, rect[j], rect[(j + 1) % 4], cv::Scalar(0, 0, 255));
                    x = x + rect[j].x;
                    y = y + rect[j].y;
                }
                photo_center.x = x / 4;
                photo_center.y = y / 4;
                return true;
            }
        }
    }
    bool isstopped(double drone_vel_x, double drone_vel_y, double time,
                   const cv::Point2f &pre_cen, const cv::Point2f &cen, double coff,
                   double limvel, double lim_drone_vel)
    {
        double x_sub = coff * (pre_cen.x - cen.x);
        double y_sub = coff * (pre_cen.y - cen.y);
        double distance = sqrt(pow(x_sub, 2) + pow(y_sub, 2));
        double vel = sqrt(pow(drone_vel_x, 2) + pow(drone_vel_y, 2));
        std::cout << "distance=" << distance << std::endl;
        if (distance < limvel * time && vel < lim_drone_vel)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    color color_kind(const cv::Mat &img, double confidence)
    {
        double color_area;
        cv::Mat out_Img;
        color color_ = nocolor;
        int photo_area = img.cols * img.rows;
        color_Range(img, out_Img, red);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarcy;
        findContours(out_Img, contours, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (unsigned long i = 0; i < contours.size(); i++)
        {
            double temp_area = cv::contourArea(contours[i]);
            color_area = color_area + temp_area;
        }
        if (color_area / photo_area > confidence)
        {
            color_ = red;
            return color_;
        }
        color_area = 0;
        color_Range(img, out_Img, blue);
        cv::GaussianBlur(out_Img, out_Img, cv::Size(7, 7), 2, 2); //高斯模糊
        cv::medianBlur(out_Img, out_Img, 3);                      //中值滤波
        findContours(out_Img, contours, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (unsigned long i = 0; i < contours.size(); i++)
        {
            double temp_area = cv::contourArea(contours[i]);
            color_area = color_area + temp_area;
        }
        if (color_area / photo_area > confidence)
        {
            color_ = blue;
            return color_;
        }
        return color_;
    }
    double angleCos(cv::Point pt1, cv::Point pt2, cv::Point pt0)
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));
    }
    shape shape_kind(const cv::Mat &img)
    {
        shape king_shape = none;
        bool istri;
        cv::Mat img_two, img_canny;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> vertices_contour, triangles;
        cv::cvtColor(img, img_two, cv::COLOR_RGB2GRAY);
        mEllipse m = mEllipse(cv::Point2d(0, 0), cv::Size(0, 0), 0);
        if (get_elp(img, m))
        {
            king_shape = circle;
        }
        else
        {
            cv::Canny(img_two, img_canny, lowThreshold, RATIO * lowThreshold, kernel_size);
            // find coutours
            cv::findContours(img_canny.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            for (int i = 0; i < contours.size(); i++)
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                cv::approxPolyDP(cv::Mat(contours[i]), vertices_contour, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

                // skip small or non-convex objects
                if (fabs(cv::contourArea(contours[i])) < 300 || !cv::isContourConvex(vertices_contour))
                {
                    continue;
                }

                if (vertices_contour.size() == 3)
                { // TRIANGLES
                    king_shape = triangle;
                    istri = true;
                }
                else if (vertices_contour.size() == 4 && istri == false)
                {
                    int nb_vertices = 4; // vertices_contour.size();

                    //     //calculation of "cos" from all corners
                    std::vector<double> cos;
                    for (int j = 2; j < nb_vertices + 1; j++)
                    {
                        cos.push_back(angleCos(vertices_contour[j % nb_vertices], vertices_contour[j - 2], vertices_contour[j - 1]));
                    }

                    //     //storage of "cos" in ascending order
                    std::sort(cos.begin(), cos.end());

                    //     //save the min and max values of "cos"
                    double mincos = cos.front();
                    double maxcos = cos.back();

                    //     //check that the corners are right angles
                    if (mincos >= -0.3 && maxcos <= 0.3)
                    {
                        king_shape = square;
                    }
                }
            }
        }
        return king_shape;
    }
    bool square_center(const cv::Mat &img, cv::Point2d &center_square)
    {
        cv::Mat img_two, img_canny;
        bool issquare = false;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> vertices_contour;
        std::vector<cv::RotatedRect> rectangles;
        int max_square = 0;
        cv::cvtColor(img, img_two, cv::COLOR_RGB2GRAY);
        cv::Canny(img_two, img_canny, lowThreshold, RATIO * lowThreshold, kernel_size);
        // find coutours
        cv::findContours(img_canny.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++)
        {
            // approximate contour with accuracy proportional
            // to the contour perimeter
            cv::approxPolyDP(cv::Mat(contours[i]), vertices_contour, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);
            if (fabs(cv::contourArea(contours[i])) < 300 || !cv::isContourConvex(vertices_contour))
            {
                continue;
            }
            if (vertices_contour.size() == 4)
            {
                int nb_vertices = 4; // vertices_contour.size();

                // calculation of "cos" from all corners
                std::vector<double> cos;
                for (int j = 2; j < nb_vertices + 1; j++)
                {
                    cos.push_back(angleCos(vertices_contour[j % nb_vertices], vertices_contour[j - 2], vertices_contour[j - 1]));
                }
                //     //storage of "cos" in ascending order
                std::sort(cos.begin(), cos.end());

                //     //save the min and max values of "cos"
                double mincos = cos.front();
                double maxcos = cos.back();

                //     //check that the corners are right angles
                if (mincos >= -0.3 && maxcos <= 0.3)
                { // RECTANGLES
                    cv::RotatedRect rotRect = cv::minAreaRect(contours[i]);
                    rectangles.push_back(rotRect); // add to the set of detected rectangles
                }
            }
        }
        if (!rectangles.empty())
        {
            for (int i = 0; i < rectangles.size(); i++)
            {
                if (rectangles[i].size.area() > rectangles[max_square].size.area())
                {
                    max_square = i;
                }
            }
            center_square = rectangles[max_square].center;
            issquare = true;
        }
        return issquare;
    }
    cv::Point2d real_s(cv::Point2d cal_point,cv::Point2d center_point)
    {
        cv::Point2f real_d=cv::Point2f(0,0);
        real_d.x = (cal_point.x - center_point.x) * real_x;
        real_d.y = (cal_point.y - center_point.y) * real_y;
        return real_d;
    }
    bool tri_center(const cv::Mat &img, cv::Point2d &center_tri)
    {
        bool istri = false;
        cv::Mat img_two, img_canny;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> vertices_contour;
        std::vector<cv::Point> max_contour;
        std::vector<std::vector<cv::Point>> triangles;
        cv::cvtColor(img, img_two, cv::COLOR_RGB2GRAY);
        Canny(img_two, img_canny, lowThreshold, RATIO * lowThreshold, kernel_size);
        // find coutours
        cv::findContours(img_canny.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++)
        {
            // approximate contour with accuracy proportional
            // to the contour perimeter
            cv::approxPolyDP(cv::Mat(contours[i]), vertices_contour, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);
            if (fabs(cv::contourArea(contours[i])) < 300 || !cv::isContourConvex(vertices_contour))
            {
                continue;
            }
            if (vertices_contour.size() == 3)
            {
                if (max_contour.empty() || cv::contourArea(contours[i]) > cv::contourArea(max_contour))
                {
                    max_contour = vertices_contour;
                }
            }
        }
        if (!max_contour.empty())
        {
            center_tri.x = (max_contour[0].x + max_contour[2].x + max_contour[1].x) / 3;
            center_tri.y = (max_contour[0].y + max_contour[1].y + max_contour[2].y) / 3;
            istri = true;
        }
        return istri;
    }
}