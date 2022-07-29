#include <opencv2/opencv.hpp>
#include <math.h>
namespace depth
{
    const double depth_scale = 0.001;
    const double cx = 320.207;
    const double cy = 236.529;
    const double fx = 388.677;
    const double fy = 388.677;
    struct axis
    {
        double real_x;
        double real_y;
        double real_z;
        axis(double _real_x,double _real_y,double _real_z){real_x=_real_x;real_y=_real_y;real_z=_real_z;}
        axis(){real_x=0;real_y=0;real_z=0;}
    };
    //计算深度 pixel.x为col,pixel.y为row
    axis Cal_axis(uint16_t depth_value, cv::Point2d pixel)
    {
        axis A;
        A.real_z = depth_value * depth_scale;
        A.real_y = A.real_z / fy * (pixel.y - cy);
        A.real_x = A.real_z / fx * (pixel.x - cx);
        return A;
    }

    // n为数组长度，error为容许的误差
    double filter_depth(double b[], int n, double error)
    {
        double sum = 0;
        for (int i = 0; i < n; i++)
        {
            sum = sum + b[i];
        }
        int n_real = n;
        double sum_real = sum;
        double arrage = sum / n;
        for (int i = 0; i < n; i++)
        {
            if (abs(b[i] - arrage) > error)
            {
                n_real = n_real - 1;
                b[i] = 0;
                sum_real = sum_real - b[i];
            }
        }
        double arrage_real = sum_real / n_real;
        return arrage_real;
    }
}
