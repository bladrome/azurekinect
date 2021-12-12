#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

class Point3D {
  private:
  public:
    Point3D(float, float, float);
    ~Point3D();
    float x, y, z;
    float get_distance(Point3D p);
};

Point3D::Point3D(float a, float b, float c) {
    x = a;
    y = b;
    z = c;
}

Point3D::~Point3D() {}

float Point3D::get_distance(Point3D p) {
    return sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) +
                (z - p.z) * (z - p.z));
}

class ClickData {
  private:
    /* data */
  public:
    ClickData(/* args */);
    ~ClickData();
    int n_clicks;
    float x1, y1;
    float x, y;
};

ClickData::ClickData() { this->n_clicks = 0; }

ClickData::~ClickData() {}

void CallBackFunc(int event, int x, int y, int flags, void* userdataptr) {
    ClickData* data = (ClickData*)userdataptr;
    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "Left button of the mouse is clicked - position (" << x
                  << ", " << y << ")" << std::endl;
        std::cout << "Surprisingly, current data->n_clicks is: " << data->n_clicks
             << std::endl;
        if (data->n_clicks == 0) {
            data->x1 = x;
            data->y1 = y;
        }
        data->n_clicks++;
    }
    // if (event == cv::EVENT_LBUTTONUP) {
       // data->x = x;
       // data->y = y;
    // }
    if (data->n_clicks > 0) {
        data->x = x;
        data->y = y;
    }

}

int average_window_filter(const cv::Mat trans_depth_image, int i, int j,
                          int height, int width) {
    bool found = false;
    int window_size = 1;
    float depth = 0;
    while (found == false) {
        window_size *= 2;
        int step = window_size / 2;

        int x_lower = std::max(i - step, 0);
        int x_upper = std::min(i + step, height);

        int y_lower = std::max(j - step, 0);
        int y_upper = std::min(j + step, width);

        int value = 0;
        int number = 0;

        for (int x = x_lower; x < x_upper; x++) {
            for (int y = y_lower; y < y_upper; y++) {
                if (trans_depth_image.at<cv::uint16_t>(x, y) > 0) {
                    found = true;
                    value += trans_depth_image.at<cv::uint16_t>(x, y);
                    number += 1;
                }
            }
        }
        if (found == true) {
            depth = int(value / number);
        }
    }
    return depth;
}

#endif // UTILS_H_
