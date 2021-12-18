#include "mkvparser.hpp"
#include "opencv2/highgui.hpp"
#include "utils.hpp"
#include <cmath>
#include <k4a/k4a.hpp>


void
cv_show(std::string filename)
{
    AzurePlayback apb(filename.c_str());
    ClickData *click_data = new ClickData();

    cv::namedWindow("color", cv::WINDOW_NORMAL);
    cv::setMouseCallback("color", CallBackFunc, click_data);

    cv::Mat color = apb.get_rgb();
    std::cout << color.cols << " " << color.rows << std::endl;
    // std::cout << "exportall: " << apb.export_all() << std::endl;

    while (true) {
        apb.next();
        cv::Mat color = apb.get_rgb();

        cv::Mat measure_image = color.clone();
        cv::imshow("color", color);



        if (cv::waitKey(30) == ' ') {
            do {
                if (click_data->n_clicks == 2) {
                    cv::line(measure_image,
                             cv::Point(click_data->x1, click_data->y1),
                             cv::Point(click_data->x, click_data->y),
                             cv::Scalar(0, 0, 0), cv::LINE_4);
                }
                cv::imshow("color", measure_image);
                float distance = apb.get_distance(click_data->x,
                                                  click_data->y,
                                                  click_data->x1,
                                                  click_data->y1);
                std::cout << "#################### distance: " << distance
                          << std::endl;
                click_data->n_clicks = 0;
            } while (cv::waitKey(0) != ' ');
        }

        if (cv::waitKey(30) == 'q')
            break;
    }

    color.release();
    cv::destroyAllWindows();
}



int
main(int argc, char *argv[])
{
    // std::string filename("/home/bladrome/jack/day06/877777.mkv");
    std::string filename("../877777.mkv");
    // cv_show(filename);
    // mkv_gen_cloud(filename);
    AzurePlayback apb(filename.c_str(), 7812711);
    std::cout << apb.get_distance(529, 75, 543, 535) << std::endl;

    std::cout << mkv_get_distance(filename, 7812711, 529, 75, 543, 535) << std::endl;


    return 0;
}
