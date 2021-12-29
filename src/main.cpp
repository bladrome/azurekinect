#include "mkvparser.hpp"
#include "opencv2/highgui.hpp"
#include "utils.hpp"
#include <cmath>
#include <k4a/k4a.hpp>
#include "cxxopts.hpp"


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

    while (apb.next()) {
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
mkvmeasure(int argc, char *argv[])
{
    // -f ../877777.mkv -s 7812711 --x1 529 --y1  75 --x2 543 --y2 535
    cxxopts::Options options("mkvmeasure", "Azure mkv distance measure");

    options.add_options()
        ("f,mkvfilename", "mkvfile name", cxxopts::value<std::string>())
        ("s,seektime", "Seek time", cxxopts::value<long long>()->default_value("0"))
        ("x1", "x1", cxxopts::value<int>()->default_value("0"))
        ("y1", "y1", cxxopts::value<int>()->default_value("0"))
        ("x2", "x2", cxxopts::value<int>()->default_value("0"))
        ("y2", "y2", cxxopts::value<int>()->default_value("0"))
        ("h,help", "Print usage")
    ;


    auto result = options.parse(argc, argv);

    if (result.count("help"))
    {
      std::cout << options.help() << std::endl;
      exit(0);
    }

    std::cout << mkv_get_distance(
        result["mkvfilename"].as<std::string>(),
        result["seektime"].as<long long>(),
        result["x1"].as<int>(),
        result["y1"].as<int>(),
        result["x2"].as<int>(),
        result["y2"].as<int>()) << std::endl;


    return 0;
}


int main(int argc, char *argv[]) {

    // AzurePlayback apb("91.mkv", 0);
    // std::cout << apb.get_distance(529, 75, 543, 535) << std::endl;
    // apb.export_point_cloud();
    // cv_show("91.mkv");
    mkvmeasure(argc, argv);
    return 0;
}
