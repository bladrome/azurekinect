#include "mkvparser.hpp"
#include "opencv2/highgui.hpp"
#include "utils.hpp"
#include <cmath>
#include <k4a/k4a.hpp>
#include <string>
#include "cxxopts.hpp"


int
mkvgenpointclouds(int argc, char *argv[])
{
    // -f ../877777.mkv -s 7812711 --x1 529 --y1  75 --x2 543 --y2 535
    cxxopts::Options options("mkvmeasure", "Azure mkv distance measure");

    options.add_options()
        ("f,mkvfilename", "mkvfile name", cxxopts::value<std::string>())
        ("s,seektime", "Seek time", cxxopts::value<long long int>()->default_value("0"))
        ("h,help", "Print usage")
    ;


    auto result = options.parse(argc, argv);

    if (argc < 2 || result.count("help"))
    {
      std::cout << options.help() << std::endl;
      exit(0);
    }

    AzurePlayback apb(result["mkvfilename"].as<std::string>(),
                      result["seektime"].as<long long int>());
    apb.export_point_cloud();

    return 0;
}


int main(int argc, char *argv[]) {

    mkvgenpointclouds(argc, argv);
    return 0;
}
