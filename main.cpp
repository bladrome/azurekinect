#include <iostream>
#include "mkvparser.hpp"



int
main(int argc, char *argv[])
{
    // std::string filename("/home/bladrome/jack/day06/877777.mkv");
    std::string filename("877777.mkv");
    // mkv_show(filename);
    // mkv_gen_cloud(filename);

    AzurePlayback apb(filename.c_str());
    std::cout << "exportall: " << apb.export_all() << std::endl;

    return 0;
}
