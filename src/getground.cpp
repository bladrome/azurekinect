#include "mkvparser.hpp"


int main(int argc, char *argv[]) {

    std::string filename = "/home/bladrome/jackdatabank/databank1/jack/dataqinghuan/dimensiondata/sideview_day01_86.mkv";
    AzurePlayback apb(filename, 64);
    auto ground = apb.get_ground();
    std::cout << ground << std::endl;

    std::cout << ground.val[3] << std::endl;

    return 0;
}
