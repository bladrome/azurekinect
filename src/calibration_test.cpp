// https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/structk4a_1_1calibration.html
#include <k4a/k4atypes.h>
#include "mkvparser.hpp"
#include "opencv2/calib3d.hpp"

using namespace cv;

void printMat(Mat M){
    for(int i = 0; i < M.rows; i++)
    {
        for(int j = 0; j < M.cols; j++)
            std::cout << M.at<float>(i, j) << "\t";
        std::cout << std::endl;
    }
}


int main(int argc, char *argv[]) {

    std::string filename = "/home/bladrome/jackdatabank/databank1/jack/dataqinghuan/dimensiondata/sideview_day01_15.mkv";
    AzurePlayback apb(filename, 64);

    std::cout << apb.calibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation << std::endl;


    Mat se3 =
        Mat(3, 3, CV_32FC1, apb.calibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation);
    Mat r_vec = Mat(3, 1, CV_32FC1);
    Rodrigues(se3, r_vec);
    Mat t_vec =
        Mat(3, 1, CV_32F, apb.calibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation);


/*
    std::cout << apb.calibration.color_camera_calibration.resolution_width << std::endl;
    std::cout << apb.calibration.color_camera_calibration.resolution_height << std::endl;

    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.cx << std::endl;
    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.cy << std::endl;
    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.fx << std::endl;
    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.fy << std::endl;


    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.k1 << std::endl;
    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.k2 << std::endl;
    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.k3 << std::endl;
    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.k4 << std::endl;
    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.k5 << std::endl;
    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.k6 << std::endl;

    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.codx << std::endl;
    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.cody << std::endl;
    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.p1 << std::endl;
    std::cout << apb.calibration.color_camera_calibration.intrinsics.parameters.param.p2 << std::endl;
*/

    // printMat(se3);
    // printMat(t_vec);


    std::cout << "Rotation:\n";
    for (int i = 0; i < 9; i++) {
        std::cout << apb.calibration.color_camera_calibration.extrinsics.rotation[i] << std::endl;
    }
    std::cout << "Translation:\n";
    for (int i = 0; i < 3; i++) {
        std::cout << apb.calibration.color_camera_calibration.extrinsics.translation[i] << std::endl;
    }

    return 0;
}
