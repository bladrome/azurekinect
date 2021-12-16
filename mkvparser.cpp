#include <iostream>
#include <filesystem>
#include <chrono>
#include <string>
#include <thread>

#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <turbojpeg.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <turbojpeg.h>

class AzurePlayback {

    std::string filename;
    std::string serial_num;

    k4a::playback playback;
    k4a::capture capture;
    k4a::calibration calibration;
    k4a::transformation transformation;
    k4a_record_configuration_t configuration;

    int width;
    int height;
    std::chrono::microseconds recored_length;

  public:
    AzurePlayback(std::string filename) {
        playback = k4a::playback::open(filename.c_str());
        playback.seek_timestamp(std::chrono::microseconds(500000),
                                K4A_PLAYBACK_SEEK_BEGIN);
        playback.get_next_capture(&capture);
        calibration = playback.get_calibration();
        recored_length = playback.get_recording_length();
        configuration = playback.get_record_configuration();
    }

    ~AzurePlayback() {
        transformation.destroy();
        capture.reset();
        playback.close();
    }

    cv::Mat trans_to_8U(cv::Mat cv_image_16U) {
        double min;
        double max;
        cv::Mat adjMap;
        cv::Mat cv_image_8U;

        // linear map, black
        // cv_image_16U.convertTo(cv_image_8U, CV_8U, 1 / 257.0);

        // normalize, black
        // cv::normalize(cv_image_16U, cv_image_8U, 255, 0, cv::NORM_MINMAX);

        cv::minMaxIdx(cv_image_16U, &min, &max);
        float scale = 255 / (max - min);
        cv_image_16U.convertTo(adjMap, CV_8UC1, scale, -min * scale);
        cv::applyColorMap(adjMap, cv_image_8U, cv::COLORMAP_HSV);

        return cv_image_8U;
    }

    cv::Mat get_rgb() {
        cv::Mat cv_rgb;

        k4a::image color = capture.get_color_image();

        k4a::image uncompressed_color = k4a::image::create(
            K4A_IMAGE_FORMAT_COLOR_BGRA32, color.get_width_pixels(),
            color.get_height_pixels(),
            color.get_width_pixels() * 4 * (int)sizeof(uint8_t));

        switch (color.get_format()) {
        case K4A_IMAGE_FORMAT_COLOR_MJPG: {
            tjhandle tjHandle = tjInitDecompress();
            if (tjDecompress2(tjHandle, color.get_buffer(),
                              static_cast<unsigned long>(color.get_size()),
                              uncompressed_color.get_buffer(),
                              color.get_width_pixels(), 0,
                              color.get_height_pixels(), TJPF_BGRA,
                              TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0) {
                std::cout << "Failed to decompressed color frame" << std::endl;
                exit(1);
            }
            if (tjDestroy(tjHandle)) {
                std::cout << "Failed to destroy turboJPEG handle" << std::endl;
                exit(1);
            }

            cv::Mat cv_color =
                cv::Mat(uncompressed_color.get_height_pixels(),
                        uncompressed_color.get_width_pixels(), CV_8UC4,
                        uncompressed_color.get_buffer());

            cv::cvtColor(cv_color, cv_rgb, cv::COLOR_BGRA2BGR);
            break;
        }
        default:
            break;
        }

        return cv_rgb;
    }

    cv::Mat get_depth() {

        k4a::image depth = capture.get_depth_image();
        transformation = k4a::transformation(calibration);
        k4a::image transformed_depth_image =
            transformation.depth_image_to_color_camera(depth);
        cv::Mat cv_depth = cv::Mat(
            transformed_depth_image.get_height_pixels(),
            transformed_depth_image.get_width_pixels(), CV_16U,
            transformed_depth_image.get_buffer(),
            static_cast<size_t>(transformed_depth_image.get_stride_bytes()));

        return cv_depth;
    }

    cv::Mat get_depth_8U() { return trans_to_8U(get_depth()); }

    cv::Mat get_ir() {
        k4a::image ir_image = capture.get_ir_image();

        cv::Mat cv_ir =
            cv::Mat(ir_image.get_height_pixels(), ir_image.get_width_pixels(),
                    CV_16U, ir_image.get_buffer(),
                    static_cast<size_t>(ir_image.get_stride_bytes()));

        return cv_ir;
    }

    cv::Mat get_ir_8U() { return trans_to_8U(get_ir()); }

    float get_temperature_c() { return capture.get_temperature_c(); }

    bool next() {
        playback.get_next_capture(&capture);
        return capture.is_valid();
    }
};

int main(int argc, char* argv[]) {
    AzurePlayback apb("/home/bladrome/jack/day06/8.mkv");

    cv::namedWindow("color", cv::WINDOW_NORMAL);

    cv::Mat color = apb.get_rgb();
    std::cout << color.cols << " " << color.rows << std::endl;

    while (apb.next()) {
        cv::Mat color = apb.get_rgb();
        cv::imshow("color", color);

        if (cv::waitKey(10) == 27)
            break;
    }

    return 0;
}
