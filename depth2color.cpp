
#include <iostream>
#include <chrono>
#include <k4a/k4a.h>
#include <k4arecord/types.h>
#include <thread>
#include <string>

#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <k4a/k4atypes.h>
#include <turbojpeg.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <turbojpeg.h>


int playback(const char *input_path);
int capture_and_show() {


int main(int argc, char *argv[]) {

    const char *input_path = argv[1];
    playback(input_path);

    return 0;
}


int playback(const char *input_path) {
    k4a::playback playback = k4a::playback::open(input_path);
    k4a::capture capture;

    k4a::image color;

    cv::Mat cv_rgb_image_with_alpha;
    cv::Mat cv_rgb_image_no_alpha;

    cv::namedWindow( "color", cv::WINDOW_NORMAL );
    cv::namedWindow( "depth", cv::WINDOW_NORMAL );

    playback.seek_timestamp(std::chrono::microseconds(1000), K4A_PLAYBACK_SEEK_BEGIN);

    while (true) {

        playback.get_next_capture(&capture);
        color = capture.get_color_image();

        // Get Color frame
        k4a::image uncompressed_color = k4a::image::create(
            K4A_IMAGE_FORMAT_COLOR_BGRA32, color.get_width_pixels(),
            color.get_height_pixels(),
            color.get_width_pixels() * 4 * (int)sizeof(uint8_t));

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

        cv_rgb_image_with_alpha =
            cv::Mat(uncompressed_color.get_height_pixels(),
                    uncompressed_color.get_width_pixels(),
                    CV_8UC4,
                    (void*)uncompressed_color.get_buffer());

        std::cout << uncompressed_color.get_format() << std::endl;
        cv::cvtColor(cv_rgb_image_with_alpha, cv_rgb_image_no_alpha,
                     cv::COLOR_BGRA2BGR);

        // Depth Frame
        k4a::image depth = capture.get_depth_image();
        k4a::calibration k4a_calibration = playback.get_calibration();
        k4a::transformation k4a_transformation =
            k4a::transformation(k4a_calibration);
        k4a::image transformed_depth_image =
            k4a_transformation.depth_image_to_color_camera(depth);
        cv::Mat cv_depth = cv::Mat(
            transformed_depth_image.get_height_pixels(),
            transformed_depth_image.get_width_pixels(), CV_16U,
            (void*)transformed_depth_image.get_buffer(),
            static_cast<size_t>(transformed_depth_image.get_stride_bytes()));
        cv::Mat cv_depth_8U;
        cv::normalize(cv_depth, cv_depth_8U, 0, (1 << 16) - 1, cv::NORM_MINMAX);
        cv_depth_8U.convertTo(cv_depth, CV_8U, 1);

        cv::imshow("color", cv_rgb_image_no_alpha);
        cv::imshow("depth", cv_depth_8U);

        // clear
        color.reset();
        uncompressed_color.reset();
        capture.reset();

        cv_rgb_image_with_alpha.release();
        cv_rgb_image_no_alpha.release();

        if (cv::waitKey(25) == 'q')
            break;

    }
    capture.reset();
    playback.close();

    return 0;
}



int capture_and_show() {

    const uint32_t device_count = k4a::device::get_installed_count();

    if (!device_count) {
        std::cout << "No Kinect connected" << std::endl;
        return 0;
    }

    std::cout << "There are " << device_count << " Kinects connected"
              << std::endl;

    k4a::device device;
    device = k4a::device::open(K4A_DEVICE_DEFAULT);

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_5;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    config.synchronized_images_only = true;

    device.start_cameras(&config);

    k4a::capture capture;

    k4a::image rgb_image;
    k4a::image depth_image;
    k4a::image ir_image;
    k4a::image transformed_depth_image;

    cv::Mat cv_rgb_image_with_alpha;
    cv::Mat cv_rgb_image_no_alpha;
    cv::Mat cv_depth;
    cv::Mat cv_depth_8U;
    cv::Mat cv_ir_image;
    cv::Mat cv_ir_image_8U;

    cv::namedWindow( "color", cv::WINDOW_NORMAL );
    cv::namedWindow( "depth", cv::WINDOW_NORMAL );
    // cv::namedWindow( "ir", cv::WINDOW_NORMAL );

    while (true) {
        if (device.get_capture(&capture)) {
            rgb_image = capture.get_color_image();
            if (rgb_image) {
                cv_rgb_image_with_alpha = cv::Mat(rgb_image.get_height_pixels(),
                                                  rgb_image.get_width_pixels(),
                                                  CV_8UC4,
                                                  (void*)rgb_image.get_buffer());
                if (cv_rgb_image_with_alpha.empty()){
                    continue;
                }

                cv::cvtColor(cv_rgb_image_with_alpha, cv_rgb_image_no_alpha, cv::COLOR_BGRA2BGR);
                cv::imshow("color", cv_rgb_image_no_alpha);
            }

            // depth
            depth_image = capture.get_depth_image();

            k4a::calibration k4a_calibration = device.get_calibration(
                config.depth_mode, config.color_resolution);
            k4a::transformation k4a_transformation =
                k4a::transformation(k4a_calibration);
            transformed_depth_image =
                k4a_transformation.depth_image_to_color_camera(depth_image);
            cv_depth =
                cv::Mat(transformed_depth_image.get_height_pixels(),
                        transformed_depth_image.get_width_pixels(), CV_16U,
                        (void*)transformed_depth_image.get_buffer(),
                        static_cast<size_t>(transformed_depth_image.get_stride_bytes()));
            cv::normalize(cv_depth, cv_depth_8U, 0, (1 << 16) - 1,
                          cv::NORM_MINMAX);
            cv_depth_8U.convertTo(cv_depth, CV_8U, 1);
            cv::imshow("depth", cv_depth);

            // ir
            // ir_image = capture.get_ir_image();
            // cv_ir_image = cv::Mat(ir_image.get_height_pixels(),
            //                       ir_image.get_width_pixels(),
            //                       CV_16U,
            //                       (void*)ir_image.get_buffer(),
            //                       static_cast<size_t>(ir_image.get_stride_bytes()));
            // cv::normalize(cv_ir_image, cv_ir_image_8U, 0, 1 << 16, cv::NORM_MINMAX);
            // cv::imshow("ir", cv_ir_image_8U);


            // k4a_transformation.destroy();
            k4a_image_release(rgb_image.handle());
            k4a_image_release(depth_image.handle());
            k4a_image_release(ir_image.handle());
            k4a_image_release(transformed_depth_image.handle());

            cv_rgb_image_no_alpha.release();
            cv_rgb_image_with_alpha.release();
            cv_depth.release();
            cv_depth_8U.release();
            cv_ir_image.release();
            cv_ir_image_8U.release();

            capture.reset();

            if (cv::waitKey(25) == 'q') {
                break;
            }

        }

    }

    cv::destroyAllWindows();

    rgb_image.reset();
    depth_image.reset();
    ir_image.reset();
    capture.reset();
    device.close();

    return 0;
}
