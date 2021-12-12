
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <k4a/k4a.hpp>
#include <k4a/k4atypes.h>
#include <k4arecord/playback.hpp>
#include <turbojpeg.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <turbojpeg.h>
#include "utils.hpp"

int playback(const char* input_path);
int capture();

static Point3D
convert_2d_depth_to_3d_point_cloud(const k4a::calibration calibration,
                                   const cv::Mat trans_depth_image,
                                   float coordinate_x, float coordinate_y) {

    float depth;
    int coordinate_x_int = static_cast<int>(coordinate_x);
    int coordinate_y_int = static_cast<int>(coordinate_y);
    depth = average_window_filter(trans_depth_image, coordinate_x_int,
                                  coordinate_y_int, trans_depth_image.rows,
                                  trans_depth_image.cols);
    // depth = trans_depth_image.at<cv::int16_t>(coordinate_x_int,
    // coordinate_y_int);
    k4a_float3_t ray;
    k4a_float2_t point_2d;

    point_2d.xy.x = coordinate_x;
    point_2d.xy.y = coordinate_y;

    std::cout << coordinate_x_int << " " << coordinate_y_int << " "
              << trans_depth_image.cols << " " << trans_depth_image.rows
              << std::endl;

    // if (calibration.convert_2d_to_3d(point_2d, depth,
    // K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ray))
    if (calibration.convert_2d_to_3d(point_2d, depth,
                                     K4A_CALIBRATION_TYPE_COLOR,
                                     K4A_CALIBRATION_TYPE_COLOR, &ray)) {
        std::cout << "2dx = " << coordinate_x_int
                  << " | 2dy = " << coordinate_y_int << " | x = " << ray.xyz.x
                  << " | y = " << ray.xyz.y << " | z = " << ray.xyz.z
                  << " | depth = " << depth << std::endl;
        return Point3D(ray.xyz.x, ray.xyz.y, ray.xyz.z);
    } else {
        std::cout
            << "k4a_calibration_2d_to_3d failed for the current input pixel!"
            << std::endl;
    }
    return Point3D(0, 0, 0);
}

int main(int argc, char* argv[]) {

    const char* input_path = argv[1];
    playback(input_path);

    return 0;
}

int playback(const char* input_path) {

    k4a::playback playback = k4a::playback::open(input_path);
    k4a::capture capture;

    k4a::image color;

    cv::Mat cv_rgb_image_with_alpha;
    cv::Mat cv_rgb_image_no_alpha;

    cv::namedWindow("color", cv::WINDOW_NORMAL);
    cv::namedWindow("depth", cv::WINDOW_NORMAL);

    ClickData* click_data = new ClickData();
    cv::setMouseCallback("color", CallBackFunc, click_data);
    cv::setMouseCallback("depth", CallBackFunc, click_data);

    playback.seek_timestamp(std::chrono::microseconds(4000000),
                            K4A_PLAYBACK_SEEK_BEGIN);

    while (true) {
    // for (int i = 0; i < 100; i++) {

        if (!playback.get_next_capture(&capture)) {
            std::cout << "Get next capture failed" << std::endl;
            break;
        }
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
                    uncompressed_color.get_width_pixels(), CV_8UC4,
                    uncompressed_color.get_buffer());

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
            transformed_depth_image.get_buffer(),
            static_cast<size_t>(transformed_depth_image.get_stride_bytes()));

        double min;
        double max;
        cv::minMaxIdx(cv_depth, &min, &max);
        cv::Mat adjMap;

        float scale = 255 / (max-min);
        cv_depth.convertTo(adjMap, CV_8UC1, scale, -min*scale);
        cv::Mat falseColorMap;
        cv::applyColorMap(adjMap, falseColorMap, cv::COLORMAP_JET);

        cv::Mat cv_depth_8U;
        cv::normalize(cv_depth, cv_depth_8U, 0, (1 << 16) - 1, cv::NORM_MINMAX);
        // cv_depth_8U.convertTo(cv_depth, CV_8U, 1);

        cv::imshow("color", cv_rgb_image_no_alpha);
        cv::imshow("depth", falseColorMap);


        // clear
        // color.reset();
        // uncompressed_color.reset();
        // capture.reset();
        // cv_rgb_image_with_alpha.release();
        // cv_rgb_image_no_alpha.release();
        click_data->n_clicks = 0;
        do {
            cv::Mat measure_image = cv_rgb_image_no_alpha.clone();

            if (click_data->n_clicks > 0) {
                cv::line(measure_image,
                         cv::Point(click_data->x1, click_data->y1),
                         cv::Point(click_data->x, click_data->y),
                         cv::Scalar(0, 0, 0), cv::LINE_4);
                Point3D p1 = convert_2d_depth_to_3d_point_cloud(
                    k4a_calibration, cv_depth, click_data->x1, click_data->y1);

                Point3D p2 = convert_2d_depth_to_3d_point_cloud(
                    k4a_calibration, cv_depth, click_data->x, click_data->y);

                std::cout << "\n############# | Distance: "
                          << p1.get_distance(p2) << std::endl;
                cv::imshow("color", measure_image);
            }
        } while (cv::waitKey(10) != 27);

        // if (cv::waitKey(25) == 'q')
        // break;
    }

    cv::destroyAllWindows();

    capture.reset();
    playback.close();

    return 0;
}

int capture() {

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

    cv::namedWindow("color", cv::WINDOW_NORMAL);
    cv::namedWindow("depth", cv::WINDOW_NORMAL);
    // cv::namedWindow( "ir", cv::WINDOW_NORMAL );

    while (true) {
        if (device.get_capture(&capture)) {
            rgb_image = capture.get_color_image();
            if (rgb_image) {
                cv_rgb_image_with_alpha = cv::Mat(
                    rgb_image.get_height_pixels(), rgb_image.get_width_pixels(),
                    CV_8UC4, (void*)rgb_image.get_buffer());
                if (cv_rgb_image_with_alpha.empty()) {
                    continue;
                }

                cv::cvtColor(cv_rgb_image_with_alpha, cv_rgb_image_no_alpha,
                             cv::COLOR_BGRA2BGR);
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
                        static_cast<size_t>(
                            transformed_depth_image.get_stride_bytes()));
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
            // cv::normalize(cv_ir_image, cv_ir_image_8U, 0, 1 << 16,
            // cv::NORM_MINMAX); cv::imshow("ir", cv_ir_image_8U);

            // k4a_transformation.destroy();
            k4a_image_release(rgb_image.handle());
            k4a_image_release(depth_image.handle());
            k4a_image_release(ir_image.handle());
            k4a_image_release(transformed_depth_image.handle());
            capture.reset();

            cv_rgb_image_no_alpha.release();
            cv_rgb_image_with_alpha.release();
            cv_depth.release();
            cv_depth_8U.release();
            cv_ir_image.release();
            cv_ir_image_8U.release();

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
