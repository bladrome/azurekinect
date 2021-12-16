#include <chrono>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <k4a/k4atypes.h>
#include <string>
#include <thread>
#include <cmath>

#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <turbojpeg.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <turbojpeg.h>

class AzurePlayback
{
    std::string filename;
    std::string serial_num;

    k4a::playback playback;
    k4a::capture capture;
    k4a::calibration calibration;
    k4a::transformation transformation;
    k4a_record_configuration_t configuration;

    k4a::image xy_table;

    int width;
    int height;
    std::chrono::microseconds recored_length;

    void
    create_xy_table(const k4a::calibration &calibration)
    {
        k4a_float2_t *table_data = (k4a_float2_t *)xy_table.get_buffer();

        int width = calibration.depth_camera_calibration.resolution_width;
        int height = calibration.depth_camera_calibration.resolution_height;

        k4a_float2_t p;
        k4a_float3_t ray;
        int valid;

        for (int y = 0, idx = 0; y < height; y++) {
            p.xy.y = (float)y;
            for (int x = 0; x < width; x++, idx++) {
                p.xy.x = (float)x;

                valid = calibration.convert_2d_to_3d(
                    p, 1, K4A_CALIBRATION_TYPE_DEPTH,
                    K4A_CALIBRATION_TYPE_DEPTH, &ray);
                if (valid) {
                    table_data[idx].xy.x = ray.xyz.x;
                    table_data[idx].xy.y = ray.xyz.y;
                } else {
                    table_data[idx].xy.x = nanf("");
                    table_data[idx].xy.y = nanf("");
                }
            }
        }
    }

   public:
    AzurePlayback(std::string filename)
    {
        playback = k4a::playback::open(filename.c_str());
        playback.seek_timestamp(std::chrono::microseconds(4000000),
                                K4A_PLAYBACK_SEEK_BEGIN);
        playback.get_next_capture(&capture);
        calibration = playback.get_calibration();
        recored_length = playback.get_recording_length();
        configuration = playback.get_record_configuration();
        xy_table = k4a::image::create(
            K4A_IMAGE_FORMAT_CUSTOM,
            calibration.depth_camera_calibration.resolution_width,
            calibration.depth_camera_calibration.resolution_height,
            calibration.depth_camera_calibration.resolution_width *
                (int)sizeof(k4a_float2_t));

        create_xy_table(calibration);
    }

    ~AzurePlayback()
    {
        transformation.destroy();
        capture.reset();
        playback.close();
    }

    cv::Mat
    trans_to_8U(cv::Mat cv_image_16U)
    {
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

    cv::Mat
    get_rgb()
    {
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
                    std::cout << "Failed to decompressed color frame"
                              << std::endl;
                    exit(1);
                }
                if (tjDestroy(tjHandle)) {
                    std::cout << "Failed to destroy turboJPEG handle"
                              << std::endl;
                    exit(1);
                }

                cv::Mat cv_color = cv::Mat(
                    uncompressed_color.get_height_pixels(),
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

    k4a::image
    get_depth()
    {
        k4a::image depth = capture.get_depth_image();
        transformation = k4a::transformation(calibration);
        k4a::image transformed_depth_image =
            transformation.depth_image_to_color_camera(depth);

        return depth;
    }

    cv::Mat
    get_cv_depth()
    {
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

    cv::Mat
    get_depth_8U()
    {
        return trans_to_8U(get_cv_depth());
    }

    cv::Mat
    get_ir()
    {
        k4a::image ir_image = capture.get_ir_image();

        cv::Mat cv_ir = cv::Mat(
            ir_image.get_height_pixels(), ir_image.get_width_pixels(), CV_16U,
            ir_image.get_buffer(),
            static_cast<size_t>(ir_image.get_stride_bytes()));

        return cv_ir;
    }

    cv::Mat
    get_ir_8U()
    {
        return trans_to_8U(get_ir());
    }

    float
    get_temperature_c()
    {
        return capture.get_temperature_c();
    }

    bool
    next()
    {
        playback.get_next_capture(&capture);
        return capture.is_valid();
    }

    int
    export_all(std::string outdir = "output/")
    {
        std::cout << outdir << std::endl;
        std::string filename;
        for (unsigned long long i = 0; next(); i += 1) {
            cv::Mat color = get_rgb();
            filename = outdir + "color_" + std::to_string(i) + ".jpeg";
            cv::imwrite(filename, color);

            cv::Mat depth = get_cv_depth();
            filename = outdir + "depth_" + std::to_string(i) + ".jpeg";
            cv::imwrite(filename, depth);
        }
        return 0;
    }

    int
    generate_point_cloud(const k4a::image &depth_image, k4a::image &point_cloud)
    {
        int width = depth_image.get_width_pixels();
        int height = depth_image.get_height_pixels();
        int point_count = 0;

        uint16_t *depth_data = (uint16_t *)depth_image.get_buffer();
        k4a_float2_t *xy_table_data = (k4a_float2_t *)xy_table.get_buffer();
        k4a_float3_t *point_cloud_data = (k4a_float3_t *)
                                             point_cloud.get_buffer();

        for (int i = 0; i < width * height; i++) {
            if (depth_data[i] != 0 && !std::isnan(xy_table_data[i].xy.x) &&
                !std::isnan(xy_table_data[i].xy.y)) {
                point_cloud_data[i].xyz.x = xy_table_data[i].xy.x *
                                            (float)depth_data[i];
                point_cloud_data[i].xyz.y = xy_table_data[i].xy.y *
                                            (float)depth_data[i];
                point_cloud_data[i].xyz.z = (float)depth_data[i];
                point_count++;
            } else {
                point_cloud_data[i].xyz.x = std::nanf("");
                point_cloud_data[i].xyz.y = std::nanf("");
                point_cloud_data[i].xyz.z = std::nanf("");
            }
        }

        return point_count;
    }

    int
    write_point_cloud(const char *file_name, const k4a::image &point_cloud,
                      int point_count)
    {
        int width = point_cloud.get_width_pixels();
        int height = point_cloud.get_height_pixels();

        k4a_float3_t *point_cloud_data = (k4a_float3_t *)
                                             point_cloud.get_buffer();

        // save to the ply file
        std::ofstream ofs(file_name);  // text mode first
        ofs << "ply" << std::endl;
        ofs << "format ascii 1.0" << std::endl;
        ofs << "element vertex"
            << " " << point_count << std::endl;
        ofs << "property float x" << std::endl;
        ofs << "property float y" << std::endl;
        ofs << "property float z" << std::endl;
        ofs << "end_header" << std::endl;
        ofs.close();

        std::stringstream ss;
        for (int i = 0; i < width * height; i++) {
            if (std::isnan(point_cloud_data[i].xyz.x) ||
                std::isnan(point_cloud_data[i].xyz.y) ||
                std::isnan(point_cloud_data[i].xyz.z)) {
                continue;
            }

            ss << (float)point_cloud_data[i].xyz.x << " "
               << (float)point_cloud_data[i].xyz.y << " "
               << (float)point_cloud_data[i].xyz.z << std::endl;
        }

        std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
        ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());

        return 0;
    }

    k4a::image
    get_point_cloud()
    {
        return k4a::image::create(
            K4A_IMAGE_FORMAT_CUSTOM,
            calibration.depth_camera_calibration.resolution_width,
            calibration.depth_camera_calibration.resolution_height,
            calibration.depth_camera_calibration.resolution_width *
                (int)sizeof(k4a_float3_t));
    }
};

void
mkv_show(std::string filename)
{
    AzurePlayback apb(filename.c_str());
    cv::namedWindow("color", cv::WINDOW_NORMAL);
    cv::Mat color = apb.get_rgb();
    std::cout << color.cols << " " << color.rows << std::endl;
    // std::cout << "exportall: " << apb.export_all() << std::endl;

    while (apb.next()) {
        cv::Mat color = apb.get_rgb();
        cv::imshow("color", color);

        if (cv::waitKey(10) == 27)
            break;
    }

    color.release();
    cv::destroyAllWindows();
}


void mkv_gen_cloud(std::string filename){
    AzurePlayback apb(filename.c_str());

    k4a::image point_cloud = apb.get_point_cloud();
    int point_count = apb.generate_point_cloud(apb.get_depth(), point_cloud);
    apb.write_point_cloud("output.ply", point_cloud, point_count);
}

int
main(int argc, char *argv[])
{
    std::string filename("/home/bladrome/jack/day06/877777.mkv");
    mkv_show(filename);

    return 0;
}