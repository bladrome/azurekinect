#ifndef __MKVPARSER
#define __MKVPARSER

const int _debug_ = 0;

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
#include <filesystem>


cv::Mat
trans_to_8U(cv::Mat cv_image_16U)
{
    double min;
    double max;
    cv::Mat adjMap;
    cv::Mat cv_image_8U;

    // TODO: color mapping
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

class AzurePlayback
{
public:
    std::string filename;
    std::string serial_num;

    k4a::playback playback;
    k4a::capture capture;
    k4a::calibration calibration;
    k4a::transformation transformation;
    k4a_record_configuration_t configuration;

    k4a::image xy_table;
    k4a::image current_color;
    k4a::image current_depth;
    k4a::image current_ir;

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
    AzurePlayback(std::string filename, long long seektime = 3000000)
        : filename(filename)
    {
        playback = k4a::playback::open(filename.c_str());
        playback.seek_timestamp(std::chrono::microseconds(seektime),
                                K4A_PLAYBACK_SEEK_DEVICE_TIME);
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

    AzurePlayback(k4a::calibration calibration)
    {
        calibration = calibration;
        recored_length = std::chrono::microseconds(0);
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
        xy_table.reset();
        current_color.reset();
        current_depth.reset();
        current_ir.reset();
        transformation.destroy();
        capture.reset();
        playback.close();
    }

    void
    seek_device_timestamp(long long seektime = 3000000)
    {
        playback.seek_timestamp(std::chrono::microseconds(seektime),
                                K4A_PLAYBACK_SEEK_DEVICE_TIME);
    }

    int
    seek_capture_frames(long long index)
    {
        seek_device_timestamp(0);
        for (; index > 0; index--)
            next();

        return 0;
    }

    cv::Mat
    get_rgb()
    {
        cv::Mat cv_rgb;

        k4a::image color = capture.get_color_image();
        current_color = color;

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
        current_depth = depth;

        return depth;
    }

    cv::Mat
    get_cv_depth()
    {
        k4a::image depth = get_depth();

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
    get_cv_depth_8U()
    {
        return trans_to_8U(get_cv_depth());
    }

    float
    average_window_filter(const cv::Mat depth_image, int i, int j,
                          float distance_limit = 1700)
    {
        bool found = false;
        int window_size = 1;
        float depth = 0;
        height = depth_image.rows;
        width = depth_image.cols;
        while (found == false) {
            window_size *= 2;
            int step = window_size / 2;

            int x_lower = std::max(i - step, 0);
            int x_upper = std::min(i + step, height);

            int y_lower = std::max(j - step, 0);
            int y_upper = std::min(j + step, width);
            if (_debug_) {
                std::cout << "(" << x_lower << "," << x_upper << ")"
                          << "(" << y_lower << "," << y_upper << ")"
                          << std::endl;
            }

            float value = 0;
            int number = 0;

            for (int x = x_lower; x < x_upper; x++) {
                for (int y = y_lower; y < y_upper; y++) {
                    if (0 < depth_image.at<cv::uint16_t>(x, y) &&
                        depth_image.at<cv::uint16_t>(x, y) < distance_limit) {
                        found = true;
                        value += depth_image.at<cv::uint16_t>(x, y);
                        number += 1;
                    }
                }
            }
            if (found == true) {
                depth = value / number;
            }
        }
        return depth;
    }

    cv::Vec4d
    get_ground()
    {
        if (!current_depth.is_valid()) {
            get_depth();
        }
        k4a::image point_cloud = get_point_cloud_template();

        int point_count = generate_point_cloud(current_depth, point_cloud);

        int width = point_cloud.get_width_pixels();
        int height = point_cloud.get_height_pixels();

        k4a_float3_t *point_cloud_data = (k4a_float3_t *)
                                             point_cloud.get_buffer();

        std::vector<cv::Vec4d> raw_points(point_count);
        // std::cout << " " << point_count << std::endl;
        for (int i = 0; i < width * height; i++) {
            if (std::isnan(point_cloud_data[i].xyz.x) ||
                std::isnan(point_cloud_data[i].xyz.y) ||
                std::isnan(point_cloud_data[i].xyz.z)) {
                continue;
            }

            raw_points.push_back(cv::Vec4d{(float)point_cloud_data[i].xyz.x,
                                           (float)point_cloud_data[i].xyz.y,
                                           (float)point_cloud_data[i].xyz.z,
                                           1});
        }

        struct {
            bool
            operator()(cv::Vec4d a, cv::Vec4d b) const
            {
                return a.val[1] > b.val[1];
            }
        } Vec4dGreater;
        std::sort(raw_points.begin(), raw_points.end(), Vec4dGreater);
        if (_debug_) {
            for (auto i = 0; i < 10; ++i)
                std::cout << raw_points[i] << std::endl;
        }
        int groundsystemnumber = 50;
        cv::Mat GroundA(groundsystemnumber, 4, CV_64FC1);
        for (auto i = 0; i < groundsystemnumber; ++i) {
            for (auto j = 0; j < 4; ++j)
                GroundA.at<double>(i, j) = raw_points[i].val[j];
        }
        cv::Vec4d x;
        cv::SVD::solveZ(GroundA, x);
        std::cout << x << std::endl;

        return x;
    }

    k4a_float3_t
    get_xyz(float coordinate_x, float coordinate_y,
            cv::Mat depth_image = cv::Mat(0, 0, 0))
    {
        int coordinate_x_int = static_cast<int>(coordinate_x);
        int coordinate_y_int = static_cast<int>(coordinate_y);
        float depth;
        cv::Mat trans_depth_image;
        if (depth_image.empty()) {
            trans_depth_image = get_cv_depth();
            // TODO: filter methods
            // depth = trans_depth_image.at<uint16_t>(coordinate_x_int,
            //                                        coordinate_y_int);
            depth = average_window_filter(trans_depth_image, coordinate_x_int,
                                          coordinate_y_int);
        }
        k4a_float3_t ray{0};
        k4a_float2_t point_2d;

        point_2d.xy.x = coordinate_x;
        point_2d.xy.y = coordinate_y;

        if (_debug_) {
            std::cout << coordinate_x_int << " " << coordinate_y_int << " "
                      << depth_image.cols << " " << depth_image.rows
                      << std::endl;
        }
        if (calibration.convert_2d_to_3d(point_2d, depth,
                                         K4A_CALIBRATION_TYPE_COLOR,
                                         K4A_CALIBRATION_TYPE_COLOR, &ray)) {
            if (_debug_) {
                std::cout << "2dx = " << coordinate_x_int
                          << " | 2dy = " << coordinate_y_int
                          << " | x = " << ray.xyz.x << " | y = " << ray.xyz.y
                          << " | z = " << ray.xyz.z << " | depth = " << depth
                          << std::endl;
            }
        } else {
            std::cout << "k4a_calibration_2d_to_3d failed for the current "
                         "input pixel!"
                      << std::endl;
        }
        return ray;
    }

    float
    get_distance(int x1, int y1, int x2, int y2,
                 cv::Mat depth_image = cv::Mat(0, 0, 0))
    {
        k4a_float3_t p1 = get_xyz(x1, y1, depth_image);
        if (x2 > 0 && y2 > 0) {
            k4a_float3_t p2 = get_xyz(x2, y2, depth_image);
            return std::sqrt(std::pow(p1.xyz.x - p2.xyz.x, 2) +
                             std::pow(p1.xyz.y - p2.xyz.y, 2) +
                             std::pow(p1.xyz.z - p2.xyz.z, 2));
        }
        cv::Vec4d ground = get_ground();

        return (p1.xyz.x * ground.val[0] + p1.xyz.y * ground.val[1] +
                p1.xyz.y * ground.val[2] + ground.val[3]);
    }

    cv::Mat
    get_ir()
    {
        k4a::image ir_image = capture.get_ir_image();
        current_ir = ir_image;
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
        return playback.get_next_capture(&capture);
    }

    int
    export_all(std::string outdir = "output/", int rate = 1)
    {

        std::size_t botDirPos = filename.find_last_of("/");
        std::string dirname = filename.substr(0, botDirPos);
        std::string basename = filename.substr(botDirPos, filename.length());
        basename = basename.substr(0, basename.find_last_of("."));

        outdir = dirname + basename;
        if (! std::filesystem::exists(outdir)){
            std::filesystem::create_directory(outdir);
        }
        std::cout << basename << std::endl;
        std::cout << dirname << std::endl;

        int n_zero = 5;
        long unsigned int count;
        for (count = 1; next(); count += rate) {

            cv::Mat color;
            try {
                color = get_rgb();
            }
            catch (const k4a::error & e) {
                // std::cerr << e.what();
                continue;
            }

            std::string strcount = std::to_string(count);
            auto index = std::string(n_zero - strcount.length(), '0') + strcount;
            auto filename = outdir + basename + "_color_" + index + ".jpg";
            cv::imwrite(filename, color);
            std::cout << filename << std::endl;

            // depth
            cv::Mat depth = get_cv_depth();
            filename = outdir + basename + "_depth_" + index + ".tif";
            cv::imwrite(filename, depth);
            std::cout << filename << std::endl;
        }
        return count;
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
                float x = xy_table_data[i].xy.x * (float)depth_data[i];
                float y = xy_table_data[i].xy.y * (float)depth_data[i];
                float z = (float)depth_data[i];
                // remove background
                if (z > 1600 || y < -534) {
                    continue;
                }

                point_cloud_data[i].xyz.x = x;
                point_cloud_data[i].xyz.y = y;
                point_cloud_data[i].xyz.z = z;

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
    get_point_cloud_template()
    {
        return k4a::image::create(
            K4A_IMAGE_FORMAT_CUSTOM,
            calibration.depth_camera_calibration.resolution_width,
            calibration.depth_camera_calibration.resolution_height,
            calibration.depth_camera_calibration.resolution_width *
                (int)sizeof(k4a_float3_t));
    }

    // TODO: Fix Path
    int
    export_point_cloud(std::string outprefix = "./")
    {
        k4a::image point_cloud = get_point_cloud_template();

        for (int count = 0; next(); count += 1) {
            int point_count = generate_point_cloud(get_depth(), point_cloud);

            std::string outfilename =
                filename + "_raw_point_cloud_" +
                std::to_string(current_depth.get_device_timestamp().count()) +
                "_" + std::to_string(count) + ".ply";
            std::cout << outfilename << std::endl;
            std::cout << configuration.start_timestamp_offset_usec << std::endl;

            write_point_cloud(outfilename.c_str(), point_cloud, point_count);
        }
        return 0;
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

void
mkv_gen_cloud(std::string filename, std::string outputfilename)
{
    AzurePlayback apb(filename.c_str());

    k4a::image point_cloud = apb.get_point_cloud_template();
    int point_count = apb.generate_point_cloud(apb.get_depth(), point_cloud);
    apb.write_point_cloud(outputfilename.c_str(), point_cloud, point_count);
}

float
mkv_get_distance(std::string mkvfilename, long long seektime, int x1, int y1,
                 int x2, int y2)
{
    AzurePlayback apb(mkvfilename);
    if (seektime > 100000) {
        apb.seek_device_timestamp(seektime);
    } else {
        apb.seek_capture_frames(seektime);
    }

    return apb.get_distance(x1, y1, x2, y2);
}

k4a::calibration
get_calibration(std::string calibrationfilename = "calibration.json")
{
    std::ifstream in(calibrationfilename);
    std::string contents((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());

    k4a::calibration calibration = k4a::calibration::get_from_raw(
        (char *)contents.c_str(),
        contents.size() + 1,  // add null to buffer
        K4A_DEPTH_MODE_WFOV_UNBINNED, K4A_COLOR_RESOLUTION_1080P);

    return calibration;
}

// TODO
float
mkv_get_distance(cv::Mat depth_image, int x1, int y1, int x2, int y2,
                 std::string calibrationfilename = "calibration.json")

{
    k4a::calibration calibration = get_calibration(calibrationfilename);
    AzurePlayback apb(calibration);
    return apb.get_distance(x1, y1, x2, y2, depth_image);
}

#endif
