//
// Created by driver on 18.10.2021.
//

#include "fifo_thread.h"

typedef unsigned char uchar;

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    auto buf = std::make_unique<char[]>( size );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

void pack_data(SLAMLogMessageStruct &data,
               std::string &json) {
    std::string encoded_frame_str;
    if (!data.frame.empty()) {
        std::vector<uchar> buf;

        if (data.matched.size() > 0) {
            for (const auto &pt : data.matched)
                cv::drawMarker(data.frame, pt, cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 2, 1, 8);
        }

        if (data.observed.size() > 0) {
            for (const auto &pt : data.observed)
                cv::drawMarker(data.frame, pt, cv::Scalar(255, 0, 0), cv::MARKER_SQUARE, 2, 1, 8);
        }

        cv::imencode(".jpg", data.frame, buf);
        auto *encoded_frame = reinterpret_cast<unsigned char*>(buf.data());
        encoded_frame_str = base64_encode(encoded_frame, buf.size());
    } else {
        encoded_frame_str = "None";
    }

    json = "{";
    json = string_format("%s \"adc\": %0.3f,", json.c_str(), data.crh);
    json = string_format("%s \"yaw\": %0.3f,", json.c_str(), data.heading);
    json = string_format("%s \"pitch\": %0.3f,", json.c_str(), data.pitch);
    json = string_format("%s \"roll\": %0.3f,", json.c_str(), data.roll);
    json = string_format("%s \"nmatched\": %d,", json.c_str(), data.nmatched);
    json = string_format("%s \"bw\": [%0.3f, %0.3f, %0.3f], ", json.c_str(), data.bwx, data.bwy, data.bwz);
    json = string_format("%s \"frame\": \"%s\"", json.c_str(), encoded_frame_str.c_str());
    json = string_format("%s}\n", json.c_str());
    //json = string_format("%20ld :: %s", strlen(json.c_str()), json.c_str());
}

void fifoThread() {
    SLAMLogMessageStruct packet_slam;
    long long unsigned t0, t1;
    int fps_cnt = 0;
    float fps = 0.0f;

    char filename[] = "/home/step305/SLAM_FIFO.tmp";

    std::cout << color_fmt_green << "fifoThread:: Connected" << color_fmt_reset << std::endl;

    int s_fifo = mkfifo(filename, S_IRWXU);
    if (s_fifo != 0)
    {
        std::cout << color_fmt_green << "mkfifo() error: "
                  << s_fifo << color_fmt_reset << std::endl;
        quitFIFO = true;
        exit_flag = 1;
    }
    FILE * wfd = fopen(filename, "w");
    if (wfd < 0)
    {
        std::cout << color_fmt_green << "open() error: "
                  << wfd << color_fmt_reset << std::endl;
        quitFIFO = true;
        exit_flag = 1;
    }

    while (!quitFIFO) {
        try {
            if (queueFIFOSLAM.pop(packet_slam)) {
                if (fps_cnt == FPS_MAX_CNT) {
                    t1 = get_us();
                    fps = (float) fps_cnt / (float) (t1 - t0) * 1.0e6f;
                    t0 = get_us();
                    fps_cnt = 0;
                    std::cout << color_fmt_green << "fifoThread:: SLAM FIFO in FPS = " << std::fixed
                              << std::setprecision(2)
                              << fps << "fps" << color_fmt_reset << std::endl;
                } else {
                    fps_cnt++;
                };

                std::string json;
                pack_data(packet_slam, json);
                char const *out_buffer = json.c_str();
                try {
                    int s_write = fprintf(wfd, "%s", out_buffer);
                    fflush(wfd);
                }
                catch (int e) {
                    quitFIFO = true;
                    exit_flag = 1;
                    break;
                }
            }
        }
        catch (int e) {
            std::cout << "fifo thread exception: " << e << std::endl;
            abort();
        }
    }
    fclose(wfd);
    unlink(filename);
}
/*
void fifoThread() {
    SLAMLogMessageStruct packet_slam;

    long long unsigned t0, t1;
    int fps_cnt = 0;
    float fps = 0.0f;

    char filename[] = "/home/step305/SLAM_FIFO.tmp";

    int s_fifo = mkfifo(filename, S_IRWXU);
    if (s_fifo != 0)
    {
        std::cout << color_fmt_green << "mkfifo() error: "
                  << s_fifo << color_fmt_reset << std::endl;
        quitFIFO = true;
        exit_flag = 1;
    }
    FILE * wfd = fopen(filename, "w");
    if (wfd < 0)
    {
        std::cout << color_fmt_green << "open() error: "
                  << wfd << color_fmt_reset << std::endl;
        quitFIFO = true;
        exit_flag = 1;
    }

    while (!quitFIFO) {
        if (queueFIFOSLAM.pop(packet_slam)) {
            if (fps_cnt == FPS_MAX_CNT) {
                t1 = get_us();
                fps = (float) fps_cnt / (float) (t1 - t0) * 1.0e6f;
                t0 = get_us();
                fps_cnt = 0;
                std::cout << color_fmt_green << "fifoThread:: SLAM FIFO in FPS = " << std::fixed << std::setprecision(2)
                          << fps << "fps" << color_fmt_reset << std::endl;
            } else {
                fps_cnt++;
            };

            char json[4096] = {0,};

            sprintf(json, "{ \"heading\": %0.4f, ", packet_slam.heading);
            sprintf(json, "%s\"roll\": %0.4f, ", json, packet_slam.roll);
            sprintf(json, "%s\"pitch\": %0.4f, ", json, packet_slam.pitch);
            sprintf(json, "%s\"bw\": [%0.3f, %0.3f, %0.3f], ", json, packet_slam.bwx,
                    packet_slam.bwy, packet_slam.bwz);
            sprintf(json, "%s\"crh\": %0.6f, ", json, packet_slam.crh);
            sprintf(json, "%s\"nmatched\": %d }\n", json, packet_slam.nmatched);
            try {
                int s_write = fprintf(wfd, "%s", json);
                fflush(wfd);
            }
            catch (int e) {
                quitFIFO = true;
                exit_flag = 1;
                break;
            }
        }
    }
    fclose(wfd);
    unlink(filename);
}
*/
