//
// Created by driver on 09.09.2021.
//

#include "SLAM_thread.h"

double mean_arr(double* arr, int len) {
    double sum  = 0.0f;
    for(int i = 0; i< len; i++) {
        sum += arr[i];
    }
    return sum/(double)len;
}

int SLAMThread() {
    SLAM_algo_init();

    //Angles
    double angles[3];
    double bias[3];
    /*double bias_x[100] = {0.0f,};
    double bias_y[100] = {0.0f,};
    double bias_z[100] = {0.0f,};
    int bias_hist_i = 0;
    int bias_hist_max = 30;*/

    long long unsigned t0_slam, t1_slam;
    int fps_slam_cnt = 0;
    float fps_slam = 0.0f;
    float adc_val = 0.0;

    t0_slam = get_us();

    unsigned long long t0, t1;
    t0 = get_us();

    unsigned long long t0_watchdog, t1_watchdog;
    t0_watchdog = get_us();

    int match_cnt = 0;

    cv::Mat emptyFrameOther;
    cv::Mat frame(640, 480, cv::DataType<float>::type);

    std::cout << color_fmt_red << "SLAMThread:: Started!" << color_fmt_reset << std::endl;
    while (!quitSLAM) {
        try {
            SyncPacket packet = SyncPacket();

            if (queueSLAM.pop(packet)) {

                if (fps_slam_cnt == FPS_MAX_CNT) {
                    t1_slam = get_us();
                    fps_slam = (float) fps_slam_cnt / (float) (t1_slam - t0_slam) * 1.0e6f;
                    t0_slam = get_us();
                    fps_slam_cnt = 0;
                    std::cout << color_fmt_red << "SLAMThread:: SLAM FPS = " << std::fixed << std::setprecision(2)
                              << fps_slam << "fps" << color_fmt_reset << std::endl;
                } else {
                    fps_slam_cnt++;
                };

                t0 = get_us();

                int matched = 0;
                float temp_val;

                std::vector <cv::Point2f> observed_points;
                std::vector <cv::Point2f> matched_points;
                matched_points.reserve(50);
                observed_points.reserve(50);

                SLAM_algo_process(packet, angles, &temp_val, bias, &matched, observed_points, matched_points);
                if (!packet.sync) {
                    adc_val = packet.adc[0] / 0.01 * 3600;
                }

                if (packet.sync) {
                    match_cnt = matched;
                }


                /*            if (!packet.sync) {
                                double abs_dthe =  std::sqrt(packet.dangle[0]*packet.dangle[0] +
                                                             packet.dangle[1]*packet.dangle[1] +
                                                             packet.dangle[2]*packet.dangle[2]);
                                if (abs_dthe < 1.7e-4) {
                                    steady = true;
                                } else {
                                    steady = false;
                                }
                            }*/

                /*   if (packet.sync) {
                       if (!packet.descriptors.empty()) {
                           t0_watchdog = get_us();
                           if (steady) {
                               bias_x[bias_hist_i] = bias[0];
                               bias_y[bias_hist_i] = bias[1];
                               bias_z[bias_hist_i] = bias[2];
                               if (bias_hist_i < (bias_hist_max-1)) {
                                   bias_hist_i++;
                               } else {
                                   bias_hist_i = 0;
                               }
                           }
                       }
                   }*/

                /*t1_watchdog = get_us();
                if ((t1_watchdog - t0_watchdog) > 1000000) {
                    bw[0] = mean_arr(bias_x, bias_hist_max)/180*3.14159265/3600;
                    bw[1] = mean_arr(bias_y, bias_hist_max)/180*3.14159265/3600;
                    bw[2] = mean_arr(bias_z, bias_hist_max)/180*3.14159265/3600;
                }*/

                if (packet.sync) {
                    t1 = get_us();
                }

                if (packet.sync) {
                    std::cout << color_fmt_red << "SLAMThread:: next: " << (t1 - t0) << "us"
                              << " map = " << map.size
                              << " matched = " << match_cnt
                              << color_fmt_reset << std::endl;
                    frame = packet.frame;
                } else {
                    frame = emptyFrameOther;
                }

                SLAMLogMessageStruct msgREC = {packet.ts,
                                               frame,
                                               packet.sync,
                                               match_cnt,
                                               matched_points,
                                               observed_points,
                                               (float) angles[0], (float) angles[1], (float) angles[2],
                                               (float) bias[0], (float) bias[1], (float) bias[2],
                                               (float) sw[0], (float) sw[1], (float) sw[2],
                                               (float) mw[0], (float) mw[1], (float) mw[2],
                                               (float) mw[3], (float) mw[4], (float) mw[5],
                                               adc_val};

                if (!queueLogSLAM.push(msgREC)) {
                    std::cout << color_fmt_red << "SLAMThread:: Error!::" << "Log Queue full!" << color_fmt_reset
                              << std::endl;
                    exit_flag = 1;
                    quitSLAM = true;
                    break;
                }
                if (!queueFIFOSLAM.push(msgREC)) {
                    // std::cout << color_fmt_red << "SLAMThread:: Warning!::" << "FIFO Queue full! Skipped data!" << color_fmt_reset << std::endl;
                }
            } // if packet
        }
        catch (int e) {
            std::cout << "SLAM thread exception: " << e << std::endl;
            abort();
        }
    } // while
    SLAMLogMessageStruct msgREC;
    queueLogSLAM.push(msgREC);
    queueFIFOSLAM.push(msgREC);
    std::cout << color_fmt_red << "SLAMThread:: Finished!" << color_fmt_reset << std::endl;
    return 0;
}
