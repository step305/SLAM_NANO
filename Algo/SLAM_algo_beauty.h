//
// Created by driver on 09.02.2022.
//

#ifndef SLAM_NANO_SLAM_ALGO_BEAUTY_H
#define SLAM_NANO_SLAM_ALGO_BEAUTY_H
#include <cstddef>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include "rtwtypes.h"
#include "vector_slam_gyro_data_types.h"
#include "vector_slam_gyro_data.h"
#include "../syncThread.h"
#include "quat_angle_types.h"
#include "quat_angle.h"

extern double bw[3];
extern SLAMALGO::map_struct_type map;
extern double sw[3];
extern double mw[6];

void SLAM_algo_init();
void SLAM_algo_process(SyncPacket packet, double angles[3], float *crh_out, double bias[3], int *matched_cnt,
                       std::vector<cv::Point2f>  &observed, std::vector<cv::Point2f>  &matched);

#endif //SLAM_NANO_SLAM_ALGO_BEAUTY_H
