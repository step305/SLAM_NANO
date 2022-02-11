//
// Created by driver on 09.09.2021.
//

#ifndef SLAM_LOGGER_SLAM_THREAD_H
#define SLAM_LOGGER_SLAM_THREAD_H

#include "syncThread.h"
#include "utils.h"
#include <signal.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include "fifo_thread.h"
#include "Algo/SLAM_algo_beauty.h"

extern std::atomic<bool> quitSLAM;
extern volatile sig_atomic_t exit_flag;

int SLAMThread();

#endif //SLAM_LOGGER_SLAM_THREAD_H
