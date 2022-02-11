//
// Created by step305 on 25.07.2021.
//

#ifndef SLAM_LOGGER_SYNCTHREAD_H
#define SLAM_LOGGER_SYNCTHREAD_H

#include "thread"
#include "CircularQueue.h"
#include "ORBdetector.h"
#include "serialStream.h"
#include <stdlib.h>
#include "utils.h"
#include <stdint.h>
#include <chrono>
#include "hdf5.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <signal.h>

#define slam_queue_len      300
#define LOG_SLAM_VIDEO      false

typedef struct {
    float x;
    float y;
} PointCoordsType;

#pragma pack(1)
typedef struct {
    uint8_t val[32];
} DescriptorType;
#pragma pack()

class SyncPacket {
public:
    long long unsigned ts;
    cv::Mat frame;
    float adc[3];
    float dangle[3];
    std::vector<PointCoordsType> points;
    std::vector<DescriptorType> descriptors;
    bool sync;

    SyncPacket();
    void add_imu_data(float dangle_vals[3],  float adc_vals[3]);
    void add_features_data(cv::Mat descriptors, std::vector<cv::Point2f> features);
};

typedef struct {
    long long unsigned ts;
    cv::Mat frame;
    bool sync;
    int nmatched;
    std::vector<cv::Point2f>  matched;
    std::vector<cv::Point2f>  observed;
    float heading;
    float pitch;
    float roll;
    float bwx;
    float bwy;
    float bwz;
    float swx;
    float swy;
    float swz;
    float mwxy;
    float mwxz;
    float mwyx;
    float mwyz;
    float mwzx;
    float mwzy;
    float crh;
} SLAMLogMessageStruct;

extern std::atomic<bool> quitSync;
extern volatile sig_atomic_t exit_flag;
extern circ_queue::CircularFifo <SyncPacket,slam_queue_len> queueSLAM;
extern circ_queue::CircularFifo <SLAMLogMessageStruct,slam_queue_len> queueLogSLAM;

void syncThread();

#endif //SLAM_LOGGER_SYNCTHREAD_H
