//
// Created by driver on 09.02.2022.
//

#include "SLAM_algo_beauty.h"

double MOTION_DTHE_LIMIT = 5e-3;

double sys_coeff = 1.0;
double dtSample = 1/98.4;



coder::array<unsigned char, 2U> frame_descriptors;
coder::array<double, 2U> frame_points;
SLAMALGO::map_struct_type map;
coder::array<double, 2U> P;
double bw[3] = {0.0,};
double q[4] = {0.0,};
double qc[4] = {0.0,};
double mw[6] = {0.0,};
double occupancy_map[1296] = {0.0,};
double sw[3] = {0.0,};
SLAMALGO::caminfo_struct_type cam_info;
SLAMALGO::frame_struct_type frame;
bool motion_flag = false;
float adc_val = 0.0f;

void SLAM_algo_init() {
    cam_info.fc[0] = 386.0949176f;
    cam_info.fc[1] = 385.85893353f;
    cam_info.cc[0] = 319.69237133f;
    cam_info.cc[1] = 240.46086919f;
    cam_info.kc[0] = 0.0103592f;
    cam_info.kc[1] = -0.06719747f;
    cam_info.kc[2] = -0.001601f;
    cam_info.kc[3] = 0.00155616f;
    cam_info.kc[4] = 0.19119122f;

/*    cam_info.cc[0] = 319.69237133;
    cam_info.cc[1] = 240.46086919;
    cam_info.fc[0] = 386.0949176;
    cam_info.fc[1] = 385.85893353;
    cam_info.kc[0] = 0.0103592;
    cam_info.kc[1] = -0.06719747;
    cam_info.kc[2] = -0.001601;
    cam_info.kc[3] = 0.00155616;
    cam_info.kc[4] = 0.19119122;*/
    cam_info.frame_rate = 1/5.0;
    cam_info.nRows = 480;
    cam_info.nCols = 640;

    P.set_size(15, 15);
    P[0 + 0*15] = 1e-20;
    P[1 + 1*15] = 1e-20;
    P[2 + 2*15] = 1e-20;
    P[3 + 3*15] = 1e-4;
    P[4 + 4*15] = 1e-4;
    P[5 + 5*15] = 1e-4;
    P[6 + 6*15] = 1e-4;
    P[7 + 7*15] = 1e-4;
    P[8 + 8*15] = 1e-4;
    P[9 + 9*15] = 1e-7;
    P[10 + 10*15] = 1e-7;
    P[11 + 11*15] = 1e-7;
    P[12 + 12*15] = 1e-7;
    P[13 + 13*15] = 1e-7;
    P[14 + 14*15] = 1e-7;

    q[0] = 1.0;

    map.size = 0;
    coder::array<SLAMALGO::feature_struct_type, 1U> temp;
    temp.set_size(2);
    for (int idx0 = 0; idx0 < temp.size(0); idx0++) {
        temp[idx0].ceil_coord[0] = 0.0;
        temp[idx0].ceil_coord[1] = 0.0;
        temp[idx0].cnt_mat = 0.0;
        temp[idx0].cnt_obs = 0.0;
        for (int idx1 = 0; idx1 < 32; idx1++) {
            temp[idx0].des[idx1] = 0;
        }
        temp[idx0].eb[0] = 0.0;
        temp[idx0].eb[1] = 0.0;
        temp[idx0].eb[2] = 0.0;
        temp[idx0].en[0] = 0.0;
        temp[idx0].en[1] = 0.0;
        temp[idx0].en[2] = 0.0;
        temp[idx0].inv = 0.0;
        temp[idx0].pos[0] = 0.0;
        temp[idx0].pos[1] = 0.0;
        temp[idx0].pos[2] = 0.0;
        temp[idx0].uv[0] = 0.0;
        temp[idx0].uv[1] = 0.0;
    }
}

void SLAM_algo_process(SyncPacket packet, double angles[3], float *crh_out, double bias[3], int *matched_cnt,
                       std::vector<cv::Point2f>  &observed, std::vector<cv::Point2f>  &matched) {
    if (packet.sync) {
        int num_features = packet.descriptors.size();
        frame_descriptors.clear();
        frame_points.clear();
        frame_descriptors.set_size(num_features, 32);
        frame_points.set_size(num_features, 2);
        for (int idx0 = 0; idx0 < num_features; idx0++) {
            for (int idx1 = 0; idx1 < 32; idx1++) {
                frame_descriptors[idx0 + num_features * idx1] = packet.descriptors[idx0].val[idx1];
            }
            frame_points[idx0] = packet.points[idx0].x;
            frame_points[idx0 + num_features] = packet.points[idx0].y;
        }
    } else {
        frame.dthe[0] = -packet.dangle[1] * sys_coeff;
        frame.dthe[1] = packet.dangle[2] * sys_coeff;
        frame.dthe[2] = -packet.dangle[0] * sys_coeff;
    }
    adc_val = packet.adc[0];
    *crh_out = adc_val;

    double abs_dthe =  std::sqrt(frame.dthe[0]*frame.dthe[0] +
            frame.dthe[1]*frame.dthe[1] +
            frame.dthe[2]*frame.dthe[2]);
    if (abs_dthe > MOTION_DTHE_LIMIT) {
        motion_flag = true;
    } else {
        motion_flag = false;
    }

    frame.sync = packet.sync;
    SLAMALGO::vector_slam_gyro_data(&frame, frame_descriptors, frame_points, &map, q,
                                    P, bw, sw, mw, dtSample, &cam_info,
                                    occupancy_map, motion_flag);
    qc[0] = q[0];
    qc[1] = -q[1];
    qc[2] = -q[2];
    qc[3] = -q[3];
    SLAMALGO::quat_angle(qc, &angles[0], &angles[1], &angles[2]);
    angles[0] *= 180.0f/3.14159265;
    angles[1] *= 180.0f/3.14159265;
    angles[2] *= 180.0f/3.14159265;

    bias[0] = bw[0]*180/3.14159265*3600;
    bias[1] = bw[1]*180/3.14159265*3600;
    bias[2] = bw[2]*180/3.14159265*3600;

    int match_cnt = 0;
    double u, v;
    if (packet.sync) {
        for (int idx0 = 0; idx0 < map.size; idx0++) {
            if (map.data[idx0].mat > 0.2) {
                match_cnt++;
                u = map.data[idx0].uv[0];
                v = map.data[idx0].uv[1];
                matched.push_back(cv::Point2f(u, v));
            }
            if (map.data[idx0].obs > 0.2) {
                u = map.data[idx0].uv[0];
                v = map.data[idx0].uv[1];
                observed.push_back(cv::Point2f(u, v));
            }
            map.data[idx0].mat = 0.0;
            map.data[idx0].obs = 0.0;
            map.data[idx0].inv = 0.0;
        }
        *matched_cnt = match_cnt;
    }
}
