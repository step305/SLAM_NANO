//
// Created by step305 on 25.07.2021.
//

#include "ORBdetector.h"

//#define CALIBR_CAMERA

// IR camera
cv::Mat camMtx = (cv::Mat1d(3, 3) << 386.0949176, 0, 319.69237133, 0, 385.85893353, 240.46086919, 0, 0, 1);
cv::Mat distCoefs = (cv::Mat1d(1, 5) << 0.0103592, -0.06719747, -0.001601, 0.00155616, 0.19119122);

//RGB camera
//cv::Mat camMtx = (cv::Mat1d(3, 3) << 377.636383056641, 0, 313.422821044922, 0, 377.265106201172, 242.470718383789, 0, 0, 1);
//cv::Mat distCoefs = (cv::Mat1d(1, 5) << -0.0598541013896465, 0.0736653730273247, -0.000628226727712899, 0.000477661902550608, -0.0230451133102179);

int const npoints = 1000;
const float  threshold_close = 8.0f;
const int  max_keypoints = 100;

bool response_comparator(const cv::KeyPoint& p1, const cv::KeyPoint& p2)
{
    return p1.response > p2.response;
}

int ORBdetectorStreamThread() {
    long long unsigned t0, t1;
    int fps_cnt;
    float fps;

    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << color_fmt_yellow << "ORBThread:: Started!" << color_fmt_reset << std::endl;

    cv::Mat empty_frame;

    std::cout << color_fmt_yellow << "ORBThread:: ORB detector thread started." << color_fmt_reset << std::endl;

    cv::Ptr<cv::cuda::ORB> orb_detector = cv::cuda::ORB::create(npoints);
    cv::Ptr<cv::ORB> orb_detector_cpu = cv::ORB::create(npoints);

    t0 = get_us();
    fps_cnt = 0;

#ifdef CALIBR_CAMERA
    int img_cnt = 0;
    int skip_cnt = 0;
    char nam_img[] = "calibr/img_000000000000.bmp";
#endif
    int frames_cnt = 0;
    int skip_frames = 0;

    while (!quitCamera) {
        try {
            ImageMessageStruct image_msg;
            if (!queueImages.pop(image_msg)) {
                continue;
            }
            cv::Mat frame(640, 480, cv::DataType<float>::type);
            frame = image_msg.frame;
            long long unsigned frame_ts = image_msg.ts;
            if (fps_cnt == FPS_MAX_CNT) {
                t1 = get_us();
                fps = (float)fps_cnt/(float)(t1 - t0) * 1.0e6f;
                t0 = get_us();
                fps_cnt = 0;
                std::cout << color_fmt_yellow << "ORBThread:: FPS = " << std::fixed << std::setprecision(2) << fps << "fps" << color_fmt_reset << std::endl;
            } else {
                fps_cnt++;
            }

            if (frame.empty()) {
                continue;
            }

            cameraStarted = true;


            cv::cuda::GpuMat gpu_frame1, gpu_frame2;
            cv::cuda::GpuMat gpu_gray_frame;

            gpu_frame2.upload(frame);

            //cv::cuda::flip(gpu_frame2, gpu_frame1, 0);
            //cv::cuda::flip(gpu_frame1, gpu_frame2, 1);

            //cv::cuda::cvtColor(gpu_frame1, gpu_frame2, cv::COLOR_BGR2GRAY);

            cv::cuda::GpuMat d_keypoints;
            cv::cuda::GpuMat d_descriptors, d_descriptors_32F;

            std::vector <cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            std::vector <cv::Point2f> dist_points;
            std::vector <cv::Point2f> undist_points;

            //orb_detector -> detectAndComputeAsync(gpu_frame2, cv::cuda::GpuMat(), d_keypoints, d_descriptors);
            orb_detector->detectAsync(gpu_frame2, d_keypoints, cv::cuda::GpuMat());
            if (d_keypoints.empty()) {
                CAMMessageStruct msg = {frame, descriptors, undist_points, frame_ts};
                if (queueCamera.push(msg) == false) {
                    std::cout << color_fmt_yellow << "ORBThread:: Error!::" << "Queue full!" << color_fmt_reset
                              << std::endl;
                    exit_flag = 1;
                    quitCamera = true;
                    break;
                }
                continue;
            }
            orb_detector->convert(d_keypoints, keypoints);

            //d_descriptors.download(descriptors);

            std::vector <cv::KeyPoint> keypoints_srt;
            std::sort(keypoints.begin(), keypoints.end(),
                      [](const cv::KeyPoint &a, const cv::KeyPoint &b) { return a.response > b.response; });

            for (auto &keypoint: keypoints) {
                bool too_close = false;
                bool swapped = false;
                for (auto &keypoint_srt: keypoints_srt)
                    if (threshold_close > cv::norm(keypoint.pt - keypoint_srt.pt)) {
                        if (keypoint.response > keypoint_srt.response) {
                            std::swap(keypoint, keypoint_srt);
                            swapped = true;
                        } else {
                            too_close = true;
                        }
                    }
                if (!too_close && !swapped)
                    keypoints_srt.push_back(keypoint);
            }

            cv::KeyPointsFilter::retainBest(keypoints_srt, max_keypoints);

            //d_keypoints.upload(keypoints_srt);

            orb_detector_cpu->compute(frame, keypoints_srt, descriptors);

            //d_descriptors.download(descriptors);

            cv::KeyPoint::convert(keypoints_srt, dist_points);
            cv::undistortPoints(dist_points, undist_points, camMtx, distCoefs);

            std::vector <cv::KeyPoint> keypoints_fin;
            cv::KeyPoint::convert(dist_points, keypoints_fin);
            cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

#ifdef CALIBR_CAMERA
            if (skip_cnt == 0) {
                sprintf(nam_img, "calibr/img_%d.bmp", img_cnt);
                img_cnt++;
                skip_cnt = 15;
                cv::imwrite(nam_img, frame);
                std::cout << color_fmt_red << "ORBThread:: Save frame:: #" << img_cnt << color_fmt_reset << std::endl;
            } else {
                skip_cnt--;
            }
#endif

            CAMMessageStruct msg = {frame, descriptors, undist_points, frame_ts};
            if (queueCamera.push(msg) == false) {
                std::cout << color_fmt_yellow << "ORBThread:: Error!::" << "Queue full!" << color_fmt_reset << std::endl;
                exit_flag = 1;
                quitCamera = true;
                break;
            }
        }
        catch (int e) {
            std::cout << "ORB thread exception: " << e << std::endl;
            abort();
        }
    }
    // send empty message to unlock waiting thread
    CAMMessageStruct msg;
    queueCamera.push(msg);

    std::cout << color_fmt_yellow << "ORBThread:: " << frames_cnt << "total" << color_fmt_reset << std::endl;

    std::cout << color_fmt_yellow << "ORBThread:: Finished!" << color_fmt_reset << std::endl;

    return 0;
}


