#include "RealsenseD455.h"
#include "RealsenseUtils.h"

int realsenseStreamThread()
try {
    std::string serial;
    if (!device_with_streams({ RS2_STREAM_GYRO  }, serial)) {
        std::cout << color_fmt_blue << "realsenseThread:: Not found supported Realsense camera!" << color_fmt_reset << std::endl;
        exit_flag = 1;
        return EXIT_SUCCESS;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::cout << color_fmt_blue << "realsenseThread:: Started!" << color_fmt_reset << std::endl;

    rs2::pipeline pipe_temp;
    rs2::pipeline_profile selection = pipe_temp.start();
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }
    pipe_temp.stop();
    rs2::pipeline pipe;

    std::cout << color_fmt_blue << "realsenseThread:: Realsense camera configured. Laser set to off." << color_fmt_reset << std::endl;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
    // Add gyro stream
    // cfg.enable_stream(RS2_STREAM_GYRO , RS2_FORMAT_MOTION_XYZ32F, IMU_FPS);
    // Enable both image streams
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_INFRARED , 1, 640, 480, RS2_FORMAT_Y8, IMAGES_FPS);
    cfg.enable_stream(RS2_STREAM_INFRARED , 2, 640, 480, RS2_FORMAT_Y8, IMAGES_FPS);

    // Define frame callback

    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    std::mutex data_mutex;
    uint64_t pose_counter = 0;
    uint64_t frame_counter = 0;
    bool first_data = true;
    auto last_print = std::chrono::system_clock::now();
    bool skip_imu_packet = false;
    uint64_t fps_image = 0, fps_imu = 0;
    rs2_vector dat_prev = {0.0f, 0.0f, 0.0f};
    int skip_frames = 0;

    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        rs2_vector dat;
        RealsenseIMUMessageStruct imu_frame;
        ImageMessageStruct image_frame;
        IMUMessageStruct crh_msg;

        // Only start measuring time elapsed once we have received the
        // first piece of data
        if (first_data) {
            first_data = false;
            last_print = std::chrono::system_clock::now();
        }

        if (auto frame_motion = frame.as<rs2::motion_frame>()) {
            pose_counter++;
            dat = frame_motion.get_motion_data();
            imu_frame.rate[2] = (dat.x + dat_prev.x)*0.5f*0.005f;
            imu_frame.rate[0] = -(dat.y + dat_prev.y)*0.5f*0.005f;
            imu_frame.rate[1] = -(dat.z + dat_prev.z)*0.5f*0.005f;
            dat_prev = dat;
            imu_frame.ts = get_us();

            if (!skip_imu_packet) {
                skip_imu_packet = true;
                if (queueSerial.pop(crh_msg)) {
                    imu_frame.adc[0] = crh_msg.adc[0];
                    imu_frame.adc[1] = crh_msg.adc[1];
                    imu_frame.adc[2] = crh_msg.adc[2];
                } else {
                    imu_frame.adc[0] = -999999.0f;
                    imu_frame.adc[1] = -999999.0f;
                    imu_frame.adc[2] = -999999.0f;
                }

                /*if (queueIMU.push(imu_frame) == false) {
                    std::cout << color_fmt_blue << "realsenseThread:: Error!:: IMU queue full!!" << color_fmt_reset
                              << std::endl;
                    exit_flag = 1;
                    quitD455 = true;
                }*/
            } else {
                skip_imu_packet = false;
            }
        } else if (auto frame_image = frame.as<rs2::frameset>()) {
            rs2::video_frame img = frame_image.get_infrared_frame(1);
            const int w = img.get_width();
            const int h = img.get_height();
            cv::Mat image(cv::Size(w, h), CV_8UC1, (void*)img.get_data());
            image_frame.frame = image;
            image_frame.ts = get_us();

            if (skip_frames < FRAMES_TO_SKIP) {
                skip_frames++;
            } else {
                skip_frames = 0;
                if (queueImages.push(image_frame) == false) {
                    std::cout << color_fmt_blue << "realsenseThread:: Error!:: Image queue full!!" << color_fmt_reset << std::endl;
                    exit_flag = 1;
                    quitD455 = true;
                }
            }
            frame_counter++;
        }

        auto now = std::chrono::system_clock::now();
        if (now - last_print >= std::chrono::seconds(1)) {
            fps_image = frame_counter;
            fps_imu = pose_counter;
            std::cout << color_fmt_blue << "realsenseThread:: FPS image: "
                        << std::fixed << std::setprecision(1)
                        << fps_image << "fps"
                        << "; FPS IMU: "
                        << fps_imu << "fps"
                        << color_fmt_reset << std::endl;
            pose_counter = 0;
            frame_counter = 0;
            last_print = now;
        }
    };

    // Start streaming through the callback
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);

    std::cout << color_fmt_blue << "realsenseThread:: Camera configured!" << color_fmt_reset << std::endl;

    while (!quitD455) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // send empty frame to unlock waiting thread
    ImageMessageStruct image_frame;
    queueImages.push(image_frame);
    std::cout << color_fmt_blue << "realsenseThread:: Finished!" << color_fmt_reset << std::endl;
    return 0;
}

catch (const rs2::error & e)
{
    std::cerr << "realsenseThread:: RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << "realsenseThread::" << e.what() << std::endl;
    return EXIT_FAILURE;
}
