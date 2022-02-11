//
// Created by step305 on 21.07.2021.
//

#include <iostream>
#include "serialStream.h"
#include <stdlib.h>
#include "thread"
#include "CircularQueue.h"
#include <signal.h>
#include "ORBdetector.h"
#include "syncThread.h"
#include "SLAM_thread.h"
#include "fifo_thread.h"
#include "RealsenseD455.h"

volatile sig_atomic_t exit_flag = 0;

std::atomic<bool> quitSerial;
std::atomic<bool> quitCamera;
std::atomic<bool> quitSync;
std::atomic<bool> quitSLAM;
std::atomic<bool> quitFIFO;
std::atomic<bool> cameraStarted;
std::atomic<bool> quitD455;

circ_queue::CircularFifo <IMUMessageStruct,serial_queue_len> queueSerial(false);
circ_queue::CircularFifo <CAMMessageStruct,camera_queue_len> queueCamera(false);
circ_queue::CircularFifo <SyncPacket,slam_queue_len> queueSLAM(true);
circ_queue::CircularFifo <SLAMLogMessageStruct,slam_queue_len> queueLogSLAM(false);
circ_queue::CircularFifo <SLAMLogMessageStruct,fifo_queue_len> queueFIFOSLAM(true);
circ_queue::CircularFifo <ImageMessageStruct,images_queue_len> queueImages(true);
//circ_queue::CircularFifo <RealsenseIMUMessageStruct,imu_queue_len> queueIMU(false);

void exit_catch(int sig) {
    std::cout << "SLAM-Logger:: User stop requested!" << std::endl;
    exit_flag = 1;
}

int main() {
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_catch;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    cameraStarted = false;

    std::thread slam_thread( SLAMThread );
    std::thread fifo_thread( fifoThread );
    std::thread sync_thread( syncThread );
    std::thread serial_thread( serialStreamThread );
    std::thread orb_thread( ORBdetectorStreamThread );
    std::thread realsense_thread( realsenseStreamThread );
    std::cout << "SLAM-Logger:: start!" << std::endl;

    bool quit = false;
    std::cout << "SLAM-Logger:: Press Ctrl+C to exit" << std::endl;

    while (!quit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (exit_flag) {
            quitSerial = true;
            quitCamera = true;
            quitD455 = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            quitSync = true;
            quitSLAM = true;
            quitFIFO = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            quit = true;
        }
    }

    if (serial_thread.joinable())
        serial_thread.join();
    if (realsense_thread.joinable())
        realsense_thread.join();
    if (orb_thread.joinable())
        orb_thread.join();
    if (sync_thread.joinable())
        sync_thread.join();
    if (slam_thread.joinable())
        slam_thread.join();
    if (fifo_thread.joinable())
        fifo_thread.join();
    std::cout << std::endl << "SLAM-Logger:: Done!" << std::endl;
    return 0;
}
