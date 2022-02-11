//
// Created by driver on 18.10.2021.
//

#ifndef SLAM_LOGGER_FIFO_THREAD_H
#define SLAM_LOGGER_FIFO_THREAD_H
#include <iomanip>
#include <stdlib.h>
#include <memory>
#include <string>
#include <stdexcept>
#include "CircularQueue.h"
#include <fcntl.h>
#include <unistd.h>
#include "thread"
#include "chrono"
#include <iostream>
#include "chrono"
#include <sys/stat.h>
#include <stdio.h>
#include <signal.h>
#include "syncThread.h"
#include "utils.h"
#include "base64.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#define BUF_SIZE 100000
#define SERVER_PORT htons(7000)
#define bzero(b,len) (memset((b), '\0', (len)), (void) 0)
#define bcopy(b1,b2,len) (memmove((b2), (b1), (len)), (void) 0)
#define fifo_queue_len      300

extern std::atomic<bool> quitFIFO;
extern volatile sig_atomic_t exit_flag;
extern circ_queue::CircularFifo <SLAMLogMessageStruct,fifo_queue_len> queueFIFOSLAM;

void fifoThread();

#endif //SLAM_LOGGER_FIFO_THREAD_H
