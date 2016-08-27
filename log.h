/*
 * Usage:
 * static peripherals::Log testlog;
 *
 * main(){
 *  testlog.init();
 *  ...
 *  testlog.stop()
 *  }
 *
 * tg():{
 *
 *  LogData temperary_data;
 *  err=rt_dev_sendto(
 *      testlog.file_id_real_time,
 *      &temperary_data,
 *      sizeof(temperary_data),
 *      0,NULL,0);
 * }
 *
 */

#ifndef LOG_H
#define LOG_H

#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <malloc.h>
//#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <rtdm/rtdm.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>
#include <rtdm/rtipc.h>
#include <iostream>
#include <thread>
#include <aris.h>
#include "log_data.h"
// define some log data here





#define LOG_XDDP_PORT_DEFAULT 5

#define NSEC_PER_SEC (1000000000L)
#define FREQUENCY_RT 100
#define PERIOD_NS_RT (NSEC_PER_SEC * 1.0 / FREQUENCY_RT)




namespace peripherals
{




class Log
{
public:
    Log();
    ~Log();
    int init();
    void stop();
    bool is_log_file_closed();
    int start_data_server();


    //send in rt
    static int file_id_real_time;
    //read in nrt
    static int file_id_non_real_time;


private:
    int init_rt();
    int init_nrt();



private:
    static void data_process_nrt(int n);
    static int create_log_file(FILE **file_id);
    static void close_log_file(FILE **file_id);


    static bool is_data_server_running;
    static bool is_logging;
    static bool is_log_file_finished;
    static bool is_read_blocking;

    static std::FILE* log_file;

    std::thread data_server;






#define BUF_SIZE 32768
    static char file_id_rt_buffer[BUF_SIZE];

    static char file_id_nrt_buffer[BUF_SIZE];
    static char* file_id_nrt_device_name;

};

}


#endif
