#include"log.h"
#include"time.h"

//When use Log, remember to turn off Aris log, becasue it will generate a lot writing operarion.

bool peripherals::Log::is_data_server_running=true;
bool peripherals::Log::is_logging=true;
bool peripherals::Log::is_log_file_finished=false;
bool peripherals::Log::is_read_blocking=false;

int peripherals::Log::file_id_non_real_time=0;
int peripherals::Log::file_id_real_time=0;

char peripherals::Log::file_id_nrt_buffer[BUF_SIZE];
char peripherals::Log::file_id_rt_buffer[BUF_SIZE];
char* peripherals::Log::file_id_nrt_device_name;

std::FILE* peripherals::Log::log_file=NULL;


peripherals::Log::Log()
{

}


peripherals::Log::~Log()
{
    is_data_server_running=false;
}


int peripherals::Log::init()
{
    std::cout<<"Custom Log init"<<std::endl;
    this->init_rt();
    this->init_nrt();
    std::cout<<"Custom Log init finished"<<std::endl;
    this->start_data_server();
    std::cout<<"Custom Log start dataserver"<<std::endl;
    return 0;
}

int peripherals::Log::init_rt()
{
    struct sockaddr_ipc saddr;
    int ret;

    size_t poolsz;

    file_id_real_time=rt_dev_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);

    if(file_id_real_time<0)
    {
        std::cout<<"RT data communication failed!\n"<<std::endl;
    }
    else
    {
        printf("1\n");
    }

    struct timeval tv;

    tv.tv_sec = 0;  /* 30 Secs Timeout */
    tv.tv_usec = 0;  // Not init'ing this can cause strange errors

    ret=rt_dev_setsockopt(file_id_real_time, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
    if (ret)
    {
        std::cout<<"RT data communication failed!\n"<<std::endl;
    }
    else
    {
        printf("2\n");
    }

    /*
     * Set a local 16k pool for the RT endpoint. Memory needed to
     * convey datagrams will be pulled from this pool, instead of
     * Xenomai's system pool.
     */
    poolsz = 16384; /* bytes */
    ret = rt_dev_setsockopt(file_id_real_time, SOL_XDDP, XDDP_POOLSZ,
             &poolsz, sizeof(poolsz));

    if (ret)
    {
        std::cout<<"RT data communication failed!\n"<<std::endl;
    }
    else
    {
        printf("3\n");
    }

    /*
     * Bind the socket to the port, to setup a proxy to channel
     * traffic to/from the Linux domain.
     *
     * saddr.sipc_port specifies the port number to use.
     */
    memset(&saddr, 0, sizeof(saddr));
    saddr.sipc_family = AF_RTIPC;
    saddr.sipc_port = LOG_XDDP_PORT_DEFAULT;
    ret = rt_dev_bind(file_id_real_time, (struct sockaddr *)&saddr, sizeof(saddr));
    if (ret)
    {
        std::cout<<"RT data communication failed!\n"<<std::endl;
    }
    else
    {
        printf("4\n");
    }

    return 0;
}

int peripherals::Log::init_nrt()
{
    if (asprintf(&file_id_nrt_device_name, "/dev/rtp%d", LOG_XDDP_PORT_DEFAULT) < 0)
        printf("Error in asprintf\n");
    file_id_non_real_time=open(file_id_nrt_device_name,O_RDWR);
    free(file_id_nrt_device_name);

    if(file_id_non_real_time<0)
        std::cout<<"Error in of FD_NRT"<<std::endl;
    else
        std::cout<<"open nrt "<<file_id_non_real_time<<std::endl;

    //set to nonblock
//    int flags = fcntl(file_id_non_real_time, F_GETFL, 0);
//    fcntl(file_id_non_real_time, F_SETFL, flags | O_NONBLOCK);

    //set to block
    int flags = fcntl(file_id_non_real_time, F_GETFL, 0);
    fcntl(file_id_non_real_time, F_SETFL, flags &~ O_NONBLOCK);

    return 0;
}

int peripherals::Log::start_data_server()
{
    is_data_server_running=true;
    create_log_file(&log_file);
    data_server=std::thread(data_process_nrt,0);

    return 0;
}

void peripherals::Log::data_process_nrt(int n)
{
    std::cout<<"data_server starts"<<std::endl;
    LogData temperory_data;

    int ret = 0;


    while(is_data_server_running)
    {

        //read from rt, this one is blocked

        is_read_blocking=true;
        ret=read(file_id_non_real_time,&temperory_data,sizeof(temperory_data));
        is_read_blocking=false;

        //write to binary file
        if(ret>0 && is_logging)
        {
            fwrite(&temperory_data,sizeof(temperory_data),1,log_file);
            //debug info
//            std::cout<<"Data logged."<<std::endl;
        }


    }

    //close log binary file
    close_log_file(&log_file);
    std::cout<<"data_server ends"<<std::endl;
    is_log_file_finished=true;
}

int peripherals::Log::create_log_file(FILE **file_id)
{
        char LogFile[300];
        char tmpDate[100];
        time_t now;
        struct tm *p;
        time(&now);
        p = localtime(&now);

        strftime(tmpDate,99,"%Y_%m_%d_%H_%M_%a",p);
        sprintf(LogFile,"Log_%s.botlog",tmpDate);

        if((*file_id=fopen(LogFile,"wb"))!=NULL)
        {
            printf("%s log file opened.\n",tmpDate);
        }

        return 0;

}

void peripherals::Log::close_log_file(FILE **file_id)
{
    fflush(*file_id);
    fclose(*file_id);
    is_log_file_finished=true;
    std::cout<<"Log file closed"<<std::endl;
}

void peripherals::Log::stop()
{
    is_data_server_running=false;
    printf("data server stop called.\n");
    if(is_read_blocking)
    {
        std::cout<<"Read operation is blocking, join data server thread"<<std::endl;
        data_server.join();
    }
    if(!is_log_file_closed())
    {
        std::cout<<"writing log file to the disk."<<std::endl;
    }
    while(!is_log_file_closed())
    {
        sleep(0.5);
    }
}

bool peripherals::Log::is_log_file_closed()
{
    return is_log_file_finished;
}

