/*
 * This is the file corresponding to the log data structure defined
 * in the header file log_data_20160420.h
*/
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <fstream>
#include "log_data_20160819.h"

#include <fstream>

std::ifstream::pos_type filesize(const char* filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    return in.tellg();
}

int main(int argc,char *argv[])
{
    int file_size=filesize(argv[1]);
    std::cout<<"file size:"<<file_size<<std::endl;
    std::cout<<"datum size:"<<sizeof(LogData)<<std::endl;
    std::cout<<((double)file_size)/((double)sizeof(LogData))
            <<" cast: "<<file_size/sizeof(LogData)<<std::endl;


    int log_file;
    log_file = open(argv[1],O_RDONLY);


    if(log_file<0)
    {
        printf("Failed to open the log file.");
        return 0;
    }
    std::string txt_file_name(argv[1]);
    const char* txt_file="txt";
    txt_file_name=txt_file_name.replace(txt_file_name.end()-6 ,txt_file_name.end(),txt_file,3);
    std::cout<<"Output to file: "<<txt_file_name<<std::endl;
    std::ofstream output_file;
    output_file.open(txt_file_name);


    LogData temp_log_data;
    int ret=1;
    int sum;
    int count=0;
    double progress=0.0;
    double current_progress=0.0;
    double refresh_threshold=0.01;
    std::cout<<"progress: "<<current_progress<<std::endl;
    while(ret>0)
    {
        ret=read(log_file,&temp_log_data,sizeof(temp_log_data));
        if(ret<sizeof(LogData))
        {

            std::cout<<"\nAbnormal data element size"<<std::endl;
            std::cout<<ret<<" count: "<<count<<std::endl;
            std::cout<<"Abnormal data element size"<<std::endl;
            break;
        }
        if(ret==0)
        {
            break;
        }
        sum+=ret;
        progress=(double)sum/(double)file_size;
        if(progress-current_progress>refresh_threshold)
        {
            current_progress=progress;
            std::cout << "\r"<<current_progress*100 <<"%"<< std::flush;
        }

        //output to txt file

        //column 0-1 matlab:1-2
        output_file<<count<<"\t"<<temp_log_data.count;

        //column 3-20 matlab:4-21
        for(int i=0;i<18;i++)
        {
            output_file<<"\t"<<temp_log_data.raw_data_position[i];
        }
        //column 21-38 matlab:22-39
        for(int i=0;i<18;i++)
        {
            output_file<<"\t"<<temp_log_data.raw_data_torque[i];
        }
        //column 39-56 matlab:40-57
        for(int i=0;i<18;i++)
        {
            output_file<<"\t"<<temp_log_data.prism_acceleration[i];
        }
        //column 57-74 matlab:58-75
        for(int i=0;i<18;i++)
        {
            output_file<<"\t"<<temp_log_data.prism_velocity[i];
        }
        //column 75-92 matlab:76-93
        for(int i=0;i<18;i++)
        {
            output_file<<"\t"<<temp_log_data.prism_position[i];
        }
        //column 93-110 matlab: 94-111
        for(int i=0;i<18;i++)
        {
            output_file<<"\t"<<temp_log_data.prism_actual_force[i];
        }
        //column 111-128 matlab: 112-129
        for(int i=0;i<18;i++)
        {
            output_file<<"\t"<<temp_log_data.prism_dynamic_force[i];
        }
        //column 129-146 matlab: 130-147
        for(int i=0;i<6;i++)
        {
            output_file<<"\t"<<temp_log_data.foot_tip_extern_force[i][0];
            output_file<<"\t"<<temp_log_data.foot_tip_extern_force[i][1];
            output_file<<"\t"<<temp_log_data.foot_tip_extern_force[i][2];
        }
        //column 147-155 matlab: 148-156
        output_file<<"\t"<<temp_log_data.imu_raw_angle[0];
        output_file<<"\t"<<temp_log_data.imu_raw_angle[1];
        output_file<<"\t"<<temp_log_data.imu_raw_angle[2];
        output_file<<"\t"<<temp_log_data.imu_raw_vel[0];
        output_file<<"\t"<<temp_log_data.imu_raw_vel[1];
        output_file<<"\t"<<temp_log_data.imu_raw_vel[2];
        output_file<<"\t"<<temp_log_data.imu_raw_acc[0];
        output_file<<"\t"<<temp_log_data.imu_raw_acc[1];
        output_file<<"\t"<<temp_log_data.imu_raw_acc[2];

        //column 156-173 matlab: 157-174
        for(int i=0;i<6;i++)
        {
            output_file<<"\t"<<temp_log_data.foot_target_coordinate_frame_body[i][0];
            output_file<<"\t"<<temp_log_data.foot_target_coordinate_frame_body[i][1];
            output_file<<"\t"<<temp_log_data.foot_target_coordinate_frame_body[i][2];
        }

        //column 174-179 matlab: 175-180
        for(int i=0;i<6;i++)
        {
            output_file<<"\t"<<temp_log_data.body_center_coordinate_frame_ground[i];
        }

        for(int i=0;i<6;i++)
        {
            for(int j=0;j<6;j++)
            {
               output_file<<"\t"<<temp_log_data.force_sensor[i][j];
            }

        }


        output_file<<"\n";
        count++;
//        break;
    }
    output_file.close();
    std::cout<<"Process finish."<<std::endl;


    return 0;
}

