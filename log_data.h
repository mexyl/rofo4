#ifndef LOG_DATA_H
#define LOG_DATA_H
/*
 * Version: 20160420
*/

/*
 * XYL:
 * Add model calculation reslut to CMachineData
 * The order of the data is arranged as described in the model
 *
 * Modify do not modify this class frequently, or the data process will not
*/
#define AXIS_NUMBER 18
class LogData
{
public:

    //model calculation already in abstract mapping

    int raw_data_position[AXIS_NUMBER];//*
    int raw_data_torque[AXIS_NUMBER];//*

    double prism_acceleration[AXIS_NUMBER];//*
    double prism_velocity[AXIS_NUMBER];//*
    double prism_position[AXIS_NUMBER];//*

    double prism_actual_force[AXIS_NUMBER];// calculated from the torque //*
    double prism_dynamic_force[AXIS_NUMBER];// calculated by fast dyn //*

    double foot_tip_extern_force[6][3];// six leg extern force //*

    //2016-04-19 not used now
    double imu_raw_angle[3];
    double imu_raw_vel[3];
    double imu_raw_acc[3];

    int count;//*

    //RunGaitAll record planning result
    double foot_target_coordinate_frame_body[6][3];
    double body_center_coordinate_frame_ground[6];

    //force sensor data
    double force_sensor[6][6];// row: leg id; col: fx, fy, fz, mx, my, mz

    //logic
    bool is_logging=false;
};

#endif // LOG_DATA_H
