#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>
#include<math.h>
#include<chrono>
#include<iostream>
#include<string>
#include<thread>
#include<vector>
#include<bits/stdc++.h>
#include "can_wrap.hpp"
#include "candata.h"
using can::operator<<;
using namespace std;

void torque_frame(const can_frame frame,int16_t &f_torque_request,int16_t &r_torque_request)
{
candata_ai_drive_request_t data;
candata_ai_drive_request_unpack(&data, frame.data, frame.can_dlc);
if(!candata_ai_drive_request_rear_trq_request_is_in_range(data.front_trq_request))
throw std::out_of_range("front-troque");
if(!candata_ai_drive_request_rear_trq_request_is_in_range(data.rear_trq_request))
 throw std::out_of_range("rear-troque");
 f_torque_request = candata_ai_drive_request_front_trq_request_decode(data.front_trq_request);
 f_torque_request = candata_ai_drive_request_front_trq_request_decode(data.rear_trq_request);

}
void speed_frame(const can_frame frame, uint16_t &fl_wheel_rpm, uint16_t &fr_wheel_rpm, uint16_t &rr_wheel_rpm, uint16_t &rl_wheel_rpm)
{
    candata_vcu_wheel_speeds_t data;
candata_vcu_wheel_speeds_unpack(&data, frame.data , frame.can_dlc);
    if(!candata_vcu_wheel_speeds_fl_wheel_speed_is_in_range(data.fl_wheel_speed))
    throw std::out_of_range("front_left_wheel_speed");
    if(!candata_vcu_wheel_speeds_fr_wheel_speed_is_in_range(data.fr_wheel_speed))
    throw std::out_of_range("front-right_wheel_speed");
    if(!candata_vcu_wheel_speeds_rl_wheel_speed_is_in_range(data.rl_wheel_speed))
    throw std::out_of_range("rear-right_wheel_speed");
    if(!candata_vcu_wheel_speeds_rr_wheel_speed_is_in_range(data.rr_wheel_speed))
    throw std::out_of_range("rear-left_wheel_speed");
    fl_wheel_rpm = candata_vcu_wheel_speeds_fl_wheel_speed_decode(data.fl_wheel_speed);
    fr_wheel_rpm = candata_vcu_wheel_speeds_fr_wheel_speed_decode(data.fr_wheel_speed);
    rl_wheel_rpm = candata_vcu_wheel_speeds_rl_wheel_speed_decode(data.rl_wheel_speed);
    rr_wheel_rpm = candata_vcu_wheel_speeds_rr_wheel_speed_decode(data.rr_wheel_speed);
}
void voltage_frame(const can_frame frame,double &battery_voltage)
{
    candata_vcu_battery_t data;
    candata_vcu_battery_unpack(&data,frame.data,frame.can_dlc);
    if(!candata_vcu_battery_voltage_is_in_range(data.voltage))
    throw std::out_of_range("battery_voltage");
    battery_voltage = candata_vcu_battery_voltage_decode(data.voltage);

}
void process_frame(const can_frame frame, uint16_t &fl_wheel_rpm,uint16_t &fr_wheel_rpm, uint16_t &rl_wheel_rpm,uint16_t &rr_wheel_rpm, int16_t &f_torque_request, int16_t &r_torque_request, double &battery_voltage,uint16_t &frame_recv_counter)
{
    switch(frame.can_id)
    {
        case CANDATA_AI_DRIVE_REQUEST_FRAME_ID:
        {
            torque_frame(frame,f_torque_request,r_torque_request);
            frame_recv_counter++;
        }
        break;
    
        case CANDATA_VCU_WHEEL_SPEEDS_FRAME_ID:
        {
            speed_frame(frame, fl_wheel_rpm,fr_wheel_rpm,rl_wheel_rpm,rr_wheel_rpm);
            frame_recv_counter++;
            
        }  
        break;
        case CANDATA_VCU_BATTERY_FRAME_ID:
        {
            voltage_frame(frame,battery_voltage);
            frame_recv_counter;
        }
        break;
       
    default:
        break; //frame unknown
    }
}
void formulate_current(uint16_t &left_wheel_rpm, uint16_t &right_wheel_rpm, int16_t &torque_request, double &battery_voltage, const double motor_q, const double &back_electromagnetic_field_factor, uint16_t &current)
{
uint16_t max_wheel_rpm = (left_wheel_rpm > right_wheel_rpm) ? left_wheel_rpm: right_wheel_rpm;

uint16_t min_wheel_rpm = (left_wheel_rpm < right_wheel_rpm) ? left_wheel_rpm: right_wheel_rpm;

torque_request = (max_wheel_rpm > 700 && min_wheel_rpm < 700) ? 50: torque_request;
torque_request = (max_wheel_rpm > 600 && min_wheel_rpm < 680) ? 85 : torque_request;
torque_request = (max_wheel_rpm > 500 && min_wheel_rpm < 500) ? 100: torque_request;
torque_request = (max_wheel_rpm > 400 && min_wheel_rpm < 400) ? 120: torque_request;
torque_request = (max_wheel_rpm > 300 && min_wheel_rpm < 300) ? 150 : torque_request;
torque_request = (battery_voltage <= 2.8) ? 20 : torque_request;
current = (torque_request *((back_electromagnetic_field_factor
* max_wheel_rpm * torque_request) + (pow (motor_q,2) * torque_request) +
100)) / (100 * battery_voltage);

}

void compute_current (uint16_t &fl_wheel_rpm, uint16_t &fr_wheel_rpm, uint16_t &rl_wheel_rpm, uint16_t &rr_wheel_rpm, int16_t
&f_torque_request, int16_t &r_torque_request, double &battery_voltage, const double &motor_q, const double
&back_electromagnetic_field_factor, uint16_t &f_current, uint16_t &r_current)
{
//front current
formulate_current(fl_wheel_rpm,fr_wheel_rpm,f_torque_request,battery_voltage,motor_q, back_electromagnetic_field_factor, f_current);
//rear current
formulate_current(rl_wheel_rpm,rr_wheel_rpm,r_torque_request,battery_voltage,motor_q, back_electromagnetic_field_factor,r_current);
}

int main()
{
    const double motor_q = 0.879;
    const double back_electromagnetic_field_factor =0.6721;
    uint16_t fl_wheel_rpm =0;
    uint16_t fr_wheel_rpm = 0;
    uint16_t rl_wheel_rpm = 0;
    uint16_t rr_wheel_rpm = 0;
    double battery_voltage=0;
    int16_t f_torque_request=0;
    int16_t r_torque_request=0;
    uint16_t f_current = 0;
    uint16_t r_current = 0;
    std::string can_channel = "vcan0";
    const int can_socket = can::connect(can_channel);
    uint16_t frame_sent_counter = 0;
    uint16_t frame_recv_counter = 0;

    try{
        while(1)
        {
            const can_frame frame = can::read(can_socket);
            std::cout<<"printing fram"<<frame<<endl;
            std::cout<<endl<<"RECV_COUNT : "<<frame_recv_counter<<endl<<"SEND COUNT :"<< frame_sent_counter<<endl;
            process_frame(frame,fl_wheel_rpm,fr_wheel_rpm,rl_wheel_rpm,rr_wheel_rpm,f_torque_request,r_torque_request,battery_voltage,frame_recv_counter);
            if((frame_recv_counter > 0) && (frame_recv_counter == frame_sent_counter * 3)); break;
            compute_current(fl_wheel_rpm, fr_wheel_rpm, rl_wheel_rpm, rr_wheel_rpm, f_torque_request, r_torque_request, battery_voltage,
            motor_q, back_electromagnetic_field_factor ,f_current, r_current);
compute_current (fl_wheel_rpm, fr_wheel_rpm, rl_wheel_rpm, rr_wheel_rpm, f_torque_request, r_torque_request, battery_voltage,
motor_q, back_electromagnetic_field_factor ,f_current, r_current);
struct can_frame current_frame;
current_frame.can_id = CANDATA_MOTOR_CURRENT_FRAME_ID;
current_frame.can_dlc = CANDATA_MOTOR_CURRENT_LENGTH;
try
{
current_frame.data[0] = (f_current >> 8) & 0xff;
current_frame.data[1] = f_current & 0xff;
current_frame.data[2] = (r_current >> 8) & 0XFf;
current_frame.data[3] = r_current & 0xff;
if( (frame_recv_counter % 3 == 0) && frame_recv_counter > 0)
{
    std::cout<<current_frame<<std::endl;
can::write(can_socket, current_frame);
frame_sent_counter++;
}

}

catch(const std::runtime_error& e)
{
    // std::cout<<e<<std::endl;
    std::cerr<<e.what()<<std::endl;
}
        }
    }


catch(const std::exception& e)
{
    std::cerr<<e.what()<<std::endl;
}

std::this_thread::sleep_for(std::chrono::seconds(1));
can::close(can_socket);

return 0;
}