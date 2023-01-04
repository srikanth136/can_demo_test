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
#include <linux/can.h>
#include <linux/can/raw.h>
#include "can_wrap.h"
#include "candata.h"
#include "can_wrap.hpp"


void getTorqueFrame(const can_frame frame,int16_t &frontTorqueRequest,int16_t &rearTorqueRequest)
{
    candata_ai_drive_request_t data;
    candata_ai_drive_request_unpack(&data, frame.data, frame.can_dlc);
    if(!candata_ai_drive_request_rear_trq_request_is_in_range(data.front_trq_request))
    {
        printf("%s","front-torque is out of range");
        return;
    }
    if(!candata_ai_drive_request_rear_trq_request_is_in_range(data.rear_trq_request))
    {
         printf("%s","rear-torque is out of range");
         return;
    }
    frontTorqueRequest = candata_ai_drive_request_front_trq_request_decode(data.front_trq_request);
    rearTorqueRequest = candata_ai_drive_request_front_trq_request_decode(data.rear_trq_request);

}

void getSpeedFrame(const can_frame frame, uint16_t &frontLeftWheelRPM, uint16_t &frontRightWheelRPM, uint16_t &rearRightWheelRPM, uint16_t &rearLeftWheelRPM)
{
    candata_vcu_wheel_speeds_t data;
    candata_vcu_wheel_speeds_unpack(&data, frame.data , frame.can_dlc);
    if(!candata_vcu_wheel_speeds_fl_wheel_speed_is_in_range(data.fl_wheel_speed))
        printf("%s","front_left_wheel_speed");
        return;
    if(!candata_vcu_wheel_speeds_fr_wheel_speed_is_in_range(data.fr_wheel_speed))
        printf("%s","front-right_wheel_speed");
        return;
    if(!candata_vcu_wheel_speeds_rl_wheel_speed_is_in_range(data.rl_wheel_speed))
        printf("%s","rear-right_wheel_speed");
        return;
    if(!candata_vcu_wheel_speeds_rr_wheel_speed_is_in_range(data.rr_wheel_speed))
        printf("%s","rear-left_wheel_speed");
        return;
    frontLeftWheelRPM= candata_vcu_wheel_speeds_fl_wheel_speed_decode(data.fl_wheel_speed);
    frontRightWheelRPM = candata_vcu_wheel_speeds_fr_wheel_speed_decode(data.fr_wheel_speed);
    rearLeftWheelRPM = candata_vcu_wheel_speeds_rl_wheel_speed_decode(data.rl_wheel_speed);
    rearRightWheelRPM = candata_vcu_wheel_speeds_rr_wheel_speed_decode(data.rr_wheel_speed);
}

void getVoltageFrame(const can_frame frame,double &battery_voltage)
{
    candata_vcu_battery_t data;
    candata_vcu_battery_unpack(&data,frame.data,frame.can_dlc);
    if(!candata_vcu_battery_voltage_is_in_range(data.voltage))
        printf("battery_voltage");
        return;
    battery_voltage = candata_vcu_battery_voltage_decode(data.voltage);

}

void getCurrentBasedOnRPM(uint16_t &left_wheel_rpm, uint16_t &right_wheel_rpm, int16_t &torque_request, double &battery_voltage, const double motor_q, const double &back_electromagnetic_field_factor, uint16_t &current)
{
        uint16_t max_wheel_rpm = (left_wheel_rpm > right_wheel_rpm) ? left_wheel_rpm: right_wheel_rpm;

        uint16_t min_wheel_rpm = (left_wheel_rpm < right_wheel_rpm) ? left_wheel_rpm: right_wheel_rpm;

        torque_request = (max_wheel_rpm > 700 && min_wheel_rpm < 700) ? 50: torque_request;
        torque_request = (max_wheel_rpm > 600 && min_wheel_rpm < 680) ? 85 : torque_request;
        torque_request = (max_wheel_rpm > 500 && min_wheel_rpm < 500) ? 100: torque_request;
        torque_request = (max_wheel_rpm > 400 && min_wheel_rpm < 400) ? 120: torque_request;
        torque_request = (max_wheel_rpm > 300 && min_wheel_rpm < 300) ? 150 : torque_request;
        torque_request = (battery_voltage <= 2.8) ? 20 : torque_request;

        current = (torque_request *((back_electromagnetic_field_factor * max_wheel_rpm * torque_request) + (pow (motor_q,2) * torque_request) + 100)) / (100 * battery_voltage);

}


void calculateCurrent (uint16_t &frontLeftWheelRPM, uint16_t &frontRightWheelRPM, uint16_t &rearLeftWheelRPM, uint16_t &rearRightWheelRPM, int16_t
&frontTorqueRequest, int16_t &rearTorqueRequest, double &battery_voltage, const double &motor_q, const double
&back_electromagnetic_field_factor, uint16_t &frontCurrent, uint16_t &rearCurrent)
{
    //get front current
    getCurrentBasedOnRPM(frontLeftWheelRPM,frontRightWheelRPM,frontTorqueRequest,battery_voltage,motor_q, back_electromagnetic_field_factor, frontCurrent);
    //get rear current
    getCurrentBasedOnRPM(rearLeftWheelRPM,rearRightWheelRPM,rearTorqueRequest,battery_voltage,motor_q, back_electromagnetic_field_factor,rearCurrent);
}


void computeFrame(const can_frame frame, uint16_t &frontLeftWheelRPM,uint16_t &frontRightWheelRPM, uint16_t &rearLeftWheelRPM,uint16_t &rearRightWheelRPM, int16_t &frontTorqueRequest, int16_t &rearTorqueRequest, double &battery_voltage,uint16_t &frameRecieveCounter)
{
    switch(frame.can_id)
    {
        case CANDATA_AI_DRIVE_REQUEST_FRAME_ID:
        {
            getTorqueFrame(frame,frontTorqueRequest,rearTorqueRequest);
            frameRecieveCounter++;
        }
        break;
    
        case CANDATA_VCU_WHEEL_SPEEDS_FRAME_ID:
        {
            getSpeedFrame(frame, frontLeftWheelRPM,frontRightWheelRPM,rearLeftWheelRPM,rearRightWheelRPM);
            frameRecieveCounter++;
            
        }  
        break;
        case CANDATA_VCU_BATTERY_FRAME_ID:
        {
            getVoltageFrame(frame,battery_voltage);
            frameRecieveCounter;
        }
        break;
       
    default:
        printf("%s","unknown frames..!")
        break; //for frames unknown
    }
}



int main()
{
    const double motor_q = 0.879;
    const double back_electromagnetic_field_factor =0.6721;
    uint16_t frontLeftWheelRPM=0;
    uint16_t frontRightWheelRPM = 0;
    uint16_t rearLeftWheelRPM = 0;
    uint16_t rearRightWheelRPM = 0;
    double battery_voltage=0;
    int16_t frontTorqueRequest=0;
    int16_t rearTorqueRequest=0;
    uint16_t frontCurrent = 0;
    uint16_t rearCurrent = 0;
    const char* canChannel = "vcan0";
    const int can_socket = can_connect( canChannel, false );
    uint16_t frameSentCounter = 0;
    uint16_t frameRecieveCounter = 0;

    
        while(1)
        {
            const can_frame frame = can::read(can_socket);
            printf("%d\n RECV_COUNT :",frameRecieveCounter);
            printf("%d\n SEND COUNT :", frameSentCounter);
            computeFrame(frame,frontLeftWheelRPM,frontRightWheelRPM,rearLeftWheelRPM,rearRightWheelRPM,frontTorqueRequest,rearTorqueRequest,battery_voltage,frameRecieveCounter);
            if((frameRecieveCounter > 0) && (frameRecieveCounter == frameSentCounter * 3)); break;
            calculateCurrent(frontLeftWheelRPM, frontRightWheelRPM, rearLeftWheelRPM, rearRightWheelRPM, frontTorqueRequest, rearTorqueRequest, battery_voltage,
            motor_q, back_electromagnetic_field_factor ,frontCurrent, rearCurrent);
            calculateCurrent (frontLeftWheelRPM, frontRightWheelRPM, rearLeftWheelRPM, rearRightWheelRPM, frontTorqueRequest, rearTorqueRequest, battery_voltage,
            motor_q, back_electromagnetic_field_factor ,frontCurrent, rearCurrent);
            struct can_frame current_frame;
            current_frame.can_id = CANDATA_MOTOR_CURRENT_FRAME_ID;
            current_frame.can_dlc = CANDATA_MOTOR_CURRENT_LENGTH;
        
            current_frame.data[0] = (frontCurrent >> 8) & 0xff;
            current_frame.data[1] = frontCurrent & 0xff;
            current_frame.data[2] = (rearCurrent >> 8) & 0XFf;
            current_frame.data[3] = rearCurrent & 0xff;
            if( (frameRecieveCounter % 3 == 0) && frameRecieveCounter > 0)
            {
                std::cout<<current_frame<<std::endl;
                if(!can_write(can_socket, current_frame))
                {
                   printf("ERROR \n") 
                }
                else{
                    frameSentCounter++;
                }
                
            }
        }
   

    sleep( 1 );
    can_close(can_socket);

return 0;
}