#include "CAN.h"
#include "ThisThread.h"
#include "can_helper.h"
#include "coap_security_handler.h"
#include "mbed.h"
#include "mbed_wait_api.h"
#include <cstdint>
#include "can_comm.h"
#include "position_control.h"

// #include <iostream>
Ticker ticker;

// can interface
CAN can_one(D10, D2);

// current motor ID

uint8_t leg0[2] = {5,6}; // there are two motors for each leg
uint8_t leg1[2] = {7,8}; // there are two motors for each leg
uint8_t leg2[2] = {2,1}; // there are two motors for each leg
uint8_t leg3[2] = {3,4}; // there are two motors for each leg

struct LegModes leg_modes; // different modes of the motor

using namespace std::chrono;
Timer t;

float toDEG = 180/PI;
RecievedData recieved_A,recieved_B;


void postion_16bit(struct LegIdentifier legs[]){

    for (int i = 0; i<4; i++){
        float alpha = legs[i].theta + legs[i].gamma; 
        float beta  = legs[i].theta - legs[i].gamma;

        int motorB_pos = float_to_uint(alpha, -95.5, 95.5, 16);
        int motorA_pos = float_to_uint(beta, -95.5, 95.5, 16);

        int var = 0.0 * 1.2;//75
        transmit(can_one, legs[i].motorA, motorA_pos, 2047, 40, 400, 2047-legs[i].tff);
        receive(can_one,recieved_A);
        transmit(can_one, legs[i].motorB, motorB_pos, 2047, 40, 400, 2047+legs[i].tff);
        receive(can_one,recieved_B);
    }

    //printf(" Err A: %f\t",  (alpha - recieved_A.f_position)*toDEG);
    //printf(" Err B: %f\n",  (beta  - recieved_B.f_position)*toDEG);
};


// main() runs in its own thread in the OS
int main()
{
    t.start();
    can_one.frequency(1000000);
    
    struct LegIdentifier legs[4] = {
    //   mA, mB,      theta, gamma
        // Left Leg
        {4, 3, -650, 0.0, 0.0}, // leg0
        {1, 2, -650, 0.0, 0.0}, // leg1
        // Right leg
        {8, 7, -615, 0.0, 0.0}, // leg2
        {5, 6, -500, 0.0, 0.0} // leg3

    };

    // these are 13 different gait parameters for the 13 different things that the dogg should be able to do
    // we are starting with TROT only, the rest are unchanged from the original code and untested for RILtaur
    struct GaitParams state_gait_params[4] = {
        //{s.h, d.a., u.a., f.p., s.l., fr., s.d.}
        {0.18, 0.00, 0.00, 0.35, 0.00, 0.0, 0.0}, // STAND
        {0.18, 0.03, 0.06, 0.35, 0.15, 5.0, 0.0}, // TROT
        {0.17, 0.04, 0.06, 0.35, 0.0, 2.0, 0.0}, // BOUND
        {0.15, 0.00, 0.06, 0.25, 0.0, 1.5, 0.0}, // WALK
    };

    printf("started...\n");

    // motor mode and zero position
    // for (int i = 0; i<2; i++){
    //     send(can_one, leg3[i], leg_modes.motor_mode, 8);
    //     send(can_one, leg3[i], leg_modes.zero_mode, 8);
    // }

    for (int i = 0; i<4; i++){
        send(can_one, legs[i].motorA, leg_modes.motor_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.motor_mode, 8);

        send(can_one, legs[i].motorA, leg_modes.zero_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.zero_mode, 8);
        printf("IDs are %i, %i\n", legs[i].motorA, legs[i].motorB);
    }

    // send(can_one, 1, leg_modes.motor_mode, 8);
    // send(can_one, 1, leg_modes.zero_mode, 8);

    while(true){
        unsigned long long time_ms = duration_cast<milliseconds>(t.elapsed_time()).count(); // get the system time in milliseconds
        // printf(" time in millis %llu \n", time_ms);
        gait(legs, state_gait_params[0], time_ms, 0.0, 0.0, 0.5, 0.5); //0.0, 0.5, 0.0, 0.5
        // wait_us(350000);
        postion_16bit(legs);

    }



}


