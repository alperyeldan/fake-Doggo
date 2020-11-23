#ifndef CAN_COMM_H
#define CAN_COMM_H

#include <cstdint>
#define PI 3.14159265359f
#define SQRT3 1.73205080757f
#define P_MIN -95.5f
#define P_MAX 95.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
#define I_MIN -40.0f
#define I_MAX 40.0f

#include "mbed.h"
#include "math.h"
#include "CAN.h"



// conversion
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

// CAN message definitions
struct LegModes{
    uint8_t motor_mode[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc}; // enters motor mode/closed loop control
    uint8_t exit_mode[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd}; // exits the closed loop control
    uint8_t zero_mode[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe}; // set current position to zero
};

// identifying each leg and their corresponding parameters
struct LegIdentifier{
    uint8_t motorA = 3;
    uint8_t motorB = 4;
    float theta = 0.0;
    float gamma = 0.0;
};

struct RecievedData{
    uint16_t u_position;
    uint16_t u_velocity;
    uint16_t u_current;
    uint8_t  u_ID;

    float f_position;
    float f_velocity;
    float f_current;
    float f_ID;
};

// can communication
void send(CAN& can_interface, uint8_t can_id, uint8_t cmsg[], int dlc);
void receive(CAN& can_interface,RecievedData& rcvd);
void transmit(CAN& can_interface, uint8_t can_id, int position, int velocity, int kp, int kd, int ff);

#endif
