/*
 * joint.h
 *
 *  Created on: Jan 20, 2026
 *      Author: USER
 */

#ifndef JOINT_H
#define JOINT_H

#include "tim.h"
#include "pid.h"
#include "motor.h"

typedef enum { POS_MODE, VEL_MODE, POSVEL_MODE } Mode_t;

typedef struct {
    Encoder_t enc;
    Motor_t mot;
    PID_t pos_pid, vel_pid;

    float outer_val;
    float theta_last;
    float theta, omega;
    float theta_ref, omega_ref;
    float gear;
    Mode_t mode;
    uint8_t calibration;
} Joint_t;

extern Joint_t joint[6];

void Joint_Init(Joint_t *j);
void Joint_Update(Joint_t *j, float dt);
void joint_config(void);

#endif
 /* INC_JOINT_H_ */
