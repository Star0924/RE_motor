/*
 * pid.h
 *
 *  Created on: Jan 20, 2026
 *      Author: USER
 */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_err;
    float out_min, out_max;
} PID_t;

void PID_Init(PID_t *p, float kp, float ki, float kd,
              float min, float max);
float PID_Update(PID_t *p, float err, float dt);

#endif
 /* INC_PID_H_ */
