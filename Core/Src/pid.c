/*
 * pid.c
 *
 *  Created on: Jan 20, 2026
 *      Author: USER
 */


#include "pid.h"
#include "math.h"

void PID_Init(PID_t *p, float kp, float ki, float kd,
              float min, float max)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integral = 0;
    p->prev_err = 0;
    p->out_min = min;
    p->out_max = max;
}

float PID_Update(PID_t *p, float err, float dt)
{
    float d = (err - p->prev_err) / dt;
    float out = p->kp * err + p->ki * p->integral + p->kd * d;

    float u = out;

	if(out > p->out_max)out = p->out_max;
	if(out < p->out_min)out = p->out_min;

    if (u <= p->out_max && u >= p->out_min){
        p->integral += 0.5f * (err + p->prev_err) * dt;
    }

    p->prev_err = err;
    return out;
}

