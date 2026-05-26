/*
 * joint.c
 *
 *  Created on: Jan 20, 2026
 *      Author: USER
 */


#include "joint.h"
#include <math.h>

Joint_t joint[6];

void Joint_Init(Joint_t *j)
{
    j->outer_val = 0;
    j->theta_last = 0;
    j->theta = 0;
    j->omega = 0;
}

void Joint_Update(Joint_t *j, float dt)
{
    Encoder_Update(&j->enc);

    //deg, deg/s
    j->theta = (float)j->enc.total / (j->enc.ppr * j->gear) * 360; //2.0f * M_PI;
    j->omega = (float)j->enc.diff * 360.0f / (j->enc.ppr * j->gear) / dt;
    j->theta_last = j->theta;

    if(j->mode == POS_MODE){
    	if(fabs(j->theta - j->theta_ref) > 0.08f){
    		j->mot.volt = PID_Update(&j->pos_pid, j->theta_ref - j->theta, dt);
    	}
    }
    else if(j->mode == VEL_MODE){
    	if(fabs(j->omega - j->omega_ref) > 0.5f){
    		j->mot.volt = PID_Update(&j->vel_pid, j->omega_ref - j->omega, dt);
    	}
    }
    else{

////////////
//    		j->outer_val = PID_Update(&j->pos_pid, j->theta_ref - j->theta, dt);
//    		j->mot.volt = PID_Update(&j->vel_pid, j->outer_val - j->omega, dt);
//    		if(fabs(j->mot.volt) < (j->mot.max_volt*j->mot.min)){
//    			if(j->mot.volt > 0.05)j->mot.volt = j->mot.max_volt*j->mot.min;
//    			if(j->mot.volt < -0.05)j->mot.volt = j->mot.min_volt*j->mot.min;
//    		}
////////////
		float pos_err = j->theta_ref - j->theta;
		j->outer_val = PID_Update(&j->pos_pid, pos_err, dt);
		j->outer_val += j->omega_ref;
		j->mot.volt = PID_Update(&j->vel_pid, j->outer_val - j->omega, dt);
		if(fabs(j->mot.volt) < (j->mot.max_volt * j->mot.min_volt)) {
			if(fabs(pos_err) > 0.1f || fabs(j->omega_ref) > 0.1f) {
				 if(j->mot.volt > 0) j->mot.volt = j->mot.max_volt * j->mot.min_volt;
				 else                j->mot.volt = -(j->mot.max_volt) * j->mot.min_volt;
			} else {
				 j->mot.volt = 0.0f;
			}
		}
    }
    Motor_Set(&j->mot, j->mot.volt); //j->mot.volt
}

void joint_config(){

	/* ===== Encoder ===== */
//	joint[0].enc.hw.timer.htim = &htim1;
	joint[0].enc.type = ENC_TYPE_GPIO; //ENC_TYPE_TIMER
	joint[0].enc.ppr = 32; //64
	joint[0].enc.hw.gpio.portA = GPIOD;
	joint[0].enc.hw.gpio.pinA = GPIO_PIN_2;
	joint[0].enc.hw.gpio.portB = GPIOC;
	joint[0].enc.hw.gpio.pinB = GPIO_PIN_12;

	joint[1].enc.hw.timer.htim = &htim2;
	joint[1].enc.type = ENC_TYPE_TIMER;
	joint[1].enc.ppr = 64;

	joint[2].enc.hw.timer.htim = &htim3;
	joint[2].enc.type = ENC_TYPE_TIMER;
	joint[2].enc.ppr = 64;

	joint[3].enc.hw.timer.htim = &htim4;
	joint[3].enc.type = ENC_TYPE_TIMER;
	joint[3].enc.ppr = 44;

	joint[4].enc.hw.timer.htim = &htim5;
	joint[4].enc.type = ENC_TYPE_TIMER;
	joint[4].enc.ppr = 44;

	joint[5].enc.hw.timer.htim = &htim1;
//	joint[5].enc.hw.gpio.portA = GPIOD;
//	joint[5].enc.hw.gpio.pinA = GPIO_PIN_2;
//	joint[5].enc.hw.gpio.portB = GPIOC;
//	joint[5].enc.hw.gpio.pinB = GPIO_PIN_12;
	joint[5].enc.type = ENC_TYPE_TIMER; //ENC_TYPE_GPIO;
//	joint[5].enc.hw.gpio.counter = 0;
	joint[5].enc.ppr = 44;

	for(int i=1; i<=5; i++){
		Encoder_Init(&joint[i].enc);
		HAL_TIM_Encoder_Start(joint[i].enc.hw.timer.htim, TIM_CHANNEL_ALL);
	}

	/* ===== Motor ===== */
	joint[0].mot.htim = &htim10;
	joint[0].mot.mode = MOTOR_DIR_IO;
	joint[0].mot.io.channel = TIM_CHANNEL_1;
	joint[0].mot.io.dir_port = GPIOC;
	joint[0].mot.io.dirA = GPIO_PIN_3;
	joint[0].mot.io.dirB = GPIO_PIN_4;
	HAL_TIM_PWM_Start(joint[0].mot.htim, TIM_CHANNEL_1);

	joint[1].mot.htim = &htim8;
	joint[1].mot.mode = MOTOR_DIR_PWM;
	joint[1].mot.pwm.channelA = TIM_CHANNEL_1;
	joint[1].mot.pwm.channelB = TIM_CHANNEL_2;
	HAL_TIM_PWM_Start(joint[1].mot.htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(joint[1].mot.htim, TIM_CHANNEL_2);

	joint[2].mot.htim = &htim12;
	joint[2].mot.mode = MOTOR_DIR_PWM;
	joint[2].mot.pwm.channelA = TIM_CHANNEL_1;
	joint[2].mot.pwm.channelB = TIM_CHANNEL_2;
	HAL_TIM_PWM_Start(joint[2].mot.htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(joint[2].mot.htim, TIM_CHANNEL_2);

	joint[3].mot.htim = &htim11;
	joint[3].mot.mode = MOTOR_DIR_IO;
	joint[3].mot.io.channel = TIM_CHANNEL_1;
	joint[3].mot.io.dir_port = GPIOB;
	joint[3].mot.io.dirA = GPIO_PIN_1;
	joint[3].mot.io.dirB = GPIO_PIN_2;
	HAL_TIM_PWM_Start(joint[3].mot.htim, TIM_CHANNEL_1);

	joint[4].mot.htim = &htim13;
	joint[4].mot.mode = MOTOR_DIR_IO;
	joint[4].mot.io.channel = TIM_CHANNEL_1;
	joint[4].mot.io.dir_port = GPIOB;
	joint[4].mot.io.dirA = GPIO_PIN_12;
	joint[4].mot.io.dirB = GPIO_PIN_13;
	HAL_TIM_PWM_Start(joint[4].mot.htim, TIM_CHANNEL_1);

	joint[5].mot.htim = &htim14;
	joint[5].mot.mode = MOTOR_DIR_IO;
	joint[5].mot.io.channel = TIM_CHANNEL_1;
	joint[5].mot.io.dir_port = GPIOC;
	joint[5].mot.io.dirA = GPIO_PIN_8;
	joint[5].mot.io.dirB = GPIO_PIN_9;
	HAL_TIM_PWM_Start(joint[5].mot.htim, TIM_CHANNEL_1);

	for(int i=0; i<=5; i++){
		Motor_Init(&joint[i].mot);
	}

	/* ===== Joint ===== */
	joint[0].gear = 596.0f;
	joint[0].mode = VEL_MODE;
	joint[0].theta_ref = 0.0;
	joint[0].omega_ref = 0.0;

	joint[1].gear = 721.0f;
	joint[1].mode = VEL_MODE;
	joint[1].theta_ref = 0.0;
	joint[1].omega_ref = 0.0;

	joint[2].gear = 427.0f;
	joint[2].mode = VEL_MODE;
	joint[2].theta_ref = 0.0;
	joint[2].omega_ref = 0.0;

	joint[3].gear = 500.0f;
	joint[3].mode = VEL_MODE;
	joint[3].theta_ref = 0.0;
	joint[3].omega_ref = 0.0;

	joint[4].gear = 488.0f;
	joint[4].mode = VEL_MODE;
	joint[4].theta_ref = 0.0;
	joint[4].omega_ref = 0.0;

	joint[5].gear = 488.0f;
	joint[5].mode = VEL_MODE;
	joint[5].theta_ref = 0.0;
	joint[5].omega_ref = 0.0;

	for(int i=0; i<=5; i++)Joint_Init(&joint[i]);

	/* ===== PID ===== */
	PID_Init(&joint[0].pos_pid, 5.4, 0.5, 0.32, -5, 5);
	PID_Init(&joint[0].vel_pid, 0.1071, 1.7, 0.0, -11.5, 11.5);

	PID_Init(&joint[1].pos_pid, 2.1, 0.1, 0.02, -5, 5);
	PID_Init(&joint[1].vel_pid, 0.1, 0.73, 0.0, -11.5, 11.5);
//	PID_Init(&joint[1].vel_pid, 0.0, 0, 0.0, -11.5, 11.5);

	PID_Init(&joint[2].pos_pid, 2.1, 0.1, 0.02, -5, 5);
	PID_Init(&joint[2].vel_pid, 0.1, 0.69, 0.0, -11.5, 11.5);
//	PID_Init(&joint[2].vel_pid, 0.0, 0, 0.0, -11.5, 11.5);

	PID_Init(&joint[3].pos_pid, 5.4, 0.5, 0.32, -5, 5);
	PID_Init(&joint[3].vel_pid, 0.025, 1.2, 0.0, -11.5, 11.5);
//	PID_Init(&joint[3].vel_pid, 0.0, 0.0, 0.0, -11.5, 11.5);

	PID_Init(&joint[4].pos_pid, 5.4, 0.5, 0.32, -5, 5);
	PID_Init(&joint[4].vel_pid, 0.025, 1.079074, 0.0, -11.5, 11.5);
//	PID_Init(&joint[4].vel_pid, 0.0, 0, 0.0, -11.5, 11.5);

	PID_Init(&joint[5].pos_pid, 5.4, 0.5, 0.32, -5, 5);
	PID_Init(&joint[5].vel_pid, 0.025, 1.079074, 0.0, -11.5, 11.5);
//	PID_Init(&joint[5].pos_pid, 13.5, 0.01, 1.8, -70, 70);
}

