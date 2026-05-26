
#include "motor.h"
#include "math.h"

void Motor_Init(Motor_t *m){
	m->volt = 0;
	m->pwm_max = 1000;
	m->max_volt = 11.5;
	m->min_volt = 0.05;
}

void Encoder_Init(Encoder_t *e)
{
    e->last = 0;
    e->total = 0;
    e->diff = 0;
}

void Motor_Set(Motor_t *m, float input_volt){
	if(m->mode == MOTOR_DIR_IO){
		if(input_volt > m->min_volt)
		{
			HAL_GPIO_WritePin(m->io.dir_port, m->io.dirA, GPIO_PIN_SET);
			HAL_GPIO_WritePin(m->io.dir_port, m->io.dirB, GPIO_PIN_RESET);
		}
		else if(input_volt < -(m->min_volt))
		{
			HAL_GPIO_WritePin(m->io.dir_port, m->io.dirA, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(m->io.dir_port, m->io.dirB, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(m->io.dir_port, m->io.dirA, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(m->io.dir_port, m->io.dirB, GPIO_PIN_RESET);
		}

		if(input_volt > m->max_volt) input_volt = m->max_volt;
		if(input_volt < -(m->max_volt)) input_volt = -(m->max_volt);

		float duty = fabsf(input_volt / m->max_volt * m->pwm_max);
		__HAL_TIM_SET_COMPARE(m->htim, m->io.channel, (uint16_t)duty);
	}

	else if(m->mode == MOTOR_DIR_PWM){
		if(input_volt > m->max_volt) input_volt = m->max_volt;
		if(input_volt < -(m->max_volt)) input_volt = -(m->max_volt);
		float duty = fabsf(input_volt / m->max_volt * m->pwm_max);

		if(input_volt > m->min_volt)
		{
			__HAL_TIM_SET_COMPARE(m->htim, m->pwm.channelA, (uint16_t)duty);
			__HAL_TIM_SET_COMPARE(m->htim, m->pwm.channelB, 0);

		}
		else if(input_volt < -(m->min_volt))
		{
			__HAL_TIM_SET_COMPARE(m->htim, m->pwm.channelA, 0);
			__HAL_TIM_SET_COMPARE(m->htim, m->pwm.channelB, (uint16_t)duty);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(m->htim, m->pwm.channelA, 0);
			__HAL_TIM_SET_COMPARE(m->htim, m->pwm.channelB, 0);
		}
	}
}

void Encoder_Update(Encoder_t *e)
{
	int32_t now = 0;

	if (e->type == ENC_TYPE_TIMER)
	{
		now = __HAL_TIM_GET_COUNTER(e->hw.timer.htim);
		e->diff = now - e->last;
	}
	else if (e->type == ENC_TYPE_GPIO)
	{
		now = e->hw.gpio.counter;
		e->diff = now - e->last;
	}

	if (e->diff > 32768)  e->diff -= 65536;
	if (e->diff < -32768) e->diff += 65536;
	e->total += e->diff;
	e->last = now;
}


