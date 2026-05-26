

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f4xx_hal.h"

typedef enum {
	MOTOR_DIR_IO,
	MOTOR_DIR_PWM
} MotorDirMode_t;

typedef enum {
    ENC_TYPE_TIMER,
    ENC_TYPE_GPIO
} EncType_t;

typedef struct {
    TIM_HandleTypeDef *htim;
} TimerEnc_t;

typedef struct {
    GPIO_TypeDef *portA;
    uint16_t pinA;
    GPIO_TypeDef *portB;
    uint16_t pinB;
    volatile int32_t counter;
} GpioEnc_t;

typedef struct {
    TIM_HandleTypeDef *htim;

    MotorDirMode_t mode;

    union{
    	struct{
    		uint32_t channel;
    		GPIO_TypeDef *dir_port;
    		uint16_t dirA, dirB;
    	} io;
    	struct{
			TIM_HandleTypeDef *htim;
			uint32_t channelA, channelB;
    	} pwm;
    };

    float volt;
    uint16_t pwm_max;
    float max_volt;
    float min_volt;
} Motor_t;

typedef struct {
	EncType_t type;

	union {
		TimerEnc_t timer;
		GpioEnc_t  gpio;
	} hw;

    int32_t last;
    int64_t total;
    uint16_t ppr;
    int32_t diff;
} Encoder_t;


void Encoder_Init(Encoder_t *e);
void Encoder_Update(Encoder_t *e);
void Motor_Init(Motor_t *m);
void Motor_Set(Motor_t *m, float input);

#endif /* INC_MOTOR_H_ */
