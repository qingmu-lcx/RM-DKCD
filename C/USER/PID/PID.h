#ifndef _PID_H_
#define _PID_H_

#include "main.h"

#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)


enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float OUT;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float set;
    float get;
    float err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float OUT;
} Gimbal_PID_t;


void abs_limit(float *num, float Limit);
float loop_fp32_constrain(float Input, float minValue, float maxValue);

extern void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_iout , float max_out);
float PID_Calc(PidTypeDef *pid, float ref, float set);
extern void PID_clear(PidTypeDef *pid);

extern void GIMBAL_PID_Init(Gimbal_PID_t *pid, float max_iout, float maxout, float kp, float ki, float kd);
extern float GIMBAL_PID_Calc(Gimbal_PID_t *pid, float get, float set, float error_delta);
extern void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);

#endif
