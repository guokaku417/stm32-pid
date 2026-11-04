#include "pid.h"

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void PID_Init(PID *pid, float kp, float ki, float kd, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = (dt > 0.0f) ? dt : 0.001f;

    pid->out_min = -1e9f;
    pid->out_max =  1e9f;
    pid->integral_min = -1e9f;
    pid->integral_max =  1e9f;

    pid->integral     = 0.0f;
    pid->prev_error   = 0.0f;
    pid->prev_error2  = 0.0f;
    pid->prev_measure = 0.0f;
    pid->prev_output  = 0.0f;

    pid->mode = PID_MODE_POSITION;
    pid->first_run = 1;
}

void PID_SetOutputLimit(PID *pid, float out_min, float out_max)
{
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void PID_SetIntegralLimit(PID *pid, float i_min, float i_max)
{
    pid->integral_min = i_min;
    pid->integral_max = i_max;
}

void PID_SetMode(PID *pid, int mode)
{
    pid->mode = mode;
    PID_Reset(pid);
}

void PID_Reset(PID *pid)
{
    pid->integral     = 0.0f;
    pid->prev_error   = 0.0f;
    pid->prev_error2  = 0.0f;
    pid->prev_measure = 0.0f;
    pid->prev_output  = 0.0f;
    pid->first_run    = 1;
}

float PID_Compute(PID *pid, float setpoint, float measure)
{
    float error  = setpoint - measure;
    float output = 0.0f;

    if (pid->mode == PID_MODE_POSITION)
    {
        /* 比例项 */
        float p = pid->kp * error;

        /* 积分项(矩形积分),内部限幅 */
        float d_integral = pid->ki * error * pid->dt;
        pid->integral = clampf(pid->integral + d_integral,
                               pid->integral_min, pid->integral_max);

        /* 微分先行:对测量值微分,避免设定值阶跃引起的微分冲击 */
        float d = 0.0f;
        if (!pid->first_run)
            d = -pid->kd * (measure - pid->prev_measure) / pid->dt;

        output = p + pid->integral + d;

        /* 输出限幅 + 抗积分饱和(条件积分法) */
        if (output > pid->out_max || output < pid->out_min)
        {
            output = clampf(output, pid->out_min, pid->out_max);
            pid->integral = clampf(pid->integral - d_integral,  /* 回退本次积分 */
                                   pid->integral_min, pid->integral_max);
        }

        pid->prev_measure = measure;
    }
    else /* PID_MODE_INCREMENT */
    {
        float d_u = pid->kp * (error - pid->prev_error)
                  + pid->ki * error * pid->dt
                  + pid->kd * (error - 2.0f * pid->prev_error + pid->prev_error2) / pid->dt;

        if (pid->first_run)          /* 首次运行退化为纯比例 */
            d_u = pid->kp * error;

        /* 输出累加后限幅,增量式天然抗积分饱和 */
        output = clampf(pid->prev_output + d_u, pid->out_min, pid->out_max);

        pid->prev_error2 = pid->prev_error;
        pid->prev_error  = error;
        pid->prev_output = output;
    }

    pid->first_run = 0;
    return output;
}
