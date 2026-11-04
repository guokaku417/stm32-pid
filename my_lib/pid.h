#ifndef PID_H
#define PID_H

#define PID_MODE_POSITION  0   /* 位置式 */
#define PID_MODE_INCREMENT 1   /* 增量式 */

typedef struct {
    /* --- 参数 --- */
    float kp, ki, kd;
    float dt;                  /* 采样周期(秒) */

    /* --- 限幅 --- */
    float out_min, out_max;    /* 输出限幅 */
    float integral_min, integral_max; /* 积分项限幅 */

    /* --- 内部状态 --- */
    float integral;            /* 积分累积 */
    float prev_error;          /* e(k-1) */
    float prev_error2;         /* e(k-2) */
    float prev_measure;        /* 上一次测量值 */
    float prev_output;         /* 上一次输出(增量式用) */

    int   mode;
    int   first_run;
} PID;

void  PID_Init(PID *pid, float kp, float ki, float kd, float dt);
void  PID_SetOutputLimit(PID *pid, float out_min, float out_max);
void  PID_SetIntegralLimit(PID *pid, float i_min, float i_max);
void  PID_SetMode(PID *pid, int mode);
void  PID_Reset(PID *pid);
float PID_Compute(PID *pid, float setpoint, float measure);

#endif
