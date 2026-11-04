#include "stm32f10x.h"

#include <stdio.h>
#include "pid.h"

int main(void)
{
    /* --- 把所有变量声明放在最前面 --- */
    PID pid;
    float setpoint = 50.0f;
    float measure = 0.0f;
    float u;
    int i; 

    PID_Init(&pid, 2.0f, 0.5f, 0.05f, 0.01f);
    PID_SetOutputLimit(&pid, -100.0f, 100.0f);
    PID_SetIntegralLimit(&pid, -50.0f, 50.0f);

    for (i = 0; i < 1000; i++)  /* 这里直接使用 i，不写 int */
    {
        u = PID_Compute(&pid, setpoint, measure);
        
        measure += (1.0f * u - measure) * 0.01f / 0.5f;
        
        if (i % 100 == 0)
            printf("t=%5.2fs  u=%8.3f  y=%8.3f\n", i * 0.01f, u, measure);
    }
    return 0;
}