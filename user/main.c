#include "delay.h"
#include "pid.h"
#include "stm32f10x.h"
#include "usart.h"

/* ============================================================================
 * 接线对照（来自接线表）
 *   驱动板 P (PWM) -> PA7  (TIM3_CH2)
 *   驱动板 A       -> PA0  (方向电平 A)
 *   驱动板 B       -> PA1  (方向电平 B)
 *   编码器 A 相    -> PB6  (TIM4_CH1)
 *   编码器 B 相    -> PB7  (TIM4_CH2)
 *   驱动板 V/G、编码器 VCC/GND -> 3.3V / GND
 *
 * 控制思路：每 20ms 读一次编码器计数差值，换算成转速(rpm)，
 * 送进你自己的 PID 算法，输出 -100~100 的占空比百分比，
 * 由 A/B 两个方向脚决定正反转，PWM 脚给占空比。
 * ==========================================================================*/

/* ---------------- 可调参数 ---------------- */
#define CONTROL_PERIOD_MS 20u /* 控制周期，单位 ms */
#define CONTROL_PERIOD_S                                                       \
  ((float)CONTROL_PERIOD_MS / 1000.0f) /* 控制周期，单位 s */

#define PWM_ARR 3599u /* 72MHz/(3599+1) = 20kHz PWM */

/* 编码器规格：必须按你的电机实际参数改，否则转速显示不准
 * （闭环调节本身仍能工作，只是"目标转速"对应的物理转速不同）
 * COUNTS_PER_REV = 编码器线数 × 4(四倍频) × 减速比 */
#define ENCODER_PPR 11.0f
#define GEAR_RATIO 48.0f
#define COUNTS_PER_REV (ENCODER_PPR * 4.0f * GEAR_RATIO)

#define TARGET_RPM 100.0f   /* 目标转速(rpm) */
#define MOTOR_DEADZONE 1.5f /* 占空比死区(%)，防止零点附近抖动 */

/* 电机接线方向如果反了，把这里改成 0 即可 */
#define MOTOR_DIR_INVERT 1

/* ---------------- 全局变量 ---------------- */
static PID pid;
static int16_t last_count = 0; /* 上一次读到的编码器计数 */

/* ---------------- PWM (PA7 / TIM3_CH2) ---------------- */
static void Motor_PWM_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* PA7 复用推挽，接驱动板 P 脚 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* 定时器 72MHz 不分频，20kHz */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = PWM_ARR;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure); /* CH2 -> PA7 */
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_Cmd(TIM3, ENABLE);
}

/* 设置占空比，percent = 0 ~ 100 */
static void Motor_SetDuty(float percent) {
  uint32_t ccr;

  if (percent < 0.0f)
    percent = 0.0f;
  if (percent > 100.0f)
    percent = 100.0f;

  ccr = (uint32_t)(percent * (float)(PWM_ARR + 1) / 100.0f);
  if (ccr > PWM_ARR)
    ccr = PWM_ARR;

  TIM_SetCompare2(TIM3, ccr);
}

/* ---------------- 方向控制 (PA0 / PA1) ---------------- */
static void Motor_Dir_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);
}

/* dir: 1 正转, -1 反转, 0 停止 */
static void Motor_SetDir(int dir) {
#if MOTOR_DIR_INVERT
  dir = -dir;
#endif

  if (dir > 0) {
    GPIO_SetBits(GPIOA, GPIO_Pin_0);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
  } else if (dir < 0) {
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
  } else {
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
  }
}

/* ---------------- 编码器 (PB6/PB7 / TIM4) ---------------- */
static void Encoder_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; /* 上拉输入 */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF; /* 16 位满量程计数 */
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* 编码器模式：A/B 两相，上下沿都计数（四倍频） */
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);

  TIM_SetCounter(TIM4, 0);
  TIM_Cmd(TIM4, ENABLE);

  last_count = 0;
}

/* 读取本次周期的编码器增量（已处理 16 位计数器溢出） */
static int16_t Encoder_ReadDelta(void) {
  int16_t now = (int16_t)TIM_GetCounter(TIM4);
  int16_t delta = (int16_t)(now - last_count);

  last_count = now;
  return delta;
}

/* ---------------- 调试串口 USART1 (PA9/PA10, 115200) ----------------
 * 只用来看曲线，不需要上位机时可以整段删掉 */
static void Debug_USART_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; /* TX */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; /* RX */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  USART_Cmd(USART1, ENABLE);
}

/* ---------------- 主函数 ---------------- */
int main(void) {
  uint32_t last_tick;
  uint32_t print_cnt = 0;
  float speed_rpm;
  float u;
  float duty;
  int16_t delta;

  Delay_Init();
  Debug_USART_Init();
  Motor_Dir_Init();
  Motor_PWM_Init();
  Encoder_Init();

  /* PID 初始化：kp, ki, kd, 采样周期 dt */
  PID_Init(&pid, 1.0f, 0.10f, 0.0f, CONTROL_PERIOD_S);
  PID_SetOutputLimit(&pid, -100.0f, 100.0f); /* 输出即占空比百分比 */
  PID_SetIntegralLimit(&pid, -50.0f, 50.0f);
  PID_SetMode(&pid, PID_MODE_POSITION); /* 速度环用位置式即可 */

  last_tick = GetTick();

  My_USART_Printf(USART1, "PID speed loop start, target = %.1f rpm\r\n",
                  TARGET_RPM);

  while (1) {
    /* ---- 每 20ms 执行一次控制 ---- */
    if ((uint32_t)(GetTick() - last_tick) >= CONTROL_PERIOD_MS) {
      last_tick += CONTROL_PERIOD_MS;

      /* 1. 采样：编码器增量 -> 转速 */
      delta = Encoder_ReadDelta();
      speed_rpm = (float)delta * 60.0f / (COUNTS_PER_REV * CONTROL_PERIOD_S);

      /* 2. PID 运算，输出 -100 ~ 100 */
      u = PID_Compute(&pid, TARGET_RPM, speed_rpm);

      /* 3. 死区处理，避免零附近来回抖 */
      if (u > -MOTOR_DEADZONE && u < MOTOR_DEADZONE)
        u = 0.0f;

      /* 4. 方向 + 占空比 */
      if (u > 0.0f) {
        Motor_SetDir(1);
        duty = u;
      } else if (u < 0.0f) {
        Motor_SetDir(-1);
        duty = -u;
      } else {
        Motor_SetDir(0);
        duty = 0.0f;
      }
      Motor_SetDuty(duty);

      /* 5. 每 100ms 打印一次，方便看曲线 */
      if (++print_cnt % 5 == 0) {
        My_USART_Printf(
            USART1, "t=%lums  target=%.1f  speed=%.2f  u=%.2f  duty=%.1f\r\n",
            (unsigned long)GetTick(), TARGET_RPM, speed_rpm, u, duty);
      }
    }
  }
}
