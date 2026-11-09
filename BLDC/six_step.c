#include "tim.h"

/* ============================================================
 * 六步换相(每步 60 电角度)
 *   1 U+ V- W悬空    2 U+ W- V悬空    3 V+ W- U悬空
 *   4 V+ U- W悬空    5 W+ U- V悬空    6 W+ V- U悬空
 *
 * 引脚(浩盛单路电机开发板 V2.0)
 *   A相(U): PWM=PC0/CH1  SD1=PB0  采样 BEMF_A=PC4/ADC2_IN5  -> Adc[1]
 *   B相(V): PWM=PC1/CH2  SD2=PA1  采样 BEMF_B=PA7/ADC2_IN4  -> Adc[0]
 *   C相(W): PWM=PC2/CH3  SD3=PA2  采样 BEMF_C=PA5/ADC2_IN13 -> Adc[2]
 *
 *   SD#=1 : 这一相正常, 由 IN(PWM) 控制上下管
 *   SD#=0 : 这一相上下管全关 -> 真正悬空, 才能测到反电动势
 * ============================================================ */

/* 每相的 SD 脚 */
static GPIO_TypeDef *const SD_Port[4] = {0, GPIOB, GPIOA, GPIOA};
static const uint16_t      SD_Pin [4] = {0, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2};

/* 换相表: {上桥PWM相, 下桥常通相, 悬空相} */
static const uint8_t StepTab[6][3] = {
	{1,2,3}, {1,3,2}, {2,3,1}, {2,1,3}, {3,1,2}, {3,2,1}
};

/* 过零表: {悬空相对应的 Adc 下标, 方向}  方向 1 = 等它跌到中性点以下 */
static const uint8_t ZcTab[6][2] = {
	{2,1}, {0,0}, {1,1}, {2,0}, {0,1}, {1,0}
};

extern uint16_t pwm_cout;   /* 换相计时, 由 TIM1 CH4 中断累加 */
extern uint16_t Adc[];

/* 设置占空比 duty = 0~100 */
void TIM_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t duty)
{
	/* 先乘后除, 否则 duty/100 恒为 0 */
	__HAL_TIM_SET_COMPARE(htim, channel, (uint32_t)duty * (__HAL_TIM_GET_AUTORELOAD(htim) + 1) / 100);
}

static void Phase_EN(uint8_t ch, uint8_t on)
{
	HAL_GPIO_WritePin(SD_Port[ch], SD_Pin[ch], on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 六步换相: 一相上桥 PWM, 一相下桥常通, 第三相 SD 拉低悬空 */
void six_step_control(uint8_t step, uint8_t duty)
{
	uint8_t pwm, sink, flt;
	if(step>5) return;

	pwm  = StepTab[step][0];
	sink = StepTab[step][1];
	flt  = StepTab[step][2];

	TIM_SetDuty(&htim1, TIM_CHANNEL_1, pwm==1 ? duty : 0);
	TIM_SetDuty(&htim1, TIM_CHANNEL_2, pwm==2 ? duty : 0);
	TIM_SetDuty(&htim1, TIM_CHANNEL_3, pwm==3 ? duty : 0);

	Phase_EN(flt , 0);      /* 先让该悬空的相悬空, 再开新的一相, 避免两相同时下地 */
	Phase_EN(pwm , 1);
	Phase_EN(sink, 1);
}

/* 悬空相是否已经过零; 中性点用三相端电压平均近似 */
uint8_t BEMF_Check(uint8_t step)
{
	uint16_t mid, v;
	if(step>5) return 0;

	mid = (Adc[0] + Adc[1] + Adc[2]) / 3;
	v   = Adc[ZcTab[step][0]];

	return ZcTab[step][1] ? (v < mid) : (v > mid);
}

/* 安全停机: 三相全悬空 = 自由停车
   (不要用 six_step_control(0,0), 那是两相下桥常通 = 短路刹车, 会拉满电流) */
void MotorStop(void)
{
	TIM_SetDuty(&htim1, TIM_CHANNEL_1, 0);
	TIM_SetDuty(&htim1, TIM_CHANNEL_2, 0);
	TIM_SetDuty(&htim1, TIM_CHANNEL_3, 0);
	Phase_EN(1, 0);
	Phase_EN(2, 0);
	Phase_EN(3, 0);
	while(1) { __NOP(); }
}
