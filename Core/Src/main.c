/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "six_step.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t Adc[3]={0};

uint16_t pwm_cout=0;
uint8_t step=0;

/* ================= 起步(开环)参数 ================= */
uint8_t  DutyOpen = 10;   /* 起步占空比: 要小! 开环低速时电流 = 占空比xVbus/(2R), 大了就爆 */
uint16_t StepMs   = 5;    /* 开环每步时间 ms */

/* ================= 运行(闭环)参数 ================= */
uint8_t  DutyRun  = 5;   /* 闭环目标占空比: 原来是 36. 先试 45, 稳了再往 55/65 加 */

/* ================= 换相时序参数(必须和 tim.c 对得上) =================
 * TIM1: 170MHz, 中心对齐, ARR=4250 -> PWM 20kHz(周期 50us)
 * CH4(CCR=1) 每周期中断 2 次 -> 1 count = 25us
 * -------------------------------------------------------------------- */
#define BLANK_TICKS   3     /* 消隐: 3*25us = 75us  (原来是 5 = 125us, 太长) */
#define ZC_LAG        1     /* ZC检测的固有滞后(ADC扫描周期): 提前 1*25us 换相, 可微调 */
#define T60_MIN       8     /* 一步时间的合理下限, 防噪声误判 */
#define T60_INIT      200   /* 进闭环首步的默认周期 = 5ms(和开环 StepMs 一致) */
#define STALL_TICKS   200   /* 一步超 200*25us=5ms 没换相 = 失步 (原来 2000=50ms) */
#define FAULT_LIMIT   20    /* 连续自救这么多次还不行 = 真堵转, 停机 */
#define DUTY_MAX      90    /* 硬件安全上限: EG2104 是自举驱动, 占空比太高时下桥导通时间
                               太短, 自举电容补不上电, 上桥栅压会掉下去(进线性区发热) */
uint8_t  DutyNow  = 10;    /* 当前实际占空比(进闭环后从 DutyOpen 慢慢升到 DutyRun) */
uint8_t  RampDiv  = 6;    /* 几次换相升1%: 数字越大升得越慢(升不上去/失步就加大) */
uint8_t  RampCnt  = 0;    /* 升压节拍 */

/* ================= 以下不用改 ================= */
uint16_t T60       = 0;  /* 上一步的 60 度时间(counts), 由换相间隔测得, 不受消隐影响 */
uint16_t zc_at     = 0;  /* 本步过零点出现的时刻(相对上次换相) */
uint16_t FaultCnt  = 0;  /* 连续失步自救次数 */
uint16_t zc_cnt    = 0;  /* 开环阶段攒到的过零次数 */
uint8_t  FlagBEMF  = 0;  /* 闭环: 这一步是否已经检测到过零 */
uint8_t  zc_flag   = 0;  /* 开环: 这一步里有没有看到过零 */
uint8_t  i         = 0;  /* 循环用 */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 换到下一相: 步进 / 下发占空比 / 计时清零 */
static void Commutate(uint8_t duty)
{
	step++;
	if(step>=6) step=0;
	six_step_control(step, duty);
	pwm_cout = 0;
	FlagBEMF = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
 HAL_TIM_Base_Start(&htim1);
 HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_4);
 
 TIM1->CCR4=1;                                //每次计数到1就中断, 频率最高
 
 HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
 HAL_ADC_Start_DMA(&hadc2,(uint32_t*)Adc,3);   // 长度=通道数3 (写4会让Adc[]相位每轮错位)

HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);


uint8_t  State=0;          /* 0=开环计数  1=闭环 */
six_step_control(0,10);    /* 预定位 */
HAL_Delay(200);
pwm_cout=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	switch(State)
	{
	case 0:   /* ---------- 开环: 定时换相, 边转边数过零 ---------- */
		zc_flag = 0;
		for(i=0;i<StepMs;i++)          /* 一步切成 StepMs 个 1ms, 边转边采样 */
		{
			HAL_Delay(1);
			if(zc_flag==0 && BEMF_Check(step)) zc_flag=1;
		}
		if(zc_flag) zc_cnt++;

		Commutate(DutyOpen);

		if(zc_cnt>=20)                 /* 攒够 20 次过零 = 信号稳了, 进闭环 */
		{
			zc_cnt=0; FlagBEMF=0;
			DutyNow=DutyOpen; RampCnt=0;
			State++;
		}
		break;

	case 1:   /* ---------- 闭环: 过零 -> 再等半个60° -> 换相 ---------- */
		if(FlagBEMF==0 && pwm_cout>=BLANK_TICKS && BEMF_Check(step))
		{
			FlagBEMF = 1;
			zc_at    = pwm_cout;       /* 消隐后检测到过零, 记下时刻 */
		}
		/* 换相点 = 过零时刻 + T60/2; T60 取"上一步的换相间隔", 与消隐无关 */
		if(FlagBEMF && (uint16_t)(pwm_cout+ZC_LAG) >= (uint16_t)(zc_at + (T60?T60:T60_INIT)/2))
		{
			if(pwm_cout>T60_MIN) T60 = T60 ? (uint16_t)((T60*3+pwm_cout)>>2) : pwm_cout;
			Commutate(DutyNow);
			FaultCnt = 0;

			/* 升压斜坡: 每 RampDiv 次换相升 1%, 慢慢升到 DutyRun */
			if(DutyNow<DutyRun && DutyNow<DUTY_MAX && ++RampCnt>=RampDiv)
			{
				RampCnt = 0;
				DutyNow++;
			}
		}
		/* 过零一直不来 = 失步: 强制换相自救(超时取上一步周期的2倍, 最低5ms)
		   不能无条件用 STALL_TICKS, 否则转速低、一步超过 5ms 时会把正常换相当成失步 */
		else if(FlagBEMF==0 && pwm_cout > (T60*2 > STALL_TICKS ? T60*2 : STALL_TICKS))
		{
			Commutate(DutyNow);
			if(++FaultCnt > FAULT_LIMIT) MotorStop();
		}
		break;

	}

    /* USER COD0E END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//串口重定向
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{
		pwm_cout++;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
