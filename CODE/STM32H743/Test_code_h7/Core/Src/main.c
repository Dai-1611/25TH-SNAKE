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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>   // strlen, strcpy, memcpy, memset...
#include <stdio.h>    // sprintf, snprintf...
#include "BMI088.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//Phép màu H7, dán vào 
// STM32H743VITX_FLASH.ld (CubeIDE) để đặt các bộ đệm DMA vào vùng RAM_D2, có thể truy cập bởi DMA1/2
// STM32H743XX_FLASH.ld (Cmake) để đặt các bộ đệm DMA vào vùng RAM_D2, có thể truy cập bởi DMA1/2

//  /* DMA buffers must be placed in a memory region accessible by DMA1/2. */
//   .dma_buffer (NOLOAD) :
//   {
//     . = ALIGN(32);
//     *(.dma_buffer)
//     *(.dma_buffer*)
//     . = ALIGN(32);
//   } >RAM_D2

__attribute__((section(".dma_buffer"), aligned(32))) char imu_txbuff[160];
__attribute__((section(".dma_buffer"), aligned(32))) uint8_t u8_rxbuff[20];
__attribute__((section(".dma_buffer"), aligned(32))) uint8_t u8_txbuff[] = "hello cmm!!!!\r\n";
__attribute__((section(".dma_buffer"), aligned(32))) uint16_t ad_get[5];
// char imu_txbuff[160];
// uint8_t u8_rxbuff[20];
// uint8_t u8_txbuff[] = "hello!!!!\r\n";
// uint16_t ad_get[5];
volatile uint8_t tx_done = 1;
volatile unsigned char dipswitch_read = 1;
volatile unsigned char b1 = 0;
volatile unsigned char b3 = 0;
volatile unsigned char button_read = 1;
volatile uint8_t bmi088_accel_dr = 0;
volatile uint8_t bmi088_gyro_dr = 0;
volatile uint32_t bmi088_accel_irq_count = 0;
volatile uint32_t bmi088_gyro_irq_count = 0;
volatile uint32_t cnt = 0;
uint32_t last_dr_report_ms = 0;
uint8_t u8_rxdata;
uint8_t _rxIndex; //con tro cua rxbuff
uint16_t Tx_flag = 0;


volatile int16_t max_left_ad_servo = -10000;
volatile int16_t max_right_ad_servo = 10000;
volatile int pwm_trace;     // Giá trị PWM của motor_st khi dò theo đường tâm
volatile int pwm_kakudo;    // Giá trị PWM của motor_st khi điều khiển góc
volatile int pwm_run;       // Giá trị PWM của 4 motor
volatile int set_angle;     // Góc được chỉ định
volatile int ang;          // Góc
volatile int asen;         // Cảm biến tương tự
volatile int saka;         // Cảm biến độ dốc (tương tự)
volatile char f_saka_up;   // Cờ báo độ dốc lên 1: LÊN
volatile char f_saka_down; // Cờ báo độ dốc lên 1: XUỐNG
volatile float vbat;       // Điện áp
volatile int angle0_ad = 34500;     // Giá trị A/D của getServoAngle ở ​​0 độ
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
unsigned char getdipsw( void );
unsigned char getbutton( void );
int getServoAngle( void );
int getlineSen( void );
void motor_servo(int pwm);
void ServoControl();  // pid điều khiển đường tâm (để ổn định lại đường tâm khi vào cua)
void ServoControl2(); // pid điều khiển góc (để ổn định lại góc khi vào cua)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//all interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim -> Instance == TIM6) {
        cnt++;
        if(cnt%1000 == 0) {
            HAL_GPIO_TogglePin(GPIOE, LED_RED_Pin);
        }

		ang = getServoAngle();
        asen = getlineSen();
        dipswitch_read=getdipsw();
        button_read = getbutton();
        //
        /*
		set_angle = 0;
		ServoControl2(); // pid điều khiển góc
        motor_servo(pwm_kakudo);
        */
		//
		

        ServoControl();
        motor_servo(pwm_trace);
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART8)
    {
        //HAL_GPIO_TogglePin(GPIOE, LED_RED_Pin);
        tx_done = 1;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == BMI088_ACCEL_DR_Pin)
    {
        bmi088_accel_dr = 1;
        bmi088_accel_irq_count++;
    }
    else if(GPIO_Pin == BMI088_GYRO_DR_Pin)
    {
        bmi088_gyro_dr = 1;
        bmi088_gyro_irq_count++;
    }
    if(GPIO_Pin == BUTTON_1_Pin) {
        b1 = 1;
    }
    if(GPIO_Pin == BUTTON_3_Pin) {
        b3 = 1;
    }
}
unsigned char getdipsw( void ) {
    unsigned char sw = (
                           ((unsigned char)!HAL_GPIO_ReadPin(GPIOB, DIP1_Pin) << 0) |
                           ((unsigned char)!HAL_GPIO_ReadPin(GPIOB, DIP2_Pin) << 1) |
                           ((unsigned char)!HAL_GPIO_ReadPin(GPIOD, DIP3_Pin) << 2) |
                           ((unsigned char)!HAL_GPIO_ReadPin(GPIOD, DIP4_Pin) << 3) );
    return sw;
}
unsigned char getbutton( void ) {

    unsigned char button = (
                               ((unsigned char) b1<<0)														|
                               ((unsigned char)!HAL_GPIO_ReadPin(GPIOE, BUTTON_2_Pin) <<1)|
                               ((unsigned char)	b3<<2)						);
    return button;
}
int getServoAngle( void ) {
    return (int32_t)( (int)ad_get[2] - angle0_ad );
}
int getlineSen( void ) {
    int left_sen = (int)ad_get[1];
    int right_sen = (int)ad_get[0];
    return (int32_t)( left_sen - right_sen );
}
void motor_servo( int pwm ) {
    int i = getServoAngle();

    // Điều khiển giới hạn bên phai dựa trên giá trị âm lượng
    if ( i >=  max_right_ad_servo) {
        if ( pwm > 10 )
            pwm = 0;
    }
    // Điều khiển giới hạn bên trai dựa trên giá trị âm lượng
    if ( i <=  max_left_ad_servo ) {

        if ( pwm < -10 )
            pwm = 0;
    }

    if ( pwm > 95 )
        pwm = 95; // Kiểm tra giới hạn trên
    if ( pwm < -95 )
        pwm = -95; // Kiểm tra giới hạn dưới
	
	    if ( pwm > 0 ) {
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, ((htim4.Init.Period+1)*(100-pwm)) / 100);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, (htim4.Init.Period+1));
    } else if ( pwm < 0 ) {
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, ((htim4.Init.Period+1)*(100 + pwm)) / 100);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, (htim4.Init.Period+1));
    } else {
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, (htim4.Init.Period+1));
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, (htim4.Init.Period+1));
    }
//    if ( pwm > 0 ) {
//		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, (htim4.Init.Period+1)*pwm / 100);
//        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);
//    } else if ( pwm < 0 ) {
//        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
//		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, (htim4.Init.Period+1)*(-pwm) / 100);
//    } else {
//		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
//		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);
//    }
}
void ServoControl() {
    static int asen_before = 0; // Giá trị cảm biến analog trước đó
    int ret;

    float work_p, work_d, kp_l, kd_l;

    kp_l = 0.00015;  // Hằng số tỷ lệ
    kd_l = kp_l * 5; // Hằng số vi phân (xấp xỉ 5-10 lần P)

    // Tính toán giá trị PWM cho động cơ lái
    work_p = kp_l * asen;                   // Tỷ lệ
    work_d = kd_l * ( asen_before - asen ); // Vi phân (khoảng 5-10 lần P)
    ret = (int)( work_p - work_d );

    // Đặt giới hạn trên và dưới cho PWM
    if ( ret > 70 ) {
        ret = 70;
    }
    if ( ret < -70 ) {
        ret = -70;
    }
    pwm_trace = ret;

    asen_before = asen; // Giá trị này sẽ là giá trị từ 1ms trước đó cho lần tiếp theo
}
void ServoControl2() {
    static int angle_before;
    float work_p, work_d, kp, kd;

    kp = 0.00325; //0.00625
    kd = kp*5;// Hằng số vi phân (xấp xỉ 5-10 lần P)

    // Tính giá trị PWM cho động cơ lái
    work_p = kp * ( ang - set_angle );    // Tỷ lệ
    work_d = kd * ( angle_before - ang ); // Vi phân
    int ret = work_p - work_d;

    // Thiết lập giới hạn trên và dưới cho PWM
    if ( ret > 90 ) {
        ret = 90;
    }
    if ( ret < -90 ) {
        ret = -90;
    }
    pwm_kakudo = -ret;

    angle_before = ang; // Giá trị này sẽ là giá trị từ 1ms trước đó cho lần tiếp theo
}
//	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//			UNUSED(huart);
//			if(huart->Instance == UART8)
//			{
//	//        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
//	//        if(u8_rxdata != 13)
//	//        {
//	//            u8_rxbuff[_rxIndex++] = u8_rxdata;
//	//        }
//	//        else if (u8_rxdata == 13)
//	//        {
//	//            _rxIndex = 0;
//	//            Tx_flag = 1;
//	//        }
//	//        HAL_UART_Receive_IT(&huart8,&u8_rxdata,1);
//				HAL_UART_Transmit(&huart8, u8_rxbuff, sizeof(u8_rxbuff),100);
//			}


//	}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_TIM12_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim6);
    bool bmi088_ok = initBMI088();
    //HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ad_get, 5) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//DC LAI
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
    //HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);//
    HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);//LED
    HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);//QUAT
    HAL_TIM_Base_Start(&htim5);
    //

    int imu_len = snprintf(imu_txbuff, sizeof(imu_txbuff),
                           "BMI088 init %s, Aid=0x%02X, Gid=0x%02X\r\n",
                           bmi088_ok ? "OK" : "FAIL",
                           BMI088val.Aid,
                           BMI088val.Gid);

    HAL_UART_Transmit(&huart8, (uint8_t*)imu_txbuff, imu_len, 100);
    // HAL_UART_Transmit_DMA(&huart8, u8_txbuff, sizeof(u8_txbuff));
    //HAL_UART_Receive_IT(&huart8,&u8_rxdata,1);
    //HAL_UART_Receive_DMA(&huart8, u8_rxbuff, sizeof(u8_rxbuff));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint32_t pulse = __HAL_TIM_GET_COUNTER(&htim5);
        static uint32_t pulse_old = 0;

        if (pulse != pulse_old)
        {
            pulse_old = pulse;
            HAL_GPIO_TogglePin(GPIOE, LED_GREEN_Pin);
        }

//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, htim1.Init.Period*0.5);
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, htim1.Init.Period*0.5);
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);

//        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, htim4.Init.Period*0.5);
//        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, htim4.Init.Period);

        __HAL_TIM_SetCompare(&htim15, TIM_CHANNEL_2, htim15.Init.Period*1); //LED
        __HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, htim16.Init.Period*0.2);
        if(cnt%1000 == 0) {
            //HAL_GPIO_TogglePin(GPIOE, LED_GREEN_Pin);
            //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
        }
        if(b1)
        {
            b1 = 0;
            HAL_GPIO_TogglePin(GPIOE, LED_BLUE_Pin);
            // xử lý Button1
        }

        if(b3)
        {
            b3 = 0;
            HAL_GPIO_TogglePin(GPIOE, LED_BLUE_Pin);
            // xử lý Button3
        }
        if(button_read == 2) {
            HAL_GPIO_TogglePin(GPIOE, LED_GREEN_Pin);
        }
        //if(tx_done && (bmi088_accel_dr || bmi088_gyro_dr || (HAL_GetTick() - last_dr_report_ms >= 500U)))
        if(tx_done && (bmi088_accel_dr || bmi088_gyro_dr || (HAL_GetTick() - last_dr_report_ms >= 500U)))
        {
            uint32_t accel_irq_count;
            uint32_t gyro_irq_count;

            __disable_irq();
            bmi088_accel_dr = 0;
            bmi088_gyro_dr = 0;
            accel_irq_count = bmi088_accel_irq_count;
            gyro_irq_count = bmi088_gyro_irq_count;
            __enable_irq();

            BMI088getAccele();
            BMI088getGyro();
            BMI088getTemp();

            int32_t ax_mg = (int32_t)(BMI088val.accele.x * 1000.0f);
            int32_t ay_mg = (int32_t)(BMI088val.accele.y * 1000.0f);
            int32_t az_mg = (int32_t)(BMI088val.accele.z * 1000.0f);

            int32_t gx_mdps = (int32_t)(BMI088val.gyro.x * 1000.0f);
            int32_t gy_mdps = (int32_t)(BMI088val.gyro.y * 1000.0f);
            int32_t gz_mdps = (int32_t)(BMI088val.gyro.z * 1000.0f);

            int32_t temp_mc = (int32_t)(BMI088val.temp * 1000.0f);
            //int debug_test = snprintf(imu_txbuff, sizeof(imu_txbuff),
            //                          "test=%d test2 = %d Encoder=%lu\r\n",dipswitch_read,button_read,pulse);
            //int debug_test2 = snprintf(imu_txbuff, sizeof(imu_txbuff),
            //                         "test=%d test2 = %d Encoder=%lu\r\n",dipswitch_read,button_read,pulse);
            //
            int imu_len = snprintf(imu_txbuff, sizeof(imu_txbuff),
                                  "DR_A=%lu DR_G=%lu ACC[mg]=%ld,%ld,%ld GYRO[mdps]=%ld,%ld,%ld TEMP[mC]=%ld\r\n",
                                  (unsigned long)accel_irq_count, (unsigned long)gyro_irq_count,
                                  ax_mg, ay_mg, az_mg,
                                   gx_mdps, gy_mdps, gz_mdps,
                                   temp_mc);

            if (HAL_UART_Transmit_DMA(&huart8, (uint8_t*)imu_txbuff, imu_len) == HAL_OK)
            {
                tx_done = 0;
                last_dr_report_ms = HAL_GetTick();
            }
        }

        //HAL_Delay(1);
    /* USER CODE END WHILE */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 200;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
