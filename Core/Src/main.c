/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_log.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "bsp_can.h"

//#include "dm_imu.h"
#include "robot.h"
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
#define W25Qxx_NumByteToTest   	32*1024					// �������ݵĳ��ȣ�32K

int32_t OSPI_Status ; 		 //����־λ

uint8_t  W25Qxx_WriteBuffer[W25Qxx_NumByteToTest];		//	д��������
uint8_t  W25Qxx_ReadBuffer[W25Qxx_NumByteToTest];		//	����������

// 注册CAN实例
CANInstance* can3_instance;
CANInstance* can1_instance;
CANInstance* can1_instance2;
CANInstance* can1_instance3;
CANInstance* can2_instance;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**


void can3_receive_callback(CANInstance* instance) {
	IMU_UpdateData(instance->rx_buff);
}
void can1_receive_callback(CANInstance* instance) {
	  dm_motor_fbdata(&dmmotor_yaw1->measure, instance->rx_buff);
}
void can1_receive2_callback(CANInstance* instance) {
	  dm_motor_fbdata(&dmmotor_pitch->measure, instance->rx_buff);
}

void can1_receive3_callback(CANInstance* instance) {
	  dm_motor_fbdata(&dmmotor_loader->measure, instance->rx_buff);
}
void can2_receive_callback(CANInstance* instance) {
	  dm_motor_fbdata(&dmmotor_yaw2->measure, instance->rx_buff);
}
void register_can3_receiver(uint32_t rx_id) {//达妙imu数据读取
    // 定义CAN初始化配置结构体
    CAN_Init_Config_s can3_config = {
        .can_handle = &hfdcan3,                 // 使用CAN3句柄
        .tx_id = 0x01,                         // 发送ID（根据需要设置）
        .rx_id = rx_id,                         // 接收ID
        .can_module_callback = can3_receive_callback, // 接收回调函数
        .id = NULL                              // 可选的模块标识符
    };
    
    // 注册CAN实例
    CANInstance* can3_instance = CANRegister(&can3_config);
    
    if (can3_instance != NULL) {
        printf("CAN3 receiver registered successfully\n");
    } else {
        printf("Failed to register CAN3 receiver\n");
    }
}
void register_can1_receiver(uint32_t rx_id) {//yaw1的电机数据读取
    // 定义CAN初始化配置结构体
    CAN_Init_Config_s can1_config = {
        .can_handle = &hfdcan1,                 // 使用CAN3句柄
        .tx_id = 0x17,                         // 发送ID（根据需要设置）
        .rx_id = rx_id,                         // 接收ID
        .can_module_callback = can1_receive_callback, // 接收回调函数
        .id = NULL                              // 可选的模块标识符
    };
    
    // 注册CAN实例
    CANInstance* can1_instance = CANRegister(&can1_config);

    if (can1_instance != NULL) {
        printf("CAN3 receiver registered successfully\n");
    } else {
        printf("Failed to register CAN3 receiver\n");
    }
}
void register_can2_receiver(uint32_t rx_id) {//yaw2的电机数据读取
    // 定义CAN初始化配置结构体
    CAN_Init_Config_s can2_config = {
        .can_handle = &hfdcan2,                 // 使用CAN3句柄
        .tx_id = 0x18,                         // 发送ID（根据需要设置）
        .rx_id = rx_id,                         // 接收ID
        .can_module_callback = can2_receive_callback, // 接收回调函数
        .id = NULL                              // 可选的模块标识符
    };
    // 注册CAN实例
    CANInstance* can2_instance = CANRegister(&can2_config);
    if (can2_instance != NULL) {
        printf("CAN3 receiver registered successfully\n");
    } else {
        printf("Failed to register CAN3 receiver\n");
    }
}

void register2_can1_receiver(uint32_t rx_id) {//pitch的电机数据读取
    // 定义CAN初始化配置结构体
    CAN_Init_Config_s can1_config2 = {
        .can_handle = &hfdcan1,                 // 使用CAN3句柄
        .tx_id = 0x16,                         // 发送ID（根据需要设置）
        .rx_id = rx_id,                         // 接收ID
        .can_module_callback = can1_receive2_callback, // 接收回调函数
        .id = NULL                              // 可选的模块标识符
    };
    // 注册CAN实例
    CANInstance* can1_instance2 = CANRegister(&can1_config2);
    if (can1_instance2 != NULL) {
        printf("CAN3 receiver registered successfully\n");
    } else {
        printf("Failed to register CAN3 receiver\n");
    }
}

void register3_can1_receiver(uint32_t rx_id) {//拨弹盘的电机数据读取
    // 定义CAN初始化配置结构体
    CAN_Init_Config_s can1_config2 = {
        .can_handle = &hfdcan1,                 // 使用CAN3句柄
        .tx_id = 0x15,                         // 发送ID（根据需要设置）
        .rx_id = rx_id,                         // 接收ID
        .can_module_callback = can1_receive3_callback, // 接收回调函数
        .id = NULL                              // 可选的模块标识符
    };
    
    // 注册CAN实例
    CANInstance* can1_instance3 = CANRegister(&can1_config2);

    if (can1_instance3 != NULL) {
        printf("CAN3 receiver registered successfully\n");
    } else {
        printf("Failed to register CAN3 receiver\n");
    }
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
  MX_ADC1_Init();
  MX_TIM12_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART7_Init();
  MX_USART10_UART_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_UART8_Init();
  MX_UART9_Init();
  MX_I2C2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  // 初始化CAN3模块
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
  // register_can3_receiver(0x11);
  // register_can1_receiver(0x17);
  // register_can2_receiver(0x18);
  // register2_can1_receiver(0x16);
  // register3_can1_receiver(0x15);

  RobotInit(); 

  LOGINFO("[main] SystemInit() and RobotInit() done");

  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  vofa_start();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM23 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM23)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
