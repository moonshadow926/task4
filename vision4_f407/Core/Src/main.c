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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * 视觉任务四 STM32 端（正点原子探索者 STM32F407ZGT6，HAL 库）
 * USART1 (PA9/PA10) -> 板载 CH340 -> USB_232 口，115200 8N1，中文按 UTF-8 发送
 *
 * KEY0  -> "我爱视觉"   基础部分（VOFA+ 用 RawData 协议、UTF-8 编码查看）
 * KEY1  -> "#CAP"      PC 拍一张照片并识别
 * KEY2  -> "#ENROLL"   PC 现场录入人脸特征
 * WK_UP -> "#CAP3"     PC 连拍 3 张做对照
 * PC 回传 "R:<name>,<MASK|NOMASK>,<sim>\r\n"：LED1 亮 = 戴口罩，LED1 灭 = 未戴口罩/无人脸
 */
#define RX_BUF_SIZE 128

#define KEY_NONE   0
#define KEY0_PRES  1
#define KEY1_PRES  2
#define KEY2_PRES  3
#define WKUP_PRES  4

#define LED_ON(n)     HAL_GPIO_WritePin(n##_GPIO_Port, n##_Pin, GPIO_PIN_RESET) /* 低电平点亮 */
#define LED_OFF(n)    HAL_GPIO_WritePin(n##_GPIO_Port, n##_Pin, GPIO_PIN_SET)
#define LED_TOGGLE(n) HAL_GPIO_TogglePin(n##_GPIO_Port, n##_Pin)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t rx_dma_buf[RX_BUF_SIZE];          /* HAL 接收缓冲 */
static char rx_line[RX_BUF_SIZE];                /* 一条完整的 PC 回传 */
static uint16_t rx_line_len = 0;
static volatile uint8_t rx_line_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t Key_Scan(void);
static void UART_StartRx(void);
static void Handle_PcReply(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* printf 重定向到 USART1（覆盖 syscalls.c 中的弱定义 _write） */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

/* 按键扫描：不支持连按，松开后才能再次触发。KEY0~2 低电平有效，WK_UP 高电平有效 */
static uint8_t Key_Scan(void)
{
  static uint8_t released = 1;
  uint8_t k0 = HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET;
  uint8_t k1 = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET;
  uint8_t k2 = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET;
  uint8_t wk = HAL_GPIO_ReadPin(WK_UP_GPIO_Port, WK_UP_Pin) == GPIO_PIN_SET;

  if (released && (k0 || k1 || k2 || wk))
  {
    HAL_Delay(10);                                  /* 消抖 */
    released = 0;
    if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET) return KEY0_PRES;
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) return KEY1_PRES;
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) return KEY2_PRES;
    if (HAL_GPIO_ReadPin(WK_UP_GPIO_Port, WK_UP_Pin) == GPIO_PIN_SET) return WKUP_PRES;
    released = 1;                                   /* 抖动误触发，丢弃 */
  }
  else if (!k0 && !k1 && !k2 && !wk)
  {
    released = 1;
  }
  return KEY_NONE;
}

/* 开启“接收到空闲”中断：PC 一次发完一行，总线空闲时进回调 */
static void UART_StartRx(void)
{
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_dma_buf, RX_BUF_SIZE);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    if (!rx_line_ready)                             /* 上一行还没处理就丢弃新数据 */
    {
      uint16_t n = Size < RX_BUF_SIZE - 1 ? Size : RX_BUF_SIZE - 1;
      memcpy(rx_line, rx_dma_buf, n);
      rx_line[n] = '\0';
      rx_line_len = n;
      rx_line_ready = 1;
    }
    UART_StartRx();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    UART_StartRx();                                 /* 溢出/帧错误后重新开始接收 */
  }
}

/* 解析 PC 回传结果，例如 "R:ZhangSan,NOMASK,0.72" / "R:-,MASK,0.00" / "R:NOFACE" */
static void Handle_PcReply(void)
{
  while (rx_line_len && (rx_line[rx_line_len - 1] == '\r' || rx_line[rx_line_len - 1] == '\n'))
  {
    rx_line[--rx_line_len] = '\0';
  }
  if (strncmp(rx_line, "R:", 2) != 0)
  {
    return;
  }
  if (strstr(rx_line, ",MASK"))
  {
    LED_ON(LED1);
  }
  else
  {
    LED_OFF(LED1);
  }
  printf("STM32 收到识别结果: %s\r\n", rx_line + 2);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LED_OFF(LED0);
  LED_OFF(LED1);
  UART_StartRx();
  printf("STM32 就绪: KEY0=我爱视觉 KEY1=拍照 KEY2=录入 WK_UP=连拍3张\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch (Key_Scan())
    {
      case KEY0_PRES:
        printf("我爱视觉\r\n");
        LED_TOGGLE(LED0);
        break;
      case KEY1_PRES:
        printf("#CAP\r\n");
        LED_TOGGLE(LED0);
        break;
      case KEY2_PRES:
        printf("#ENROLL\r\n");
        LED_TOGGLE(LED0);
        break;
      case WKUP_PRES:
        printf("#CAP3\r\n");
        LED_TOGGLE(LED0);
        break;
      default:
        break;
    }

    if (rx_line_ready)
    {
      Handle_PcReply();
      rx_line_ready = 0;
    }

    /* 若接收在发送期间因忙而未能重新启动，这里补救 */
    if (huart1.RxState == HAL_UART_STATE_READY)
    {
      UART_StartRx();
    }

    HAL_Delay(5);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
