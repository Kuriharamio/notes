# CAN（Controller Area Network）

## 基本介绍：

1. 多主控制：只有ID用于决定优先级，没有地址
2. 传输速率快（1Mbps）、距离远（40m）
3. 数据可靠

## 电路结构：

差分电路：

![image-20231016135000026](C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20231016135000026.png)

![image-20231016135159929](C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20231016135159929.png)

1. 显性电平对应逻辑0：CAN_H与CAN_L之差为2.5V左右
2. 隐性电平对应逻辑1：CAN_H与CAN_L之差为0V
3. 显性电平在总线上具有优先权，只要一个单元输出显性电平，总线即为显性电平
4. 在CAN总线的起止端都有一个120Ω的终端电阻，用来做阻抗匹配，以减少回波反射

## CAN协议：

![image-20231016135243296](C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20231016135243296.png)

## 数据帧：

![image-20231015235031922](C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20231015235031922.png)

### 优先级决定：

- 在总线空闲状态，最先开始发送的单元获得发送权
- 多个单元同时开始发送时，各发送单元从仲裁段的第一位开始进行仲裁，连续输出显性电平最多的单元可继续发送
- ![image-20231016135453517](C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20231016135453517.png)

### 位时序：

![image-20231016001438345](C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20231016001438345.png)

### 流程：

发送：先程序选择 1 个空置的邮箱（TME=1）→设置标识符（ID），数据长度和 发送数据→设置 CAN_TIxR 的 TXRQ 位为 1，请求发送→邮箱挂号（等待成为最高优先级）→ 预定发送（等待总线空闲）→发送→邮箱空置

接收：FIFO 空→收到有效报文→挂号_1（存入 FIFO 的一个邮箱，这个由硬件 控制，我们不需要理会）→收到有效报文→挂号_2→收到有效报文→挂号_3→收到有效报文溢 出。

![image-20231016135807188](C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20231016135807188.png)



CubeMX配置：

<img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20231016145207996.png" alt="image-20231016145207996" style="zoom: 50%;" /><img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20231016145218970.png" alt="image-20231016145218970" style="zoom: 50%;" />

### 滤波器配置：

```c
	 /* USER CODE BEGIN CAN_Init 2 */
     CAN_FilterTypeDef sFilterConfig;/* 配置 CAN 过滤器*/
     sFilterConfig.FilterBank = 0; /* 过滤器 0 */
     sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
     sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
     sFilterConfig.FilterIdHigh = 0x0000; /* 32 位 ID */
     sFilterConfig.FilterIdLow = 0x0000;
     sFilterConfig.FilterMaskIdHigh = 0x0000; /* 32 位 MASK */
     sFilterConfig.FilterMaskIdLow = 0x0000;
     sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; /* 过滤器 0 关联到 FIFO0 */
     sFilterConfig.FilterActivation = ENABLE; /* 激活滤波器 0 */
     sFilterConfig.SlaveStartFilterBank = 14;
     HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
     HAL_CAN_Start(&hcan);
	/* USER CODE END CAN_Init 2 */
```

### 收发函数：

```c
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef g_canx_txheader;    /* 发送参数句柄 */
CAN_RxHeaderTypeDef g_canx_rxheader;    /* 接收参数句柄 */
/* USER CODE END 0 */
```

```c
/**
* @brief CAN 发送一组数据
* @note 发送格式固定为: 标准 ID, 数据帧
* @param id : 标准 ID(11 位)
* @retval 发送状态 0, 成功; 1, 失败;
*/
uint8_t can_send_msg(uint32_t id, uint8_t *msg, uint8_t len)
{
	uint32_t TxMailbox = CAN_TX_MAILBOX0;
	g_canx_txheader.StdId = id; /* 标准标识符 */
	g_canx_txheader.ExtId = id; /* 扩展标识符(29 位) */
	g_canx_txheader.IDE = CAN_ID_STD; /* 使用标准帧 */
	g_canx_txheader.RTR = CAN_RTR_DATA; /* 数据帧 */
	g_canx_txheader.DLC = len;
	if (HAL_CAN_AddTxMessage(&hcan, &g_canx_txheader,msg, &TxMailbox) != HAL_OK) 		/* 发送消息 */
	{
		return 1;
	}
	/* 等待发送完成,所有邮箱为空(3 个邮箱) */
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3);
	return 0;
}

/**
* @brief CAN 接收数据查询
* @note 接收数据格式固定为: 标准 ID, 数据帧
* @param id : 要查询的 标准 ID(11 位)
* @param buf : 数据缓存区
* @retval 接收结果
* @arg 0 , 无数据被接收到;
* @arg 其他, 接收的数据长度
*/
uint8_t can_receive_msg(uint32_t id, uint8_t *buf)
{
	if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) != 1)
	{
		return 0;
	}
	if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &g_canx_rxheader,buf) != HAL_OK)
	{
		return 0;
	}
	/* 接收到的 ID 不对 / 不是标准帧 / 不是数据帧 */
	if (g_canx_rxheader.StdId!= id || g_canx_rxheader.IDE != CAN_ID_STD || g_canx_rxheader.RTR != CAN_RTR_DATA)
	{
		return 0; 
	}
	return g_canx_rxheader.DLC;
}
```





```c
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

// 标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE   (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

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

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief 配置CAN的滤波器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号 | FIFOx | ID类型 | 帧类型
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x3ff, 0x1fffffff)
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure;

    // 检测关键传参
    assert_param(hcan != NULL);

    if ((Object_Para & 0x02)) {
        // 标准帧
        // 掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 3 >> 16;
        // 掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
    } else {
        // 扩展帧
        // 掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 5;
        // 掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
    }

    // 滤波器序号, 0-27, 共28个滤波器, can1是0~13, can2是14~27
    can_filter_init_structure.FilterBank = Object_Para >> 3;
    // 滤波器绑定FIFOx, 只能绑定一个
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    // 使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;
    // 滤波器模式, 设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32位滤波
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    // 从机模式选择开始单元
    can_filter_init_structure.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    // 检测关键传参
    assert_param(hcan != NULL);

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE   = 0;
    tx_header.RTR   = 0;
    tx_header.DLC   = Length;

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}

/**
 * @brief 点灯
 *
 * @param data 收到的数据
 */
void LED_Control(uint8_t data)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, ((data & 1) == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, ((data & 2) == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, ((data & 4) == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, ((data & 8) == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, ((data & 16) == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, ((data & 32) == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, ((data & 64) == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, ((data & 128) == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data;

    HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &header, &data);

    LED_Control(data);
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
    MX_CAN1_Init();
    /* USER CODE BEGIN 2 */

    uint8_t Send_Data = 0;

    CAN_Init(&hcan1);
    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(13) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0x114, 0x7ff);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        Send_Data++;
        CAN_Send_Data(&hcan1, 0x114, &Send_Data, 1);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        HAL_Delay(250);
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
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 6;
    RCC_OscInitStruct.PLL.PLLN       = 180;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Activate the Over-Drive mode
     */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
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
    while (1) {
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

```

