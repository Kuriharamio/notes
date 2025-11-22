# USART

## 通用同步/异步串行接收/发送器 (Universal Synchronous/Asynchronous Receiver/Transmitter)：

- 异步通信（Asynchronous）：TX、RX	               （通常使用）

- 同步通信（Synchronous)：TX、RX、CLK 			（数据量大时使用，效率高）

  ​	*TX：发送数据输出引脚	RX：接受数据输入引脚*

- 不同设备之间的通信还需要有GND线，VCC为可选项

- USART 是异步串口通信**协议**的一种，能进行**全双工**数据交换

- 波特率

  ------

  ## 时序图：

<img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929155013468.png" alt="image-20230929155013468" style="zoom:67%;" />

- **启动位**

  - 无数据发送时为逻辑“1”状态；发出一个逻辑“0”信号时表示开始传输字符

- **数据位**

  - 一般发送8个数据位，也就是一帧数据有8字节，构成一个字符
  - 从最低位开始传送，靠时钟定位	**例如：**0x11 （0001 0001）则发送10001000

  - 1为高电平，0为低电平

- **奇偶校验位**（可有可无）

  - 数据位加上这一位后，使得“1”的位数应为偶数（偶校验）或奇数（奇校验），以此来检验资料传送的正确性

- **停止位**

  - 一个字符数据的结束标志；可以是1位、1.5位、2位的高电平
  - 不仅仅代表传输结束，并且提供计算机校正时钟同步的机会

  ------

## FIFO（First-In First-Out) ：

- ### 发送FIFO：

  - 相当于一个缓冲区，降低cpu多次中断的负担

  - 只要有数据填充到发送FIFO里，就会立即启动发送过程，在发送的同时，其它需要发送的数据还可以继续填充到发送FIFO里，但已填满时不能继续填充，防止数据丢失

- ### 接收FIFO：

  - 当硬件逻辑接收到数据时，就会往接收FIFO里填充数据，程序应及时取走这些数据，防止数据丢失

*收发FIFO主要是为了解决UART收发中断过于频繁而导致CPU效率不高的问题而引入的，可以不用每收发1个数据就中断处理一次，而是接收多个数据后才中断处理一次*

------

**使用DMA：**无须CPU干预，节省CPU资源来做其它操作

**接收数据就绪可读：**是一个中断事件，事件标志为TXNE

------

**IDLE：监测到总线空闲（IDLE line detected）**

- 当监测到总线空闲时，该位被设为“1”，
- 如果不知道数据有多少字节，需要用空闲中断：若USART_CR1中的IDLEIE为1，则产生中断

------

## 函数：

- ### 接收函数

```c
/*	UART_HandleTypeDef *huart 指定串口 e.g. &huart1	*/
/*	uint8_t *pData  指定存储位置，提前定一个数组用来存储数据 */
/*	uint16_t Size	数据长度 */
/*	uint32_t Timeout 超时时间，如果到该时间还未接收到数据则接收失败 HAL_MAX_DELAY为最大超时时间*/
HAL_UART_Receive(UART_HandleTypeDef *huart , uint8_t *pData , uint16_t Size, uint32_t Timeout);
```

```c
/*  有数据来的时候就进入中断，而不需要等待，故不需要超时时间参数 */
/* 	使能串口接收中断 进入HAL_UART_RxCpltCallback	*/
HAL_UART_Receive_IT(UART_HandleTypeDef *huart , uint8_t *pData , uint16_t Size);
```

```c
HAL_UART_Receive_DMA(UART_HandleTypeDef *huart , uint8_t *pData , uint16_t Size);
```

- ### 发送函数

```c
HAL_UART_Transmit(UART_HandleTypeDef *huart , uint8_t *pData , uint16_t Size, uint32_t Timeout);
```

```c
HAL_UART_Transmit_IT(UART_HandleTypeDef *huart , uint8_t *pData , uint16_t Size);
```

```c
HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart , uint8_t *pData , uint16_t Size);
```

- ### 中断函数

```c
void USART1_IRQHandler(void)
{
	/*	USER CODE BEGIN USART1_IRQn 0 */
	
	/*	USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/*	USER CODE BEGIN USART1_IRQn 1 */
	
	/*	USER CODE END USART1_IRQn 1 */
	
}
```

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hurat) //Rx Transfer completed callbacks
{
    /*	需要再一次使能中断	*/
	HAL_UART_Receive_IT(UART_HandleTypeDef *huart , uint8_t *pData , uint16_t Size);
    /*	功能	*/
}
```

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hurat)
{
     /*	需要再一次使能中断	*/
	HAL_UART_Transmit_IT(UART_HandleTypeDef *huart , uint8_t *pData , uint16_t Size);
    /*	功能	*/
}
```

- ### DMA

  ```c
  extern UART_HandleTypeDef huart4;
  extern DMA_HandleTypeDef hdma_uart4_rx;
  
  //在主函数运行以下两句
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, Lidar_rx, 58); //开启
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx,DMA_IT_HT); //关闭过半中断
  
  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
  {
  	if(huart == &huart4) {
          
          //process
          
          //再次开启
  		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, Lidar_rx, 58);
          __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
      }	
  }
  ```
  
  
  
- ### 空闲中断：

  ​																		*一定要使用DMA*

  <img src="C:\Users\13618\AppData\Roaming\Typora\typora-user-images\image-20230929173406213.png" alt="image-20230929173406213" style="zoom:25%;" />

```c
/* USER CODE BEGIN 0 */
uint8_t receive_data[500]			//先定义一个足够大的数组存储数据 
/* USER CODE END 0 */
    
/* USER CODE BEGIN 2 */
__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);		//打开串口空闲中断
HAL_UART_Receive_DMA(&huart1, receive_data, 100);	//通过DMA接收数据
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))			//判断一帧数据是否接收完毕
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);					//清空IDLE标志位
        (void)USART1 ->SR;			//清空SR寄存器
        (void)USART1 ->DR;			//清空DR寄存器
        __HAL_DMA_CLEAR_FLAG(huart, DMA_FLAG_TC5);			//清空DMA传输完成标志位
        HAL_UART_DMAStop(huart);
        //接收到的字节数
        int len = 100 - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        HAL_UART_Transmit(&huart1, receive_data, len, 10);
        HAL_UART_Receive_DMA(huart,receive_data,100u);		//再次使能接收，NDTR重载
    }
}
/* USER CODE END 4 */

/* 在 stm32f1xx_it.c 中 */

/* USER CODE BEGIN PFP */
void UART_IDLE_Callback(UART_HandleTypeDef *huart);    //声明函数原型
/* USER CODE BEGIN PFP */

void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */
    UART_IDLE_Callback(&huart1);
    return;  //屏蔽下面那个自带的中断，会报warning，不用管
    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */
    
    /* USER CODE END USART1_IRQn 1 */
    
}
```



- ### printf重定向：

  #### include “ retarget.h”

```c
#ifndef	__RETARGET_H
#define	__RETARGET_H

#include "stm32f1xx.h"

#include "usart.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define _USART1 1	                                //??1??
#define _USART2 0	                                //??2??
#define _USART3 0	                                //??3??

#define BUFFER_MAX_SIZE         512
static char USART_BUFFER[BUFFER_MAX_SIZE];

#if _USART1
    #define printf(FORMAT,...) \
    {\
        memset(USART_BUFFER, 0, BUFFER_MAX_SIZE);\
        sprintf(USART_BUFFER,FORMAT,##__VA_ARGS__); \
	    HAL_UART_Transmit(&huart1,(uint8_t *)USART_BUFFER,strlen(USART_BUFFER), 1);\
    }
#else
    #define printf(FORMAT,...)
#endif

#if _USART2
    #define printf2(FORMAT,...) \
    {\
        memset(USART_BUFFER, 0, BUFFER_MAX_SIZE);\
        sprintf(USART_BUFFER,FORMAT,##__VA_ARGS__); \
	    HAL_UART_Transmit(&huart2,(uint8_t *)USART_BUFFER,strlen(USART_BUFFER), 1);\
    }
#else
    #define printf2(FORMAT,...)
#endif

#if _USART3
    #define printf3(FORMAT,...) \
    {\
    memset(USART_BUFFER, 0, BUFFER_MAX_SIZE);\
    sprintf(USART_BUFFER,FORMAT,##__VA_ARGS__); \
	HAL_UART_Transmit(&huart3,(uint8_t *)USART_BUFFER,strlen(USART_BUFFER), 1);\
    }
#else
    #define printf3(FORMAT,...)
#endif

#endif
```







