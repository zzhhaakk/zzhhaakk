/*
     串口 高级 编程思想
 .h  文件
 */



#ifndef __PLATFORM_USART_H__
#define __PLATFORM_USART_H__
 
#include "stdio.h"

/**
 @file fifo.c
 @brief 
 @author ZHK
 @version 0.1
 @date 2022/04/26
 @copyright Copyright (c) 2022  Shenzhen Nephotonics Inc
 */
 
#include <stdint.h>
 
 /**
  @brief 
  */
typedef struct {
  uint32_t usart_periph;

  uint8_t *tx_buf;
  uint16_t tx_buf_size;
  uint16_t tx_rd;
  uint16_t tx_wr;

  uint8_t *rx_buf;
  uint16_t rx_buf_size;
  uint16_t rx_rd;
  uint16_t rx_wr;

  void (*config)(void);
  void (*deconfig)(void);
} usart_context_t;  // 串口 句柄 对象
 
 
void usart_config_init(usart_context_t *pusart_context, uint32_t baud_rate);
void usart_config_deinit(usart_context_t *pusart_context);
void usart_send_it(usart_context_t *pusart_context, const void *_send_buf, const uint16_t send_count);
void usart_wait_sned_finished(usart_context_t *pusart_context);
void usart_it(usart_context_t *pusart_context);
int usart_receive_read(usart_context_t *pusart_context, void *_receive_buf, const int receive_count);
void usart_printf(usart_context_t *pusart_context, char *arg, ...);
void u_tm_log(char *arg, ...);
void u_log(char *arg, ...);
extern usart_context_t usart0_context;
extern usart_context_t usart1_context;
extern usart_context_t usart3_context;
extern usart_context_t usart4_context;
#endif








/*
    .c 文件
   
 * Copyright (C) 2017, 2020  huohongpeng
 * Author: huohongpeng <1045338804@qq.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Change logs:
 * Date        Author        Notes
 * 2017-02-29  huohongpeng   First add
 * 2020-09-19  huohongpeng   add u_tm_log()
 */
#include <gd32f30x.h>
#include "platform_usart.h"
 
 
static uint8_t usart0_tx_buf[1024];
static uint8_t usart0_rx_buf[16];
 
static uint8_t usart1_tx_buf[128];
static uint8_t usart1_rx_buf[128];
 
static uint8_t usart3_tx_buf[128];
static uint8_t usart3_rx_buf[128];
 
static uint8_t usart4_tx_buf[128];
static uint8_t usart4_rx_buf[128];
 
 
static void usart0_config(void)
{
  nvic_irq_enable(USART0_IRQn, 0, 1); // 串口0中断
  rcu_periph_clock_enable(RCU_GPIOA); // A 时钟 
  rcu_periph_clock_enable(RCU_USART0); // 串口0 时钟
 
  /* connect port to USARTx_Tx */
  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
  // IO 初始化
  /* connect port to USARTx_Rx */
  gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
}
 
static void usart1_config(void)
{
  nvic_irq_enable(USART1_IRQn, 1, 1);  // 串口1 中断
  rcu_periph_clock_enable(RCU_GPIOD);  // D 时钟
  rcu_periph_clock_enable(RCU_USART1); // 串口1 时钟
  
  rcu_periph_clock_enable(RCU_AF);    // 串口1 时钟
  gpio_pin_remap_config(GPIO_USART1_REMAP, ENABLE); // 串口1 时钟
  
  
  /* connect port to USARTx_Tx */
  gpio_init(GPIOD, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
 // IO 初始化
  /* connect port to USARTx_Rx */
  gpio_init(GPIOD, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
}
 
static void usart3_config(void)   // 中断串口3 IO时钟 io
{
  nvic_irq_enable(UART3_IRQn, 1, 1);
  rcu_periph_clock_enable(RCU_GPIOC);
  rcu_periph_clock_enable(RCU_UART3);
 
  /* connect port to USARTx_Tx */
  gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
 
  /* connect port to USARTx_Rx */
  gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
}
 
static void usart4_config(void)  // 中断 串口4io 时钟
{
  nvic_irq_enable(UART4_IRQn, 1, 1);
  rcu_periph_clock_enable(RCU_GPIOC);
  rcu_periph_clock_enable(RCU_GPIOD);
  rcu_periph_clock_enable(RCU_UART4);
 
  /* connect port to USARTx_Tx */
  gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
 
  /* connect port to USARTx_Rx */
  gpio_init(GPIOD, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
}
 
static void usart0_deconfig(void)  // 串口0复位
{
  gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
  nvic_irq_disable(USART0_IRQn);   // 关中断
  rcu_periph_clock_disable(RCU_GPIOA); // 关闭 IO 时钟（降智操作）
  rcu_periph_clock_disable(RCU_USART0);// 关闭串口时钟
}
 
 
static void usart1_deconfig(void)  // 串口1复位
{
}
 
 
static void usart3_deconfig(void)  // 串口2复位
{
}
 
static void usart4_deconfig(void)  // 串口3复位
{ 
}
 
 
usart_context_t usart0_context = {
  USART0,    // 串口句柄
  usart0_tx_buf,  // 发送缓冲区
  sizeof(usart0_tx_buf), // 发送缓冲区大小 （字节数）
  0,
  0,
  usart0_rx_buf, // 接收缓冲区
  sizeof(usart0_rx_buf), // 接收缓冲区大小 （字节数）
  0,
  0,
  usart0_config,   // 函数指针 -> 指向初始化这个串口的函数
  usart0_deconfig  // 函数指针 —> 指向复位这个串口的函数
};
 
 
usart_context_t usart1_context = {
  USART1,
  usart1_tx_buf,
  sizeof(usart1_tx_buf),
  0,
  0,
  usart1_rx_buf,
  sizeof(usart1_rx_buf),
  0,
  0,
  usart1_config,
  usart1_deconfig,
};
 
usart_context_t usart3_context = {
  UART3,
  usart3_tx_buf,
  sizeof(usart3_tx_buf),
  0,
  0,
  usart3_rx_buf,
  sizeof(usart3_rx_buf),
  0,
  0,
  usart3_config,
  usart3_deconfig,
};
 
usart_context_t usart4_context = {
  UART4,
  usart4_tx_buf,
  sizeof(usart4_tx_buf),
  0,
  0,
  usart4_rx_buf,
  sizeof(usart4_rx_buf),
  0,
  0,
  usart4_config,
  usart4_deconfig,
};
 
 /**
  @brief   
  @param   pusart_context 
  @param   baud_rate 
  */
 
void usart_config_init(usart_context_t *pusart_context, uint32_t baud_rate)
{
  pusart_context->config();
  
  /* USART configure */
  usart_deinit(pusart_context->usart_periph); // 复位 串口x  
  usart_baudrate_set(pusart_context->usart_periph, baud_rate);// 设置 波特率
  usart_receive_config(pusart_context->usart_periph, USART_RECEIVE_ENABLE); // 使用接收功能
  usart_transmit_config(pusart_context->usart_periph, USART_TRANSMIT_ENABLE);// 使用发送功能
  usart_word_length_set(pusart_context->usart_periph, USART_WL_8BIT); // 设置字节长度
  usart_stop_bit_set(pusart_context->usart_periph, USART_STB_1BIT);  // 停止位 长度
  usart_enable(pusart_context->usart_periph); // 使能串口x
  
  usart_interrupt_enable(pusart_context->usart_periph, USART_INT_RBNE); // 开启串口中断
}
 
void usart_config_deinit(usart_context_t *pusart_context)
{
  usart_disable(pusart_context->usart_periph); //关闭串口
  usart_deinit(pusart_context->usart_periph);  // 复位串口
  pusart_context->deconfig();  //调用串口关闭函数
}
 
/*
 * Rewrite fputc for printf 
 * NOTE: IAR options->C/C++ compiler->preprocessor add symbal _DLIB_FILE_DESCRIPTOR
 */
int fputc(int ch, FILE  *f)  // 重写printf相关函数
{
    usart_send_it(&usart0_context, &ch, 1);
    return ch;
}

/*
参数1 ：串口句柄
参数2 ：发送的内容 字符串
参数3 ：发送的长度
*/
void usart_send(usart_context_t *pusart_context, const void *_send_buf, const uint16_t send_count)
{
  const uint8_t *send_buf = (const uint8_t *)_send_buf;
  uint16_t i;
  for(i = 0; i < send_count; i++) {
    while(RESET == usart_flag_get(pusart_context->usart_periph, USART_FLAG_TBE)); // 当前串口判忙->等待
    usart_data_transmit(pusart_context->usart_periph, (uint8_t)send_buf[i]);      // 发送 1字节 数据
  }
  while(RESET == usart_flag_get(pusart_context->usart_periph, USART_FLAG_TC));    // 等待发送完成
}
/*
 * Usart send base on usart send interrupt 
 */
void usart_send_it(usart_context_t *pusart_context, const void *_send_buf, const uint16_t send_count)
{
  const uint8_t *send_buf = (const uint8_t *)_send_buf;
  uint16_t i;
  /*
   * Write send data to send buffer and use interrupt send data.
   * Wait buffer effective when send buffer is full. 
   * 发送缓冲区满时，等待缓冲区生效
   */
  for(i = 0; i < send_count; i++) {      
    while((pusart_context->tx_wr+1) % pusart_context->tx_buf_size == pusart_context->tx_rd);
    // 等待 
    pusart_context->tx_buf[pusart_context->tx_wr++] = send_buf[i];     
    pusart_context->tx_wr = pusart_context->tx_wr % pusart_context->tx_buf_size; 
    // tx_buf 写入计数器 tx_wr
    usart_interrupt_enable(pusart_context->usart_periph, USART_INT_TBE);     
    }
}
 
void usart_wait_sned_finished(usart_context_t *pusart_context)
{
  while(pusart_context->tx_wr != pusart_context->tx_rd);
  while(RESET == usart_flag_get(pusart_context->usart_periph, USART_FLAG_TC));
  // 获取 USART_FLAG_TC标志位  等待发送完成中断
}
/*
 * read data from receive buffer
 */

/**
 @brief 
 @param   pusart_context   
 @param   _receive_buf     
 @param   receive_count    
 @return int 
 */
int usart_receive_read(usart_context_t *pusart_context, void *_receive_buf, const int receive_count)
// 从接收缓冲区中读数据 （接收缓冲 由 uart_receive 存放数据）
{
  uint8_t *receive_buf = (uint8_t *)_receive_buf;
  int i, receive_count_real;
  /*
   * Read data from receive buffer. 
   * The buffer have data that received from usart.
   */

  // rx_rd (read rx_buff -> use software)
  // rx_wr (write data to rx_buf -> use hardware )
 
  for(i = 0, receive_count_real = 0; i < receive_count; i++) {
    if(pusart_context->rx_rd == pusart_context->rx_wr) {
      return receive_count_real;
      } else {
      receive_buf[i] = pusart_context->rx_buf[pusart_context->rx_rd++];
      pusart_context->rx_rd %= pusart_context->rx_buf_size;
      receive_count_real++;
      }
    }
    return receive_count_real;
}
 
// RBNE  读数据缓冲区非空中断和标志
/**
 @brief 
 @param   pusart_context  UART handle
 */
static void usart_rbne_it(usart_context_t *pusart_context)
{
  pusart_context->rx_buf[ pusart_context->rx_wr++ ] = usart_data_receive(pusart_context->usart_periph);
  pusart_context->rx_wr %= pusart_context->rx_buf_size;
  // 如果rx_wr != rx_buf_size  ，则 保存 rx_wr 

  /*
   * overflow handle  溢出处理
   */
  if(pusart_context->rx_wr == pusart_context->rx_rd) {
    // 如果 
    pusart_context->rx_rd++;
    pusart_context->rx_rd = pusart_context->rx_rd % pusart_context->rx_buf_size ;
  }
}


   // 如果 tx_rd ！= tx_wr
   // 已经 tx_buf 发走的数据 != tx_buf 存的数据数
   // tx_rd : tx_buf receive data from mcu
   // tx_wr : tx_buf transmited counter by uart  
   
// TBE 中断 发送缓冲区 空 中断
static void usart_tbe_it(usart_context_t *pusart_context)
{
  if(pusart_context->tx_rd != pusart_context->tx_wr) {
 

    usart_data_transmit(pusart_context->usart_periph, pusart_context->tx_buf[pusart_context->tx_rd++]);

    pusart_context->tx_rd %= pusart_context->tx_buf_size;
  } else {
    usart_interrupt_disable(pusart_context->usart_periph, USART_INT_TBE);
  }
}
 
 // 串口中断
 /**
  @brief 
  @param   pusart_context 
  */
void usart_it(usart_context_t *pusart_context)
{
  if(usart_interrupt_flag_get(pusart_context->usart_periph, USART_INT_FLAG_RBNE) == SET) {  
    // 收到 RBEN 中断标志位
    usart_interrupt_flag_clear(pusart_context->usart_periph, USART_INT_FLAG_RBNE);
    // 清 中断标志位
    usart_rbne_it(pusart_context); 
    // 执行 中断读数据缓冲区非空中断和过载错误中断 
  }
 
  if(usart_interrupt_flag_get(pusart_context->usart_periph, USART_INT_FLAG_TBE) == SET) {
    // 收到 TBE 中断标志 
    usart_interrupt_flag_clear(pusart_context->usart_periph, USART_INT_FLAG_TBE);// 清中断标志位
    usart_tbe_it(pusart_context); // 执行  中断 发送缓冲区 空 中断
  }
  if(usart_interrupt_flag_get(pusart_context->usart_periph, USART_INT_FLAG_ERR_ORERR) == SET) {
    // 错误中断 标志位
    usart_interrupt_flag_clear(pusart_context->usart_periph, USART_INT_FLAG_ERR_ORERR);
    // 清除 错误中断标志位
  }
}
 
 
void USART0_IRQHandler(void) // 串口0 中断服务函数
{
  usart_it(&usart0_context);
}
 
void USART1_IRQHandler(void) // 串口1 中断服务函数
{
  usart_it(&usart1_context);
}
 
void UART3_IRQHandler(void) // 串口2 中断服务函数
{
  usart_it(&usart3_context);
}
 
void UART4_IRQHandler(void) // 串口3 中断服务函数
{
  usart_it(&usart4_context);
}
 
 
#include <stdarg.h>
#include <stdio.h>
 
/*
 * usart printf function
 */
void usart_printf(usart_context_t *pusart_context, char *arg, ...)  
{  
  char buf[256], len;  
 
  va_list vl;  
  __va_start(vl, arg);  
  len = vsprintf(buf, arg, vl);   
  __va_end(vl);  
 
  usart_send_it(pusart_context, buf, len);
} 

/**
 @file fifo.c
 @brief 
 @author ZHK
 @version 0.1
 @date 2022/04/26
 @copyright Copyright (c) 2022  Shenzhen Nephotonics Inc
 */
 
 
#include <platform_systick.h>
 
void u_tm_log(char *arg, ...)
{
  char buf[512];
  uint32_t s, ms, len , n;
  uint32_t tm = get_systick_ms();
  s = tm / 1000;
  ms = tm % 1000;
  
  n = sprintf(buf, "[%d.%03d] ", s, ms);
  
  va_list vl;  
  __va_start(vl, arg);  
  len = vsprintf(buf+n, arg, vl);   
  __va_end(vl);  
  len = len+n;
  usart_send_it(&usart0_context, buf, len);
}
 
void u_log(char *arg, ...)
{
  char buf[512];
  uint32_t len;
  
  va_list vl;  
  __va_start(vl, arg);  
  len = vsprintf(buf, arg, vl);   
  __va_end(vl);  
  usart_send_it(&usart0_context, buf, len);
}
 
 