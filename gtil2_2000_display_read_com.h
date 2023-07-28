/* 
 * File:     gtil2_2000_display_read_com.h
 * Author:   Josia Kammler
 * Comments: 
 * Revision history: 
 *           20230718 pinout from board adapted
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef GTIL2_2000_DISPLAY_READ_COM_H
#define	GTIL2_2000_DISPLAY_READ_COM_H

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"

#define UART0_ID uart0
#define UART1_ID uart1
#define BAUD_RATE 9600
#define BUFFER_SIZE 254

// Pin assignments
#define UART0_TX_PIN 16        // 21
#define UART0_RX_PIN 17        // 22
#define UART1_TX_RX_CTRL_PIN 7 // 10
#define UART1_TX_PIN 8         // 11
#define UART1_RX_PIN 9         // 12
// for testing
#define PIN_LED      25
#define PIN_TEST10   10        // 14
#define PIN_TEST11   11        // 15
// device_id setting
#define PIN_DEV_ID0  20        // 26
#define PIN_DEV_ID1  19        // 25
#define PIN_DEV_ID2  18        // 24
// connection to display
#define PIN_RX2      15        // 20
// PWM out
#define PIN_PWM_OUT  14        // 19
// GPIO pins used: 7, 8, 9, 10, 11, 14, 15, 16, 17, 18, 19, 20, 25

// for UART0
#define tx0_intervall    10000000l // 10000000l us = 10 sec.

// for UART1
#define x1_start_byte            0x2b
#define x1_request_data_byte     0xb0
#define x1_response_data_byte    0xb1
#define x1_request_length_byte   0x02
#define x1_response_length_byte  0x0e   // 14 data bytes

// info
char version[] = "GTIL2_2000_Display_read_com Build 000 20230719";

// UART0 used to query energy day and energy total from WR
uint8_t  tx0_request_str[][6] = {{0x01, 0x03, 0x00, 0x3c, 0x00, 0x01}, // 0 energy day (60)
                                 {0x01, 0x03, 0x00, 0x3f, 0x00, 0x02}, // 1 energy total (63 + 64 --> read 2 register)
                                };
uint16_t tx0_request_idx = 0;
uint64_t tx0_reqest_time = 0;

// UART1 used for communication with the controller raspberry
// UART1 request string sample (coming from controller raspberry) - 7 bytes
// 2b     tx1_start_byte
// 01     device_id
// b0     tx1_request_data_byte
// 02     data_length
// dd dd  data (pwm_ctrl_val)
// cc     crc
// sample: 
// 2b 00 b0 02 00 10 ed

// UART1 response string string
// 2b 00 b1 0c dd dd dd dd dd dd dd dd dd dd dd dd dd dd xx

// buffers
uint8_t  tx0_buffer[BUFFER_SIZE + 1];
uint16_t tx0_in_p;
uint16_t tx0_out_p;
uint8_t  rx0_buffer[BUFFER_SIZE + 19];
uint16_t rx0_in_p;
uint16_t rx0_out_p;
uint64_t rx0_timeout;
uint64_t tx1_timeout = 0;
uint8_t  tx1_buffer[BUFFER_SIZE + 1];
uint16_t tx1_in_p;
uint16_t tx1_out_p;
uint8_t  rx1_buffer[BUFFER_SIZE + 19];
uint16_t rx1_in_p;
uint16_t rx1_out_p;
uint64_t rx1_timeout = 0;
uint8_t  rx2_buffer[BUFFER_SIZE + BUFFER_SIZE + 1];
uint16_t rx2_in_p;
uint16_t rx2_out_p;
uint64_t rx2_timeout = 0;

uint8_t  send_buffer1[BUFFER_SIZE + 1];

bool     rx2_rx_in_progress = 0;
uint64_t rx2_bit_times[10] = {0};
uint16_t rx2_bit_cnt = 0;
uint8_t  rx2_in_byte;
uint8_t  rx2_last_in_byte = 0;

// analyse
uint8_t  x1_device_id = 0x00;

// pwm
uint16_t pwm_ctrl_val = 1;
uint16_t pwm_slice;
uint16_t pwm_channel;

// functions
void IO_Init_loop(void);
void check_IO(void);
void parse_Rx0(void);
void parse_Rx1(void);
void parse_Rx2(void);
uint16_t crc_x0(uint8_t *, int);
void transfer_data_for_Tx1(void);
void read_device_id_setting(void);
void UART0_Init(void);
void UART0_Rx(void);
void UART0_Tx(void);
void UART1_Init(void);
void UART1_Rx(void);
void UART1_Tx(void);
void rx2_ISR();


#endif	/* GTIL2_2000_DISPLAY_READ_COM_H */
 
