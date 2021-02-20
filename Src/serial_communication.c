/**
  ******************************************************************************
  * File Name          : serial_communication.c
  * Description        : This file provides code for serial communication
  *                      of uart0.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "serial_communication.h"

/* Exported functions prototypes ---------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
struct Projector_parameter  g_projector_para;
struct asu_date asu_rec_data;
volatile uint8_t UartReceiveRxBuffer[UART_BUFFER_MAX_SIZE] = {0};
volatile uint8_t UartReceiveLength = 0;
int  data_len = 0;
char receive_buffer[40];
char R_0[15] = {114,32,48,13,13,10,32,79,75,32,50,48,66,13,10};
char R_1[13] = {114,32,49,13,13,10,32,79,75,32,49,13,10};
char R_2[15] = {82,32,50,13,13,10,32,79,75,32,50,52,49,13,10};
char OK_0[8] =  {13,13,10,32,79,75,13,10};
char OK_1[10] = {13,13,10,32,79,75,32,48,13,10};
char OK_3[10] = {13,13,10,32,79,75,32,50,13,10};
char OK_7[10] = {13,13,10,32,79,75,32,54,13,10};
char flag_0 = 0;

uint8_t wc_temp[438] = {0};
struct Projector_WC wc_data_temp[4];
uint8_t g_red_value,g_green_value, g_blue_value;
/*just for debug in bringup*/
uint8_t a[20] = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
uint8_t c[20] = {0xee,0xee,0xee,0xee,0xee,0xee,0xee,0xee,0xee,0xee};

const uint8_t g_kst[121][16] = {
	{0x08,	0xE3,	0xBE,	0x85,	0xFF,	0xDA,	0xC0,	0x47,	0x10,	0x9F,	0xFF,	0x7C,	0x00,	0xC7,	0xFF,	0x3B}, //-5,-5
	{0x08,	0xF1,	0xBE,	0xD0,	0xFF,	0xCD,	0xC0,	0x4E,	0x10,	0xCA,	0xFF,	0x6F,	0x00,	0xC8,	0xFF,	0x61}, //-4,-5
	{0x08,	0xFF,	0xBF,	0x1C,	0xFF,	0xC1,	0xC0,	0x55,	0x10,	0xF6,	0xFF,	0x62,	0x00,	0xC9,	0xFF,	0x88}, //-3,-5
	{0x09,	0x0D,	0xBF,	0x6A,	0xFF,	0xB5,	0xC0,	0x5D,	0x11,	0x23,	0xFF,	0x54,	0x00,	0xCA,	0xFF,	0xB1}, //-2,-5
	{0x09,	0x1A,	0xBF,	0xB4,	0xFF,	0xAA,	0xC0,	0x64,	0x11,	0x4F,	0xFF,	0x46,	0x00,	0xCB,	0xFF,	0xD8},
	{0x09,	0x26,	0xBF,	0xFF,	0xFF,	0x9F,	0xC0,	0x6B,	0x11,	0x7D,	0xFF,	0x38,	0x00,	0xCC,	0x00,	0x00},
	{0x09,	0x32,	0xC0,	0x4D,	0xFF,	0x9E,	0xC0,	0x73,	0x11,	0xA9,	0xFF,	0x2A,	0x00,	0xCD,	0x00,	0x28},
	{0x09,	0x3E,	0xC0,	0x9B,	0xFF,	0x9F,	0xC0,	0x7B,	0x11,	0xD6,	0xFF,	0x1D,	0x00,	0xCE,	0x00,	0x51},
	{0x09,	0x49,	0xC0,	0xEB,	0xFF,	0xA0,	0xC0,	0x83,	0x12,	0x03,	0xFF,	0x0F,	0x00,	0xCF,	0x00,	0x7B},
	{0x09,	0x53,	0xC1,	0x3D,	0xFF,	0xA1,	0xC0,	0x8B,	0x12,	0x31,	0xFF,	0x02,	0x00,	0xD1,	0x00,	0xA5},
	{0x09,	0x5D,	0xC1,	0x92,	0xFF,	0xA3,	0xC0,	0x93,	0x12,	0x5F,	0xFE,	0xF4,	0x00,	0xD2,	0x00,	0xD0},

	{0x08,	0xA6,	0xBE,	0x88,	0xFF,	0xEE,	0xC0,	0x38,	0x10,	0x55,	0xFF,	0x98,	0x00,	0x9E,	0xFF,	0x3D},//-5,-4
	{0x08,	0xB4,	0xBE,	0xD3,	0xFF,	0xE2,	0xC0,	0x3E,	0x10,	0x7F,	0xFF,	0x8D,	0x00,	0x9F,	0xFF,	0x63},
	{0x08,	0xC2,	0xBF,	0x1E,	0xFF,	0xD6,	0xC0,	0x44,	0x10,	0xAA,	0xFF,	0x83,	0x00,	0x9F,	0xFF,	0x89},
	{0x08,	0xCF,	0xBF,	0x6B,	0xFF,	0xCA,	0xC0,	0x49,	0x10,	0xD6,	0xFF,	0x78,	0x00,	0xA0,	0xFF,	0xB1},
	{0x08,	0xDC,	0xBF,	0xB4,	0xFF,	0xBF,	0xC0,	0x4F,	0x11,	0x01,	0xFF,	0x6D,	0x00,	0xA1,	0xFF,	0xD8},
	{0x08,	0xE8,	0xBF,	0xFF,	0xFF,	0xB4,	0xC0,	0x55,	0x11,	0x2D,	0xFF,	0x62,	0x00,	0xA1,	0x00,	0x00},
	{0x08,	0xF3,	0xC0,	0x4C,	0xFF,	0xB4,	0xC0,	0x5B,	0x11,	0x5A,	0xFF,	0x57,	0x00,	0xA2,	0x00,	0x28},
	{0x08,	0xFE,	0xC0,	0x9A,	0xFF,	0xB4,	0xC0,	0x62,	0x11,	0x85,	0xFF,	0x4C,	0x00,	0xA3,	0x00,	0x51},
	{0x09,	0x09,	0xC0,	0xE9,	0xFF,	0xB5,	0xC0,	0x68,	0x11,	0xB2,	0xFF,	0x42,	0x00,	0xA4,	0x00,	0x7A},
	{0x09,	0x13,	0xC1,	0x3B,	0xFF,	0xB7,	0xC0,	0x6E,	0x11,	0xDF,	0xFF,	0x37,	0x00,	0xA5,	0x00,	0xA4},
	{0x09,	0x1C,	0xC1,	0x8E,	0xFF,	0xB9,	0xC0,	0x75,	0x12,	0x0C,	0xFF,	0x2C,	0x00,	0xA6,	0x00,	0xCE},

	{0x08,	0x6D,	0xBE,	0x8C,	0x00,	0x01,	0xC0,	0x2A,	0x10,	0x0F,	0xFF,	0xB3,	0x00,	0x75,	0xFF,	0x3F},//-5,-3
	{0x08,	0x79,	0xBE,	0xD5,	0xFF,	0xF5,	0xC0,	0x2E,	0x10,	0x35,	0xFF,	0xAB,	0x00,	0x76,	0xFF,	0x64},
	{0x08,	0x87,	0xBF,	0x20,	0xFF,	0xE9,	0xC0,	0x32,	0x10,	0x5F,	0xFF,	0xA3,	0x00,	0x76,	0xFF,	0x8B},
	{0x08,	0x94,	0xBF,	0x6C,	0xFF,	0xDE,	0xC0,	0x37,	0x10,	0x8A,	0xFF,	0x9B,	0x00,	0x77,	0xFF,	0xB2},
	{0x08,	0xA0,	0xBF,	0xB5,	0xFF,	0xD3,	0xC0,	0x3B,	0x10,	0xB4,	0xFF,	0x93,	0x00,	0x77,	0xFF,	0xD8},
	{0x08,	0xAB,	0xBF,	0xFF,	0xFF,	0xC8,	0xC0,	0x3F,	0x10,	0xE0,	0xFF,	0x8B,	0x00,	0x78,	0x00,	0x00},
	{0x08,	0xB6,	0xC0,	0x4C,	0xFF,	0xC8,	0xC0,	0x44,	0x11,	0x0B,	0xFF,	0x83,	0x00,	0x79,	0x00,	0x28},
	{0x08,	0xC1,	0xC0,	0x98,	0xFF,	0xC9,	0xC0,	0x49,	0x11,	0x36,	0xFF,	0x7B,	0x00,	0x79,	0x00,	0x50},
	{0x08,	0xCB,	0xC0,	0xE7,	0xFF,	0xCA,	0xC0,	0x4D,	0x11,	0x62,	0xFF,	0x73,	0x00,	0x7A,	0x00,	0x79},
	{0x08,	0xD4,	0xC1,	0x39,	0xFF,	0xCC,	0xC0,	0x52,	0x11,	0x8F,	0xFF,	0x6B,	0x00,	0x7B,	0x00,	0xA3},
	{0x08,	0xDF,	0xC1,	0x8B,	0xFF,	0xCC,	0xC0,	0x57,	0x11,	0xC0,	0xFF,	0x5F,	0x00,	0x7C,	0x00,	0xCC},

	{0x08,	0x48,	0xBE,	0x92,	0x00,	0x01,	0xC0,	0x1C,	0x0F,	0xEF,	0xFF,	0xCD,	0x00,	0x4E,	0xFF,	0x40},//-5,-2
	{0x08,	0x49,	0xBE,	0xD9,	0x00,	0x01,	0xC0,	0x1F,	0x0F,	0xFE,	0xFF,	0xC8,	0x00,	0x4E,	0xFF,	0x66},
	{0x08,	0x4D,	0xBF,	0x22,	0xFF,	0xFC,	0xC0,	0x21,	0x10,	0x16,	0xFF,	0xC3,	0x00,	0x4E,	0xFF,	0x8C},
	{0x08,	0x5A,	0xBF,	0x6D,	0xFF,	0xF1,	0xC0,	0x24,	0x10,	0x40,	0xFF,	0xBD,	0x00,	0x4F,	0xFF,	0xB3},
	{0x08,	0x65,	0xBF,	0xB5,	0xFF,	0xE6,	0xC0,	0x27,	0x10,	0x69,	0xFF,	0xB8,	0x00,	0x4F,	0xFF,	0xD9},
	{0x08,	0x71,	0xBF,	0xFF,	0xFF,	0xDC,	0xC0,	0x2A,	0x10,	0x94,	0xFF,	0xB3,	0x00,	0x4F,	0x00,	0x00},
	{0x08,	0x7B,	0xC0,	0x4B,	0xFF,	0xDC,	0xC0,	0x2D,	0x10,	0xBF,	0xFF,	0xAD,	0x00,	0x50,	0x00,	0x27},
	{0x08,	0x85,	0xC0,	0x97,	0xFF,	0xDC,	0xC0,	0x30,	0x10,	0xE9,	0xFF,	0xA8,	0x00,	0x50,	0x00,	0x4F},
	{0x08,	0x8F,	0xC0,	0xE5,	0xFF,	0xDE,	0xC0,	0x33,	0x11,	0x14,	0xFF,	0xA3,	0x00,	0x51,	0x00,	0x78},
	{0x08,	0xA1,	0xC1,	0x35,	0xFF,	0xD7,	0xC0,	0x36,	0x11,	0x52,	0xFF,	0x8B,	0x00,	0x51,	0x00,	0xA1},
	{0x08,	0xB7,	0xC1,	0x84,	0xFF,	0xCD,	0xC0,	0x39,	0x11,	0x9A,	0xFF,	0x69,	0x00,	0x52,	0x00,	0xCB},

	{0x08,	0x24,	0xBE,	0x98,	0x00,	0x01,	0xC0,	0x0E,	0x0F,	0xCE,	0xFF,	0xE7,	0x00,	0x27,	0xFF,	0x42},//-5,-1
	{0x08,	0x24,	0xBE,	0xDE,	0x00,	0x01,	0xC0,	0x10,	0x0F,	0xDC,	0xFF,	0xE4,	0x00,	0x27,	0xFF,	0x67},
	{0x08,	0x24,	0xBF,	0x25,	0x00,	0x01,	0xC0,	0x11,	0x0F,	0xEC,	0xFF,	0xE2,	0x00,	0x27,	0xFF,	0x8D},
	{0x08,	0x25,	0xBF,	0x6F,	0x00,	0x01,	0xC0,	0x12,	0x0F,	0xFE,	0xFF,	0xDF,	0x00,	0x27,	0xFF,	0xB3},
	{0x08,	0x2D,	0xBF,	0xB6,	0xFF,	0xF8,	0xC0,	0x14,	0x10,	0x20,	0xFF,	0xDD,	0x00,	0x27,	0xFF,	0xD9},
	{0x08,	0x38,	0xBF,	0xFF,	0xFF,	0xEE,	0xC0,	0x15,	0x10,	0x49,	0xFF,	0xDA,	0x00,	0x28,	0x00,	0x00},
	{0x08,	0x42,	0xC0,	0x4B,	0xFF,	0xEE,	0xC0,	0x17,	0x10,	0x73,	0xFF,	0xD7,	0x00,	0x28,	0x00,	0x27},
	{0x08,	0x4F,	0xC0,	0x96,	0xFF,	0xEC,	0xC0,	0x18,	0x10,	0xA4,	0xFF,	0xCE,	0x00,	0x28,	0x00,	0x4E},
	{0x08,	0x64,	0xC0,	0xE2,	0xFF,	0xE2,	0xC0,	0x19,	0x10,	0xE7,	0xFF,	0xB2,	0x00,	0x28,	0x00,	0x77},
	{0x08,	0x7A,	0xC1,	0x2F,	0xFF,	0xD8,	0xC0,	0x1B,	0x11,	0x2D,	0xFF,	0x94,	0x00,	0x28,	0x00,	0xA0},
	{0x08,	0x90,	0xC1,	0x7D,	0xFF,	0xCE,	0xC0,	0x1C,	0x11,	0x74,	0xFF,	0x74,	0x00,	0x29,	0x00,	0xC9},

	{0x08,	0x00,	0xBE,	0x9F,	0xFF,	0xFF,	0xC0,	0x00,	0x0F,	0xAC,	0x00,	0x00,	0x00,	0x00,	0xFF,	0x44},//-5,0
	{0x08,	0x00,	0xBE,	0xE3,	0xFF,	0xFF,	0xC0,	0x00,	0x0F,	0xBA,	0x00,	0x00,	0x00,	0x00,	0xFF,	0x68},
	{0x08,	0x00,	0xBF,	0x29,	0x00,	0x01,	0xC0,	0x00,	0x0F,	0xC9,	0x00,	0x00,	0x00,	0x00,	0xFF,	0x8E},
	{0x08,	0x00,	0xBF,	0x71,	0x00,	0x01,	0xC0,	0x00,	0x0F,	0xDB,	0x00,	0x00,	0x00,	0x00,	0xFF,	0xB4},
	{0x08,	0x00,	0xBF,	0xB7,	0x00,	0x01,	0xC0,	0x00,	0x0F,	0xEC,	0x00,	0x00,	0x00,	0x00,	0xFF,	0xD9},
	{0x08,	0x00,	0xBF,	0xFF,	0xFF,	0xFF,	0xC0,	0x00,	0x10,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},
	{0x08,	0x15,	0xC0,	0x49,	0xFF,	0xF6,	0xC0,	0x00,	0x10,	0x3F,	0xFF,	0xEA,	0x00,	0x00,	0x00,	0x26},
	{0x08,	0x2A,	0xC0,	0x93,	0xFF,	0xEC,	0xC0,	0x00,	0x10,	0x7F,	0xFF,	0xD2,	0x00,	0x00,	0x00,	0x4E},
	{0x08,	0x3F,	0xC0,	0xDE,	0xFF,	0xE2,	0xC0,	0x00,	0x10,	0xC1,	0xFF,	0xB9,	0x00,	0x00,	0x00,	0x76},
	{0x08,	0x54,	0xC1,	0x2A,	0xFF,	0xD8,	0xC0,	0x00,	0x11,	0x06,	0xFF,	0x9D,	0x00,	0x00,	0x00,	0x9E},
	{0x08,	0x6A,	0xC1,	0x76,	0xFF,	0xCE,	0xC0,	0x00,	0x11,	0x4C,	0xFF,	0x81,	0x00,	0x00,	0x00,	0xC7},

	{0x07,	0xDD,	0xBE,	0xA5,	0x00,	0x01,	0xBF,	0xF3,	0x0F,	0x89,	0x00,	0x00,	0xFF,	0xDA,	0xFF,	0x45},
	{0x07,	0xDD,	0xBE,	0xE8,	0xFF,	0xFF,	0xBF,	0xF2,	0x0F,	0x96,	0x00,	0x00,	0xFF,	0xDA,	0xFF,	0x6A},
	{0x07,	0xDD,	0xBF,	0x2D,	0x00,	0x01,	0xBF,	0xF0,	0x0F,	0xA6,	0x00,	0x00,	0xFF,	0xDA,	0xFF,	0x8F},
	{0x07,	0xDD,	0xBF,	0x74,	0xFF,	0xFF,	0xBF,	0xEF,	0x0F,	0xB6,	0x00,	0x00,	0xFF,	0xD9,	0xFF,	0xB5},
	{0x07,	0xE4,	0xBF,	0xB8,	0x00,	0x01,	0xBF,	0xEE,	0x0F,	0xD7,	0x00,	0x00,	0xFF,	0xD9,	0xFF,	0xDA},
	{0x07,	0xEF,	0xBF,	0xFF,	0x00,	0x01,	0xBF,	0xEC,	0x10,	0x00,	0x00,	0x00,	0xFF,	0xD9,	0x00,	0x00},
	{0x07,	0xF9,	0xC0,	0x48,	0xFF,	0xF6,	0xBF,	0xEB,	0x10,	0x29,	0x00,	0x00,	0xFF,	0xD9,	0x00,	0x26},
	{0x08,	0x05,	0xC0,	0x90,	0xFF,	0xED,	0xBF,	0xE9,	0x10,	0x58,	0xFF,	0xFA,	0xFF,	0xD9,	0x00,	0x4D},
	{0x08,	0x19,	0xC0,	0xDA,	0xFF,	0xE3,	0xBF,	0xE8,	0x10,	0x9A,	0xFF,	0xE1,	0xFF,	0xD8,	0x00,	0x75},
	{0x08,	0x2E,	0xC1,	0x25,	0xFF,	0xD9,	0xBF,	0xE7,	0x10,	0xDE,	0xFF,	0xC6,	0xFF,	0xD8,	0x00,	0x9D},
	{0x08,	0x43,	0xC1,	0x70,	0xFF,	0xCF,	0xBF,	0xE5,	0x11,	0x23,	0xFF,	0xAA,	0xFF,	0xD8,	0x00,	0xC5},

	{0x07,	0xBA,	0xBE,	0xAB,	0xFF,	0xFF,	0xBF,	0xE6,	0x0F,	0x64,	0x00,	0x00,	0xFF,	0xB5,	0xFF,	0x47},
	{0x07,	0xBA,	0xBE,	0xED,	0xFF,	0xFF,	0xBF,	0xE3,	0x0F,	0x71,	0x00,	0x00,	0xFF,	0xB5,	0xFF,	0x6B},
	{0x07,	0xBE,	0xBF,	0x30,	0xFF,	0xFF,	0xBF,	0xE1,	0x0F,	0x88,	0x00,	0x00,	0xFF,	0xB4,	0xFF,	0x90},
	{0x07,	0xC9,	0xBF,	0x75,	0x00,	0x01,	0xBF,	0xDE,	0x0F,	0xB0,	0x00,	0x00,	0xFF,	0xB4,	0xFF,	0xB5},
	{0x07,	0xD4,	0xBF,	0xB9,	0x00,	0x01,	0xBF,	0xDB,	0x0F,	0xD8,	0x00,	0x00,	0xFF,	0xB4,	0xFF,	0xDA},
	{0x07,	0xDE,	0xBF,	0xFF,	0x00,	0x00,	0xBF,	0xD8,	0x10,	0x00,	0x00,	0x00,	0xFF,	0xB3,	0x00,	0x00},
	{0x07,	0xE8,	0xC0,	0x48,	0xFF,	0xF6,	0xBF,	0xD5,	0x10,	0x28,	0x00,	0x00,	0xFF,	0xB3,	0x00,	0x26},
	{0x07,	0xF0,	0xC0,	0x8F,	0xFF,	0xED,	0xBF,	0xD3,	0x10,	0x51,	0x00,	0x00,	0xFF,	0xB3,	0x00,	0x4C},
	{0x07,	0xF9,	0xC0,	0xD7,	0xFF,	0xE3,	0xBF,	0xD0,	0x10,	0x7A,	0x00,	0x01,	0xFF,	0xB2,	0x00,	0x73},
	{0x08,	0x09,	0xC1,	0x20,	0xFF,	0xDA,	0xBF,	0xCD,	0x10,	0xB4,	0xFF,	0xF0,	0xFF,	0xB2,	0x00,	0x9B},
	{0x08,	0x1E,	0xC1,	0x69,	0xFF,	0xD0,	0xBF,	0xCA,	0x10,	0xF9,	0xFF,	0xD4,	0xFF,	0xB1,	0x00,	0xC3},

	{0x07,	0x98,	0xBE,	0xB1,	0xFF,	0xFF,	0xBF,	0xD9,	0x0F,	0x3E,	0x00,	0x00,	0xFF,	0x90,	0xFF,	0x49},
	{0x07,	0xA2,	0xBE,	0xF0,	0x00,	0x01,	0xBF,	0xD5,	0x0F,	0x61,	0x00,	0x00,	0xFF,	0x90,	0xFF,	0x6C},
	{0x07,	0xAE,	0xBF,	0x32,	0x00,	0x01,	0xBF,	0xD1,	0x0F,	0x89,	0x00,	0x00,	0xFF,	0x90,	0xFF,	0x91},
	{0x07,	0xBA,	0xBF,	0x76,	0x00,	0x01,	0xBF,	0xCD,	0x0F,	0xB1,	0x00,	0x00,	0xFF,	0x8F,	0xFF,	0xB6},
	{0x07,	0xC4,	0xBF,	0xB9,	0x00,	0x01,	0xBF,	0xC9,	0x0F,	0xD8,	0x00,	0x00,	0xFF,	0x8F,	0xFF,	0xDA},
	{0x07,	0xCE,	0xBF,	0xFF,	0xFF,	0xFF,	0xBF,	0xC5,	0x10,	0x00,	0x00,	0x00,	0xFF,	0x8E,	0x00,	0x00},
	{0x07,	0xD8,	0xC0,	0x47,	0xFF,	0xF6,	0xBF,	0xC0,	0x10,	0x28,	0x00,	0x00,	0xFF,	0x8E,	0x00,	0x25},
	{0x07,	0xE0,	0xC0,	0x8E,	0xFF,	0xED,	0xBF,	0xBC,	0x10,	0x50,	0x00,	0x01,	0xFF,	0x8D,	0x00,	0x4C},
	{0x07,	0xE8,	0xC0,	0xD5,	0xFF,	0xE4,	0xBF,	0xB8,	0x10,	0x78,	0x00,	0x02,	0xFF,	0x8C,	0x00,	0x72},
	{0x07,	0xF0,	0xC1,	0x1C,	0xFF,	0xDA,	0xBF,	0xB3,	0x10,	0xA1,	0x00,	0x02,	0xFF,	0x8C,	0x00,	0x9A},
	{0x07,	0xF9,	0xC1,	0x63,	0xFF,	0xD1,	0xBF,	0xAF,	0x10,	0xCD,	0x00,	0x00,	0xFF,	0x8B,	0x00,	0xC1},

	{0x07,	0x88,	0xBE,	0xB4,	0xFF,	0xFF,	0xBF,	0xCC,	0x0F,	0x3C,	0x00,	0x00,	0xFF,	0x6D,	0xFF,	0x4A},
	{0x07,	0x94,	0xBE,	0xF2,	0x00,	0x01,	0xBF,	0xC7,	0x0F,	0x62,	0x00,	0x00,	0xFF,	0x6C,	0xFF,	0x6D},
	{0x07,	0xA0,	0xBF,	0x33,	0xFF,	0xFF,	0xBF,	0xC2,	0x0F,	0x89,	0x00,	0x00,	0xFF,	0x6C,	0xFF,	0x92},
	{0x07,	0xAB,	0xBF,	0x77,	0xFF,	0xFF,	0xBF,	0xBC,	0x0F,	0xB1,	0x00,	0x00,	0xFF,	0x6B,	0xFF,	0xB7},
	{0x07,	0xB5,	0xBF,	0xBA,	0x00,	0x01,	0xBF,	0xB7,	0x0F,	0xD8,	0x00,	0x00,	0xFF,	0x6A,	0xFF,	0xDB},
	{0x07,	0xBF,	0xBF,	0xFF,	0x00,	0x01,	0xBF,	0xB2,	0x10,	0x00,	0x00,	0x00,	0xFF,	0x6A,	0x00,	0x00},
	{0x07,	0xC8,	0xC0,	0x47,	0xFF,	0xF6,	0xBF,	0xAC,	0x10,	0x27,	0x00,	0x00,	0xFF,	0x69,	0x00,	0x25},
	{0x07,	0xD1,	0xC0,	0x8D,	0xFF,	0xED,	0xBF,	0xA6,	0x10,	0x4E,	0x00,	0x01,	0xFF,	0x68,	0x00,	0x4B},
	{0x07,	0xD9,	0xC0,	0xD3,	0xFF,	0xE4,	0xBF,	0xA0,	0x10,	0x76,	0x00,	0x02,	0xFF,	0x67,	0x00,	0x71},
	{0x07,	0xE0,	0xC1,	0x1A,	0xFF,	0xDA,	0xBF,	0x9A,	0x10,	0x9E,	0x00,	0x03,	0xFF,	0x66,	0x00,	0x99},
	{0x07,	0xE7,	0xC1,	0x5F,	0xFF,	0xD1,	0xBF,	0x94,	0x10,	0xC7,	0x00,	0x04,	0xFF,	0x65,	0x00,	0xC0},

	{0x07,	0x7A,	0xBE,	0xB6,	0x00,	0x01,	0xBF,	0xC0,	0x0F,	0x3C,	0x00,	0x00,	0xFF,	0x49,	0xFF,	0x4C},
	{0x07,	0x86,	0xBE,	0xF4,	0xFF,	0xFF,	0xBF,	0xBA,	0x0F,	0x62,	0x00,	0x00,	0xFF,	0x49,	0xFF,	0x6F},
	{0x07,	0x92,	0xBF,	0x35,	0xFF,	0xFF,	0xBF,	0xB3,	0x0F,	0x89,	0x00,	0x00,	0xFF,	0x48,	0xFF,	0x92},
	{0x07,	0x9D,	0xBF,	0x78,	0xFF,	0xFF,	0xBF,	0xAC,	0x0F,	0xB1,	0x00,	0x00,	0xFF,	0x47,	0xFF,	0xB7},
	{0x07,	0xA7,	0xBF,	0xBA,	0xFF,	0xFF,	0xBF,	0xA5,	0x0F,	0xD8,	0x00,	0x00,	0xFF,	0x46,	0xFF,	0xDB},
	{0x07,	0xB1,	0xBF,	0xFF,	0x00,	0x01,	0xBF,	0x9F,	0x10,	0x00,	0x00,	0x00,	0xFF,	0x46,	0x00,	0x00},
	{0x07,	0xBA,	0xC0,	0x46,	0xFF,	0xF6,	0xBF,	0x97,	0x10,	0x27,	0x00,	0x00,	0xFF,	0x45,	0x00,	0x25},
	{0x07,	0xC2,	0xC0,	0x8C,	0xFF,	0xED,	0xBF,	0x90,	0x10,	0x4D,	0x00,	0x02,	0xFF,	0x44,	0x00,	0x4A},
	{0x07,	0xCA,	0xC0,	0xD2,	0xFF,	0xE4,	0xBF,	0x89,	0x10,	0x75,	0x00,	0x03,	0xFF,	0x43,	0x00,	0x70},
	{0x07,	0xD1,	0xC1,	0x18,	0xFF,	0xDB,	0xBF,	0x82,	0x10,	0x9C,	0x00,	0x04,	0xFF,	0x42,	0x00,	0x97},
	{0x07,	0xD8,	0xC1,	0x5D,	0xFF,	0xD2,	0xBF,	0x7B,	0x10,	0xC4,	0x00,	0x06,	0xFF,	0x40,	0x00,	0xBE},
};

extern uint32_t g_fan_value;
extern uint8_t gpiopin;
extern uint16_t g_RedCurrent;
extern uint16_t g_GreenCurrent;
extern uint16_t g_BlueCurrent;
/* Private function prototypes -----------------------------------------------*/
unsigned char CharToHex(unsigned char bHex)
{
	if((bHex>='0')&&(bHex<='9'))
	{
		bHex -= 0x30;
	}
	else if((bHex>='a')&&(bHex<='f'))
	{
		bHex -= 0x57;
	}
	else if((bHex>='A')&&(bHex<='F'))
	{
		bHex -= 0x37;
	}
	return bHex;
}

unsigned char HexToChar(unsigned char bHex)
{
	unsigned char bHex_ret;
	if(bHex<=0x9)
	{
		bHex += 0x30;
	}
	else
	{
		bHex += 0x57;
	}
	bHex_ret =  bHex;
	return bHex_ret;
}

int read_evm(char* buf,int len, int* para0_len,int* para1_len)
{
	int i, read_len,flag;
	char read_char,flag0,flag1;
	read_len = 0;
	*para0_len = 0;
	*para1_len = 0;
	flag = 1;
	for(i=0; i < len;i++)
	{
		read_len = read_len + 1;
		read_char = buf[i];

		if(i==0)
		{
			flag0 = read_char;
		}
		if(i==1)
		{
			flag1 = read_char;
		}
		if((flag0 == 'h' && flag1 == 'r') || (flag0 == 'p' && flag1 == 'r'))
		{
			if((read_len>3) && (flag==1))
			{
				if(read_char == 13)
				{
					flag = 0;
				}
				else
				{
					*para0_len = *para0_len + 1;
				}
			}
		}

		else if((flag0 == 'h' && flag1 == 'w') || (flag0 == 'p' && flag1 == 'w'))
		{
			if((read_len>3) && (flag==1))
			{
				if(read_char == 32)
				{
					flag = 2;
				}
				else
				{
					*para0_len = *para0_len + 1;
				}
			}
			if(flag==2 && read_char!=32)
			{
				if(read_char == 13)
				{
					flag = 3;
				}
				else
				{
					*para1_len = *para1_len + 1;
				}
			}
		}
		else
		{
			if((read_len>5) && (flag==1))
			{
				if(read_char == 13)
				{
					flag = 0;
				}
				else
				{
					*para0_len = *para0_len + 1;
				}
			}
		}	
		if(read_char==10)
			break;
	}
	return read_len;
}

void para_Analysis(char* buf,int para0_len,int para1_len)
{

	if(((buf[2]=='8') && (buf[3]=='8')) || ((buf[2]=='8') && (buf[3]=='e')))
	{
		asu_rec_data.chip_addr = CharToHex(buf[5])*16 + CharToHex(buf[6]);
	}
	if(((buf[2]=='8') && (buf[3]=='9')) || ((buf[2]=='8') && (buf[3]=='f')))
	{
		if(para0_len == 1)
			asu_rec_data.reg_addr = CharToHex(buf[5]);
		if(para0_len == 2)
			asu_rec_data.reg_addr = CharToHex(buf[5])*16 + CharToHex(buf[6]);
		if(para0_len == 3)
			asu_rec_data.reg_addr = CharToHex(buf[5])*16*16 + CharToHex(buf[6])*16+CharToHex(buf[7]);
		if(para0_len == 4)
			asu_rec_data.reg_addr = CharToHex(buf[5])*16*16*16 + CharToHex(buf[6])*16*16 + CharToHex(buf[7])*16 + CharToHex(buf[8]);
	}
	if(((buf[2]=='8') && (buf[3]=='b'))||((buf[2]=='8') && (buf[3]=='B')) || ((buf[2]=='9') && (buf[3]=='1')))
	{
		if(para0_len == 1)
			asu_rec_data.reg_value = CharToHex(buf[5]);
		if(para0_len == 2)
			asu_rec_data.reg_value = CharToHex(buf[5])*16 + CharToHex(buf[6]);
		if(para0_len == 3)
			asu_rec_data.reg_value = CharToHex(buf[5])*16*16 + CharToHex(buf[6])*16+CharToHex(buf[7]);
		if(para0_len == 4)
			asu_rec_data.reg_value = CharToHex(buf[5])*16*16*16 + CharToHex(buf[6])*16*16 + CharToHex(buf[7])*16 + CharToHex(buf[8]);
		asu_rec_data.w_r = 1 ;

	}
	if(((buf[2]=='9') && (buf[3]=='2')) || ((buf[2]=='8') && (buf[3]=='c'))) 
	{
		asu_rec_data.w_r = 2 ;
	}

	if((buf[0]=='p') && (buf[1]=='r'))
	{
		asu_rec_data.chip_addr = SXRD241_I2C_ADDRESS;
		if(para0_len == 1)
			asu_rec_data.reg_addr = CharToHex(buf[3]);
		if(para0_len == 2)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16 + CharToHex(buf[4]);
		if(para0_len == 3)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16 + CharToHex(buf[4])*16 + CharToHex(buf[5]);
		if(para0_len == 4)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16*16 + CharToHex(buf[4])*16*16 + CharToHex(buf[5])*16 + CharToHex(buf[6]);
		asu_rec_data.w_r = 3 ;
	}
	if((buf[0]=='h') && (buf[1]=='r'))
	{
		asu_rec_data.chip_addr = CXD3554_I2C_ADDRESS;
		if(para0_len == 1)
			asu_rec_data.reg_addr = CharToHex(buf[3]);
		if(para0_len == 2)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16 + CharToHex(buf[4]);
		if(para0_len == 3)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16 + CharToHex(buf[4])*16 + CharToHex(buf[5]);
		if(para0_len == 4)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16*16 + CharToHex(buf[4])*16*16 + CharToHex(buf[5])*16 + CharToHex(buf[6]);
		asu_rec_data.w_r = 4 ;
	}
	if(para1_len!=0)
	{
		if((buf[0]=='h') && (buf[1]=='w'))
			asu_rec_data.chip_addr = CXD3554_I2C_ADDRESS;
		else
			asu_rec_data.chip_addr = SXRD241_I2C_ADDRESS;

		if(para0_len == 1)
			asu_rec_data.reg_addr = CharToHex(buf[3]);
		if(para0_len == 2)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16 + CharToHex(buf[4]);
		if(para0_len == 3)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16 + CharToHex(buf[4])*16 + CharToHex(buf[5]);
		if(para0_len == 4)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16*16 + CharToHex(buf[4])*16*16 + CharToHex(buf[5])*16 + CharToHex(buf[6]);

		if(para1_len == 1)
			asu_rec_data.reg_value = CharToHex(buf[3 + para0_len +1]);
		if(para1_len == 2)
			asu_rec_data.reg_value = CharToHex(buf[3 + para0_len +1])*16 + CharToHex(buf[3 + para0_len +2]);
		if(para1_len == 3)
			asu_rec_data.reg_value = CharToHex(buf[3 + para0_len +1])*16*16 + CharToHex(buf[3 + para0_len +2])*16 + CharToHex(buf[3 + para0_len +3]);
		if(para1_len == 4)
			asu_rec_data.reg_value = CharToHex(buf[3 + para0_len +1])*16*16*16 + CharToHex(buf[3 + para0_len +2])*16*16 + CharToHex(buf[3 + para0_len +3])*16 + CharToHex(buf[3 + para0_len +4]) ;
		asu_rec_data.w_r = 5 ;
	}
}

void show_A100_data(struct asu_date *data)
{
	if(data->reg_value <=0xf)
		data_len = 1;
	if(data->reg_value >=0x10 && data->reg_value <=0xff)
		data_len = 2;
	if(data->reg_value >=0x100 && data->reg_value <=0xfff)
		data_len = 3;
	if(data->reg_value >=0x1000 && data->reg_value <=0xffff)
		data_len = 4;
}
/* flash ------------------------------------------------------------------*/
/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}

void A100_GetParameter(void)
{
	uint32_t *Projector_Config;
	uint32_t i;
	uint32_t Address;	
	
	/* Check the correctness of written data */
	Projector_Config = (uint32_t *)&g_projector_para;
	Address = FLASH_USER_START_ADDR;
	for(i=0 ;i<sizeof(struct Projector_parameter)/4 ;i++)
	{
		Projector_Config[i] = *(__IO uint32_t*) Address	;
		Address += 4;
	}
	
	return;
}

void A100_setparameter(struct Projector_parameter  *g_projector_parameter )
{
	uint32_t FirstPage = 0, NbOfPages = 0;
	uint32_t Address = 0, PageError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint64_t *projector_Config;
	uint32_t i;

	projector_Config =  (uint64_t *)g_projector_parameter;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
  
  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  /* Get the 1st page to erase */
  FirstPage = GetPage(FLASH_USER_START_ADDR);

  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;	
	
	printf("A100_setparameter FirstPage[%d] NbOfPages[%d]\r\n",FirstPage, NbOfPages);
 
  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    /*
      Error occurred while page erase.
      User can add here some code to deal with this error.
      PageError will contain the faulty page and then to know the code error on this page,
      user can call function 'HAL_FLASH_GetError()'
    */
		printf("HAL_FLASHEx_Erase errcode[%d]\r\n",HAL_FLASH_GetError());
  }
  /* Program the user Flash area word by word
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  Address = FLASH_USER_START_ADDR;
	for(i=0 ;i<sizeof(struct Projector_parameter)/8 ;i++)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, *projector_Config) == HAL_OK)
		{
			Address = Address + 8;
			projector_Config = projector_Config + 1;
		} else {
		/* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */
			printf("HAL_FLASH_Program errcode[%d]\r\n",HAL_FLASH_GetError());				
		}
	}

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
	
	return;
}


HAL_StatusTypeDef A100_SetBootPinMode(void)
{
	FLASH_OBProgramInitTypeDef pOBInit;
	HAL_StatusTypeDef ret = HAL_ERROR;
	
	memset(&pOBInit,0,sizeof(FLASH_OBProgramInitTypeDef));
	pOBInit.WRPArea = OB_WRPAREA_ZONE_A;
	HAL_FLASHEx_OBGetConfig(&pOBInit);
	printf("get---RDPLevel:0x%x OPTIONTYPE:0x%x USERConfig:0x%x\r\n", pOBInit.RDPLevel, pOBInit.OptionType, pOBInit.USERConfig);
	if(GET_BIT(pOBInit.USERConfig, FLASH_OPTR_nBOOT_SEL_Pos) == 0) {
		return HAL_OK;
	}
	
	pOBInit.OptionType = OPTIONBYTE_USER;
	pOBInit.USERType = OB_USER_nBOOT_SEL;
	pOBInit.USERConfig = OB_BOOT0_FROM_PIN;
	ret = HAL_FLASH_Unlock();
	if(HAL_OK != ret)
	{
			printf("Flash Unlock fail \r\n");
			return ret;
	}	
	ret = HAL_FLASH_OB_Unlock();
	if(HAL_OK != ret)
	{
			printf("ob Unlock fail \r\n");
			return ret;
	}
	
	ret = HAL_FLASHEx_OBProgram(&pOBInit);
	if(HAL_OK != ret)
	{
			printf("HAL_FLASHEx_OBProgram fail:%d \r\n",ret);
			return ret;
	}

	ret = HAL_FLASH_OB_Lock();
	if(HAL_OK != ret)
	{
			printf("ob Unlock fail \r\n");
			return ret;
	}
	ret = HAL_FLASH_Lock();
	if(HAL_OK != ret)
	{
			printf("flash Unlock fail:%d \r\n",ret);
			return ret;
	}
	
	return HAL_OK;
}
/* usart ------------------------------------------------------------------*/
#ifdef __GUNC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);	
	return ch;
}

void A100_UartCmdHandler(uint8_t *pRx,uint8_t length)
{
	uint8_t I2cBuf;
	I2C_HandleTypeDef hi2c;
  uint16_t i;
	uint16_t  checksum;
	uint16_t  MemAddSize;
	uint32_t  value0;

	struct Projector_date *recevie_data;
	struct Projector_CCT  *recevie_cct_data;
	struct Projector_Gama *recevie_gamma_data;
	struct Projector_wb_date *recevie_wb_data;
#if 0
	printf("\n\r A100_UartCmdHandler %d Bytes:",length);
	for(uint16_t i = 0; i < length; i++)
	{
		printf(" 0x%02X", pRx[i]);
	}
	printf("\n\r");
#endif	
	if(pRx[0] == A100_SET_TUNING_PARAMETER)
	{
		recevie_cct_data = (struct Projector_CCT *)(pRx);
		memcpy(&(g_projector_para.projector_tuning),recevie_cct_data,sizeof(struct Projector_CCT));
		//A100_setparameter(&g_projector_para);
		return;
	}
	else if(pRx[0] == A100_SET_WC_PARAMETER)
	{
			int i;
			recevie_wb_data = ( struct Projector_wb_date *)(pRx);
			if((recevie_wb_data->direction)<4)
			{
				wc_data_temp[recevie_wb_data->direction].power_num = recevie_wb_data->power_num;
				wc_data_temp[recevie_wb_data->direction].gain = recevie_wb_data->gain;
				wc_data_temp[recevie_wb_data->direction].offset = recevie_wb_data->offset;
				wc_data_temp[recevie_wb_data->direction].l = recevie_wb_data->l;
			}
			if((recevie_wb_data->direction)==0)
			{
				for(i=0;i<132;i++)
				{
						wc_temp[i+0xA0] = recevie_wb_data->reg[i];
				}
			}
			else if ((recevie_wb_data->direction)==1)
			{
				for(i=0;i<78;i++)
				{
						wc_temp[i] = recevie_wb_data->reg[i];
				}
			}
			else if ((recevie_wb_data->direction)==2)
			{
				for(i=0;i<132;i++)
				{
						wc_temp[i+0x130] = recevie_wb_data->reg[i];
				}
			}
			else if ((recevie_wb_data->direction)==3)
			{
				for(i=0;i<78;i++)
				{
						wc_temp[i+0x50] = recevie_wb_data->reg[i];
				}
			}
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40,0x6c);
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A42,0x70);		
			for(i=0; i<438; i++)
			{
					A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS,0x3500 +i , wc_temp[i]);
			}
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40,0x6d);
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A42,0x30);
			HAL_UART_Transmit(&huart1, c,  10, 100);
			return;
	}	
	else if(pRx[0] == A100_SET_GAMA_PARAMETER)
	{
		recevie_gamma_data = (struct Projector_Gama *)(pRx);
		memcpy(&(g_projector_para.gamma_data),recevie_gamma_data,sizeof(struct Projector_Gama));
		//A100_setparameter(&g_projector_para);
		HAL_UART_Transmit(&huart1, a,  20, 100);
	}
	else if(pRx[0] == A100_UART_COMMAND_HEAD)
	{
	//cal checksum here
		recevie_data = (struct Projector_date *)(pRx);
		checksum = (recevie_data->command + recevie_data->data0 + recevie_data->data1 + recevie_data->data2 + recevie_data->data3 + recevie_data->data4+ recevie_data->data5) % 0x1000;
	//check STX & ETX
		if((A100_UART_COMMAND_HEAD != recevie_data->start_head) || (A100_UART_COMMAND_END != recevie_data->end_tail))
		{
			recevie_data->data5 = 101;
			HAL_UART_Transmit(&huart1, (uint8_t *)recevie_data,  sizeof(struct Projector_date), 100);
		}

		//checksum error
		if(checksum != recevie_data->checksum)
		{
			if(checksum != 0xffff) //universual checksum
			{
				recevie_data->data4 = checksum;
				recevie_data->data5 = 100;
				HAL_UART_Transmit(&huart1, (uint8_t *)recevie_data,  sizeof(struct Projector_date), 100);
				return;
			}
		}	
			switch(recevie_data->command)
			{		
			/*		
					case A100_SET_LED:
						switch(recevie_data->data0)
						{
							case A100_SET_LED_RRGGBBGG:
								A100_Set_LED_RRGGBBGG();
							break;

							case A100_SET_LED_RRGGRRBB:
								A100_Set_LED_RRGGRRBB();
							break;

							case A100_SET_LED_RRGGBBWW:
								A100_Set_LED_RRBBGGBB();
							break;

							case A100_SET_LED_RRBBGGBB:
								A100_Set_LED_RRBBGGBB();
							break;
						}
					break;
			*/
					case A100_SET_OE_FAN_SPEED:
						if(recevie_data->data0 == 1)
							A100_SetFan12Speed(recevie_data->data1);
						else if(recevie_data->data0 == 2)
							A100_SetFan34Speed(recevie_data->data1);
						else if(recevie_data->data0 == 3)	
							A100_SetFan5Speed(recevie_data->data1);							
					break;

					case A100_SET_CURRENT_MODE:
						if(recevie_data->data0<3 && g_projector_para.projector_tuning.tuning_valid == 1 )
						{
								uint16_t Red_Current = 0x0000;
								uint16_t Green_Current = 0x0000;
								uint16_t Blue_Current = 0x0000;
								g_projector_para.projector_tuning.briness_index = recevie_data->data0;
								Red_Current 	= g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].red_current ;
								Green_Current = g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].green_current ;
								Blue_Current 	= g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].blue_current ;
								A100_SetRedCurrent(Red_Current);
								A100_SetGreenCurrent(Green_Current);
								A100_SetBlueCurrent(Blue_Current);

								recevie_data->data0 = g_projector_para.projector_tuning.cct_index;
								recevie_data->data1 = g_projector_para.projector_tuning.briness_index;
								recevie_data->data3 = Red_Current;
								recevie_data->data4 = Green_Current;
								recevie_data->data5 = Blue_Current;
						}
						else
						{
								recevie_data->data0 = 257;
						}
					break;

					case A100_SET_CCT_MODE:
					{
							if(recevie_data->data0<3  && g_projector_para.projector_tuning.tuning_valid == 1 )
							{
								uint16_t Red_Current = 0x0000;
								uint16_t Green_Current = 0x0000;
								uint16_t Blue_Current = 0x0000;
								g_projector_para.projector_tuning.cct_index = recevie_data->data0;
								Red_Current 	= g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].red_current ;
								Green_Current = g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].green_current ;
								Blue_Current 	= g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].blue_current ;
								A100_SetRedCurrent(Red_Current);
								A100_SetGreenCurrent(Green_Current);
								A100_SetBlueCurrent(Blue_Current);
								recevie_data->data0 = g_projector_para.projector_tuning.cct_index;
								recevie_data->data1 = g_projector_para.projector_tuning.briness_index;
								recevie_data->data3 = Red_Current;
								recevie_data->data4 = Green_Current;
								recevie_data->data5 = Blue_Current;
							}
							else
							{
								recevie_data->data0 = 256;
							}
					}
					break;

					case A100_SET_HAPTIC_START:

					break;

					case A100_I2C_WRITE:
					{
							if(recevie_data->data0 == 1)
							{
								hi2c = hi2c1;
							}
							else if(recevie_data->data0 == 2)
							{
								hi2c = hi2c2;
							}
							else
							{
								HAL_UART_Transmit(&huart1, (uint8_t *)c,  16, 100);
							}
							if(recevie_data->data1 == CXD3554_I2C_ADDRESS)
							{
								MemAddSize =	I2C_MEMADD_SIZE_16BIT;
							}
							else
							{
								MemAddSize = I2C_MEMADD_SIZE_8BIT;
							}
							HAL_I2C_Mem_Write(&hi2c,
																recevie_data->data1,
																recevie_data->data2,
																MemAddSize,
																(uint8_t *)(&recevie_data->data3),
																1,
																300);
					}
					break;
					case A100_I2C_READ:
					{
							if(recevie_data->data0 == 1)
							{
								hi2c = hi2c1;
							}
							else if(recevie_data->data0 == 2)
							{
								hi2c = hi2c2;
							}
							else
							{
								HAL_UART_Transmit(&huart1, (uint8_t *)c,  16, 100);
							}
							if(recevie_data->data1 == CXD3554_I2C_ADDRESS)
								MemAddSize =	I2C_MEMADD_SIZE_16BIT;
							else
								MemAddSize = I2C_MEMADD_SIZE_8BIT;

							HAL_I2C_Mem_Read(&hi2c,
																recevie_data->data1,
																recevie_data->data2,
																MemAddSize,
																&I2cBuf,
																1,
																1000);
							HAL_I2C_Mem_Read(&hi2c,
																recevie_data->data1,
																recevie_data->data2,
																MemAddSize,
																&I2cBuf,
																1,
																1000);
							recevie_data->data3 = I2cBuf;
					}
				case A100_SET_INT_PATTERN_TYPE:
					switch(recevie_data->data0)
					{
								case INTERNAL_PATTERN_RASTER:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_RASTER);
								break;

								case INTERNAL_PATTERN_WINDOW:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_WINDOW);
								break;

								case INTERNAL_PATTERN_VERTICAL_stripe:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_VERTICAL_stripe);
								break;

								case INTERNAL_PATTERN_HORIZONTAL_STRIPE:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_HORIZONTAL_STRIPE);
								break;

								case INTERNAL_PATTERN_CROSSHATCH:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_CROSSHATCH);
								break;

								case INTERNAL_PATTERN_DOT:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_DOT);
								break;

								case INTERNAL_PATTERN_CROSSHATCH_DOT:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_CROSSHATCH_DOT);
								break;

								case INTERNAL_PATTERN_HRAMP:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_HRAMP);
								 break;

								case INTERNAL_PATTERN_VRAMP:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_VRAMP);
								 break;

								case INTERNAL_PATTERN_FRAME:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_FRAME);
								 break;

								case INTERNAL_PATTERN_CHECKBOX:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_CHECKBOX);
								 break;

								case INTERNAL_PATTERN_COLORBAR:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0x01);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, 0x71);
								 break;
							}
					break;

					case A100_SET_SOURCE_INPUT:
					switch(recevie_data->data0)
					{
						case SOURCE_TEST_PATTERN1:
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0x00);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, 0x76);
						break;

						case SOURCE_TEST_PATTERN2:
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0x00);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, 0x71);//colorbar
						break;

						case SOURCE_CUSTOMER_INPUT:								
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0x00);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, (0x70 + recevie_data->data1));
						break;
						
						case SOURCE_EXTERNAL_INPUT:
						default:
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0x00);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, 0x00);
						break;
					}
					
					break;

					case A100_GET_VERSION:
						recevie_data->data0 = VERSION0;
						recevie_data->data1 = VERSION1;
						recevie_data->data2 = VERSION2;
					break;

					case A100_SET_KST:
						if(recevie_data->data0 < 11)
						{
							uint16_t index;
							g_projector_para.keystone = recevie_data->data0;
							index = g_projector_para.keystone * 11 + g_projector_para.side_keystone;
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40, 0x6d);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A41, 0x68);
							for(i = 0; i<16 ;i++)
							{
								A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A49 + i, g_kst[index][i]);
							}
							recevie_data->data3 = index;
						}
						else
						{
							recevie_data->data3 = 101;
						}

						g_projector_para.keystone_valid = 1;

					break;

					case A100_SET_SIDE_KST:
						if(recevie_data->data0 < 11)
						{
							uint16_t index;
							g_projector_para.side_keystone = recevie_data->data0;
							index = g_projector_para.keystone * 11 + g_projector_para.side_keystone;
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40, 0x6d);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A41, 0x68);
							for(i = 0; i<16 ;i++)
							{
								A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A49 + i, g_kst[index][i]);
							}
							recevie_data->data3 = index;
						}
						else
						{
							recevie_data->data3 = 132;
						}
						g_projector_para.side_keystone_valid = 1;
					break;

					case A100_GET_KST:
						recevie_data->data0 = g_projector_para.keystone;
					break;

					case A100_GET_SIDE_KST:
						recevie_data->data0 = g_projector_para.side_keystone;
					break;

					case A100_GET_CURRENT_MODE:
						recevie_data->data0 = g_projector_para.projector_tuning.briness_index;
					break;

					case A100_GET_CCT_MODE:
						recevie_data->data0 = g_projector_para.projector_tuning.cct_index;
					break;

					case A100_SET_HORIZONTAL:
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0090, (H_FLIP_Mode)recevie_data->data0);
						g_projector_para.hflip = (H_FLIP_Mode)recevie_data->data0;
						g_projector_para.hflip_valid = 0x01;
					break;

					case A100_SET_VERTICAL:
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0091, (V_FLIP_Mode)recevie_data->data0);
						g_projector_para.vflip = (V_FLIP_Mode)recevie_data->data0;
						g_projector_para.vflip_valid = 0x01;
					break;

					case A100_GET_HORIZONTAL:
						recevie_data->data0 = g_projector_para.hflip;
					break;

					case A100_GET_VERTICAL:
						recevie_data->data0 = g_projector_para.vflip;
					break;

					case A100_SET_LED_INPUT:
					switch((Led_type)recevie_data->data0)
					{
								case RED_LED_ON:
									A100_SetRedCurrent(recevie_data->data1);
									A100_SetGreenCurrent(0);
									A100_SetBlueCurrent(0);
									recevie_data->data3 = 102;
								break;			

								case RED_LED1_ON:

									recevie_data->data3 = 103;
								break;

								case RED_LED2_ON:

									recevie_data->data3 = 104;
								break;

								case GREEN_LED_ON:
									A100_SetRedCurrent(0);
									A100_SetGreenCurrent(recevie_data->data1);
									A100_SetBlueCurrent(0);
									recevie_data->data3 = 105;
								break;

								case GREEN_LED1_ON:

									recevie_data->data3 = 106;
								break;

								case GREEN_LED2_ON:

									recevie_data->data3 = 107;
								break;

								case BLUE_LED1_ON:

								break;

								case BLUE_LED_ON:
									A100_SetRedCurrent(0);
									A100_SetGreenCurrent(0);
									A100_SetBlueCurrent(recevie_data->data1);
									recevie_data->data3 = 108;
								break;

								case LED_ON:
									g_projector_para.red_current = recevie_data->data1;
									g_projector_para.green_current = recevie_data->data2;
									g_projector_para.blue_current = recevie_data->data3;
									g_projector_para.brightness = 0x00;
									g_projector_para.current_valid = 0x01;
									g_projector_para.projector_tuning.tuning_valid = 0;
									
									printf("set red:%d green:%d blue:%d \r\n",recevie_data->data1,recevie_data->data2,recevie_data->data3);
									A100_SetRedCurrent(recevie_data->data1);
									A100_SetGreenCurrent(recevie_data->data2);
									A100_SetBlueCurrent(recevie_data->data3);
																	
									recevie_data->data5 = 109;
								break;

								case ALL_LED_ON:
								default:
									A100_SetRGBCurrent();
								break;
					 }
					break;

					case A100_GET_ADC_TEMP:
							value0 = adc_GetLDTemp();
							recevie_data->data0 =  value0 >> 16;
							recevie_data->data1 =  value0;
							recevie_data->data2 = 114;
					break;

					case A100_GET_CURRENT:
							recevie_data->data0 =  g_RedCurrent;
							recevie_data->data1 =  g_GreenCurrent;
							recevie_data->data2 =  g_BlueCurrent;
							recevie_data->data3 = 115;
							//A100_Lcos_CheckRegError(0);
					break;

					case A100_SET_DISPLAY_ON:
							A100_display_on(recevie_data->data0);
							recevie_data->data3 = 116;
					break;

					case A100_SET_MOTOR_START:
						Motor_start(recevie_data->data0, recevie_data->data1);
						break;					

					case A100_GET_BRITNRSS_NUM:
						recevie_data->data0 = A100_BRITNRSS_NUM;
					break;

					case A100_GET_KST_NUM:
						recevie_data->data0 = A100_KST_NUM;
					break;

					case A100_GET_SIDE_KST_NUM:
						recevie_data->data0 = A100_SIDE_KST_NUM;
					break;

					case A100_GET_DISPLAY_SIZE_NUM:
						recevie_data->data0 = A100_DISPLAY_SIZE_NUM;
					break;

					case A100_GET_HORIZONTAL_NUM:
						recevie_data->data0 = A100_HORIZONTAL_NUM;
					break;

					case A100_GET_VERTICAL_NUM:
						recevie_data->data0 = A100_VERTICAL_NUM;
					break;
					
					case A100_GET_CCT_NUM:
						recevie_data->data0 = A100_CCT_NUM;
					break;

					case A100_GET_LED_ON_OFF:
						recevie_data->data0 = gpiopin;
					break;		
					
					case A100_GET_ADC_PS:
							value0 = adc_GetPsOut();
							recevie_data->data0 =  value0 >> 16;
							recevie_data->data1 =  value0;
							recevie_data->data2 = 124;
					break;		

					case A100_GET_ADC_TS:
							value0 = adc_GetTsOut();
							recevie_data->data0 =  value0 >> 16;
							recevie_data->data1 =  value0;
							recevie_data->data2 = g_fan_value;
					break;		

					case A100_GET_TUNING_PARAMETER:
						HAL_UART_Transmit(&huart1, (uint8_t *)(&(g_projector_para.projector_tuning)), sizeof(struct Projector_CCT), 1000);
					break;

					case A100_SET_R_GAIN:
					{
						uint8_t val;
						val = recevie_data->data0;
						if(val<=100)
						{
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3216, val+155);
							g_projector_para.r_gain = val+155 ;
						}
						else
						{
							recevie_data->data5 = 0xEE;
						}
						recevie_data->data3 = 125;
						break;
					}
					case A100_SET_G_GAIN:
					{
						uint8_t val;
						val = recevie_data->data0;
						if(val<=100)
						{
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3217, recevie_data->data0+155);
							g_projector_para.g_gain = val+155 ;
						}
						else
						{
							recevie_data->data5 = 0xEE;
						}
						recevie_data->data3 = 126;
						break;
					}
					case A100_SET_B_GAIN:
					{
						uint8_t val;
						val = recevie_data->data0;
						if(val<=100)
						{
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3218, recevie_data->data0+155);
							g_projector_para.b_gain = val+155 ;
						}
						else
						{
							recevie_data->data5 = 0xEE;
						}
						recevie_data->data3 = 127;
						break;
					}
					case A100_GET_R_GAIN:
					{
						uint8_t val;
						A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x3216, &val);

						recevie_data->data0 = val-155;
						recevie_data->data1 = (unsigned short)g_projector_para.r_gain;
						recevie_data->data2 = (char)g_projector_para.r_gain;
						recevie_data->data3 = (unsigned short)g_projector_para.r_gain;
						recevie_data->data4 = sizeof(struct Projector_parameter);
						break;
					}
					case A100_GET_G_GAIN:
					{
						uint8_t val;
						A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x3217, &val);
						recevie_data->data0 = val-155 ;
						recevie_data->data3 = 129;
						break;
					}
					case A100_GET_B_GAIN:
					{
						uint8_t val;
						A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x3218, &val);
						recevie_data->data0 = val-155;
						recevie_data->data3 = 130;
						break;
					}	
					case A100_READ_GAMA:
					{
						uint8_t reg_val;
						uint8_t reg_addr;
						
						reg_addr = 0x58 + recevie_data->data0;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr, &reg_val);
						recevie_data->data0 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 1, &reg_val);
						recevie_data->data1 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 2, &reg_val);
						recevie_data->data2 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 3, &reg_val);
						recevie_data->data3 = reg_val;
						break;
					}
					case A100_WRITE_GAMA:
					{
						uint8_t reg_val;
						uint8_t reg_addr;
						reg_addr = 0x58 + recevie_data->data0;
						reg_val = recevie_data->data1;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr, reg_val);
						reg_addr = reg_addr + 4;
						reg_val = recevie_data->data2;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr, reg_val);
						break;
					}
					case A100_GET_GAMA:
					{
						uint8_t reg_val;
						uint8_t reg_addr;
						
						reg_addr = 0x58 + 8 * recevie_data->data0;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr, &reg_val);
						recevie_data->data0 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 1, &reg_val);
						recevie_data->data1 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 2, &reg_val);
						recevie_data->data2 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 3, &reg_val);
						recevie_data->data3 = reg_val;
						
						break;
					}
					case A100_SET_GAMA:
					{
						uint8_t reg_val;
						uint8_t reg_addr;

						reg_addr = 0x58 + 8 * recevie_data->data0;
						reg_val = recevie_data->data1;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr, reg_val);
						reg_val = recevie_data->data2;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 1, reg_val);
						reg_val = recevie_data->data3;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 2, reg_val);
						//reg_val = recevie_data->data4;
						//A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 3, reg_val);
						reg_val = 255 - recevie_data->data1;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 4, reg_val);
						reg_val = 255 - recevie_data->data2;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 5, reg_val);
						reg_val = 255 - recevie_data->data3;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 6, reg_val);
						//reg_val = 255 - recevie_data->data4;
						//A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 7, reg_val);
						break;
					}
					case A100_READ_CXD:
					{
						uint8_t  reg_val;
						uint16_t reg_addr;
						reg_addr = recevie_data->data0;
						A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, reg_addr, &reg_val);
						recevie_data->data0 = reg_val;
						break;
					}
					case A100_WRITE_CXD:
					{
						uint8_t  reg_val;
						uint16_t reg_addr;
						reg_addr = recevie_data->data0;
						reg_val  = recevie_data->data1;
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, reg_addr, reg_val);
						break;
					}
					case A100_SET_IPG:
					{
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xf2);
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, 0x00);
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, 0x01);
						break;
					}
					case A100_GET_WC:
					{
						if((recevie_data->data0)<4)
						{
							recevie_data->data1 = wc_data_temp[recevie_data->data0].power_num;
							recevie_data->data2 = wc_data_temp[recevie_data->data0].gain;
							recevie_data->data3 = wc_data_temp[recevie_data->data0].offset;
							recevie_data->data4 = wc_data_temp[recevie_data->data0].l;
						}
						else
						{
							recevie_data->data1 = 0xff;
							recevie_data->data2 = 0xff;
							recevie_data->data3 = 0xff;
							recevie_data->data4 = 0xff;
						}
						break;
					}
					case A100_SET_WC_SAVE:
					{
							memcpy(g_projector_para.wc,wc_temp,438);
							memcpy(g_projector_para.wc_data,wc_data_temp,sizeof(struct Projector_WC)*4);
							g_projector_para.wc_valid = 0x02;
						break;
					}
					case A100_GET_GAMA_PARAMETER:
					{
							//for(i=0;i<88;i++) printf("reg[%d]=0x%x\r\n",i+58, g_projector_para.gamma_data.gamma_reg[i]);
							HAL_UART_Transmit(&huart1, (uint8_t *)&g_projector_para.gamma_data,  sizeof(struct Projector_Gama), 100);
							return;
					}
					case A100_SET_PARAMRTER:
					{
							A100_setparameter(&g_projector_para);
							break;
					}
					case A100_DUMP_CXD3554:
					{
							A100_Dump_Cxd3554(recevie_data->data0, recevie_data->data1);
							break;
					}
					
					default:
					break;
				}
				recevie_data->checksum = UartReceiveLength;
				HAL_UART_Transmit(&huart1, (uint8_t *)recevie_data,  sizeof(struct Projector_date), 100);
		}
	else
	{
		int para0_len,para1_len,read_len;
		unsigned char temp;
		
		memset(&receive_buffer,0,40);
		para0_len = 0;
		para1_len = 0;
		memcpy(receive_buffer,pRx,UartReceiveLength);
		read_len = read_evm(receive_buffer,UartReceiveLength,&para0_len,&para1_len);
		para_Analysis((char*)(&receive_buffer),para0_len,para1_len);
		if(asu_rec_data.w_r == 1)
		{
			if(asu_rec_data.chip_addr == CXD3554_I2C_ADDRESS)
			{
/*				
						uint16_t temp_value;
						temp_value = (asu_rec_data.reg_value+1)*4-1;
						if(asu_rec_data.reg_addr==0x3216)
						{
							A100_SetRedCurrent(temp_value);
							g_red_value	= asu_rec_data.reg_value;
						}
						else if(asu_rec_data.reg_addr==0x3217)
						{
							A100_SetGreenCurrent(temp_value);
							g_green_value	= asu_rec_data.reg_value;
						}
						else if(asu_rec_data.reg_addr==0x3218)
						{
							A100_SetBlueCurrent(temp_value);
							g_blue_value	= asu_rec_data.reg_value;
						}
						else
*/
						{
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);
						}
			}
			else if (asu_rec_data.chip_addr == SXRD241_I2C_ADDRESS)
			{
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);
			}
			memset(&asu_rec_data,0x00,sizeof(struct asu_date));
		}

		if(asu_rec_data.w_r == 2)
		{
			if(asu_rec_data.chip_addr == CXD3554_I2C_ADDRESS)
			{
				A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);
			}
			else
				A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);
		}

		else if(asu_rec_data.w_r == 5)
		{
			if(asu_rec_data.chip_addr == CXD3554_I2C_ADDRESS)
			{
/*				
					uint16_t temp_value;
					temp_value = (asu_rec_data.reg_value+1)*4-1;
					if(asu_rec_data.reg_addr==0x3216)
					{
						A100_SetRedCurrent(temp_value);
						g_red_value	= asu_rec_data.reg_value;
					}
					else if(asu_rec_data.reg_addr==0x3217)
					{
						A100_SetGreenCurrent(temp_value);
						g_green_value	= asu_rec_data.reg_value;
					}
					else if(asu_rec_data.reg_addr==0x3218)
					{
						A100_SetBlueCurrent(temp_value);
						g_blue_value = asu_rec_data.reg_value;
					}
					else
*/
					{
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);			
					}
			}
			else if (asu_rec_data.chip_addr == SXRD241_I2C_ADDRESS)
			{
			      A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);
			}
			memset(&asu_rec_data,0x00,sizeof(struct asu_date));
		}
		if((asu_rec_data.w_r == 3) || (asu_rec_data.w_r == 4))
		{

			if(asu_rec_data.chip_addr == CXD3554_I2C_ADDRESS)
			{
/*				
						if(asu_rec_data.reg_addr==0x3216)
						{
							asu_rec_data.reg_value = g_red_value;
						}
						else if(asu_rec_data.reg_addr==0x3217)
						{
							asu_rec_data.reg_value = g_green_value;
						}
						else if(asu_rec_data.reg_addr==0x3218)
						{
							asu_rec_data.reg_value = g_blue_value;
						}
						else
*/				
						{
							A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);
						}
			}
			else if (asu_rec_data.chip_addr == SXRD241_I2C_ADDRESS)
			{
							A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);
			}
		}
		show_A100_data(&asu_rec_data);
		if((receive_buffer[0]=='R') ||(receive_buffer[0]=='r'))
		{
			if (receive_buffer[2] =='0')
			{
				HAL_UART_Transmit(&huart1, (uint8_t *)(&R_0), 15, 100);
			}
			else if(receive_buffer[2] =='1')
			{
				HAL_UART_Transmit(&huart1, (uint8_t *)(&R_1), 13, 100);
			}
			else if(receive_buffer[2] =='2')
			{
				HAL_UART_Transmit(&huart1, (uint8_t *)(&R_2), 15, 100);
			}
			else if (((receive_buffer[2] == '9') && (receive_buffer[3] == '2'))|| ((receive_buffer[2] == '8') && (receive_buffer[3] == 'c')) )
			{

				for(i = 0; i < 8 ;i++)
				{
					receive_buffer[read_len-2+i] = OK_0[i];
				}

				receive_buffer[read_len+4] = 32;
				if(data_len == 1)
				{
					temp = asu_rec_data.reg_value;
					receive_buffer[read_len+5] = HexToChar(temp);
				}

				if(data_len == 2)
				{
					temp = asu_rec_data.reg_value;
					receive_buffer[read_len+5] = HexToChar(temp/16);
					receive_buffer[read_len+6] = HexToChar(temp%16);
				}

				if(data_len == 4)
				{
					temp = asu_rec_data.reg_value >>8;
					receive_buffer[read_len+5] = HexToChar(temp/16);
					receive_buffer[read_len+6] = HexToChar(temp%16);
					temp = asu_rec_data.reg_value & 0xff;
					receive_buffer[read_len+7] = HexToChar(temp/16);
					receive_buffer[read_len+8] = HexToChar(temp%16);
				}
				receive_buffer[read_len -4 +8 +1 + data_len+1] = 13;
				receive_buffer[read_len -4 +8 +1 + data_len+2] = 10;
				HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len -2 + 8 +1 + data_len+2, 100);
				memset(&asu_rec_data,0x00,sizeof(struct asu_date));
			}
			else if ((receive_buffer[2] == '8') && ((receive_buffer[3] == '7') ||(receive_buffer[3] == 'd')) && (flag_0 != 0))
			{
				for(i=0;i<10;i++)
				{
					if(flag_0 == '0')
						receive_buffer[read_len -2 + i] = OK_1[i];
					if(flag_0 == '2')
						receive_buffer[read_len -2 + i] = OK_3[i];
					if(flag_0 == '6')
						receive_buffer[read_len -2 + i] = OK_7[i];
				}
				flag_0 = 0 ;
				HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len+8, 100);
			}
			else
			{
				for(i = 0; i < 10 ;i++)
					receive_buffer[read_len -2 + i] = OK_1[i];
				HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len+8, 100);
			}
		}
		else if (receive_buffer[0] == 'W' || receive_buffer[0] == 'w')
		{
			if((receive_buffer[2] == '8') && ((receive_buffer[3] == '7') || (receive_buffer[3] == 'd')) && (receive_buffer[5] == '1'))
			{
				flag_0 = '0' ;
			}
			else if((receive_buffer[2] == '8') && ((receive_buffer[3] == '7') || (receive_buffer[3] == 'd')) && (receive_buffer[5] == '7'))
			{
				flag_0 = '6' ;
			}
			else if((receive_buffer[2] == '8') && ((receive_buffer[3] == '7') || (receive_buffer[3] == 'd')) && (receive_buffer[5] == '3'))
			{
				flag_0 = '2' ;
			}
			for(i = 0; i < 8 ;i++)
				receive_buffer[read_len-2+i] = OK_0[i];
				HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len+6, 100);
		}

		else if(((receive_buffer[0]=='p') && (receive_buffer[1]=='r')) || ((receive_buffer[0]=='h') && (receive_buffer[1]=='r')) || ((receive_buffer[0]=='m') && (receive_buffer[1]=='r')) )
		{
			for(i = 0; i < 8 ;i++)
				receive_buffer[read_len-2+i] = OK_0[i];

			receive_buffer[read_len+4] = 32;
			if(data_len == 1)
			{
				temp = asu_rec_data.reg_value;
				receive_buffer[read_len+5] = HexToChar(temp);
			}
			if(data_len == 2)
			{
				temp = asu_rec_data.reg_value;
				receive_buffer[read_len+5] = HexToChar(temp/16);
				receive_buffer[read_len+6] = HexToChar(temp%16);
			}
			if(data_len == 4)
			{
				temp = asu_rec_data.reg_value >>8;
				receive_buffer[read_len+5] = HexToChar(temp/16);
				receive_buffer[read_len+6] = HexToChar(temp%16);
				temp = asu_rec_data.reg_value & 0xff;
				receive_buffer[read_len+7] = HexToChar(temp/16);
				receive_buffer[read_len+8] = HexToChar(temp%16);
			}
			receive_buffer[read_len -4 +8 +1 + data_len+1] = 13;
			receive_buffer[read_len -4 +8 +1 + data_len+2] = 10;
			HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len -2 + 8 +1 + data_len+2, 100);
			memset(&asu_rec_data,0x00,sizeof(struct asu_date));
		}

		else if(((receive_buffer[0]=='p') && (receive_buffer[1]=='w')) || ((receive_buffer[0]=='h') && (receive_buffer[1]=='w')) || ((receive_buffer[0]=='m') && (receive_buffer[1]=='w')) )
		{
			for(i = 0; i < 8 ;i++)
				receive_buffer[read_len-2+i] = OK_0[i];
			HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len+6, 100);
			memset(&asu_rec_data,0x00,sizeof(struct asu_date));
		}
	}
}

void A100_ReceiveUart1Data(void)
{
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)UartReceiveRxBuffer, UART_BUFFER_MAX_SIZE);	
}

void HAL_UART_AbortReceiveCpltCallback (UART_HandleTypeDef *huart)
{

	A100_UartCmdHandler((uint8_t *)UartReceiveRxBuffer,UartReceiveLength);
	UartReceiveLength = 0;

	//Re-start receiving
	A100_ReceiveUart1Data();
	/* NOTE : This function should not be modified, when the callback is needed,
	the HAL_UART_AbortTransmitCpltCallback can be implemented in the user file.
	*/
}	
/* lcos ------------------------------------------------------------------*/

void A100_LcosSetGain(void)
{
		if((unsigned char)g_projector_para.r_gain<155)
			g_projector_para.r_gain = 255;
		if((unsigned char)g_projector_para.g_gain<155)
			g_projector_para.g_gain = 255;
		if((unsigned char)g_projector_para.b_gain<155)
			g_projector_para.b_gain = 255;

		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3216, (unsigned char)g_projector_para.r_gain);
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3217, (unsigned char)g_projector_para.g_gain);
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3218, (unsigned char)g_projector_para.b_gain);
}

void A100_LcosSetGamma(void)
{
	  int i;

		printf("gamma_valid=0x%x\r\n",g_projector_para.gamma_data.gamma_valid);
		if((g_projector_para.gamma_data.gamma_valid) == 0xFDDF)
		{
			for(i=0; i<88; i++) {
				A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, 0x58+i, g_projector_para.gamma_data.gamma_reg[i]);
				printf("reg[%d]=0x%x\r\n",i+58, g_projector_para.gamma_data.gamma_reg[i]);
			}
		}
		
#if 0		
		uint8_t reg_data;
		for(i=0; i<88; i++) {
			A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, 0x58+i, &reg_data);
			printf("reg[%d]=%d\r\n",i+58, reg_data);
		}	
#endif			
}

void A100_LcosSetFlip(void)
{
	A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0090, 0x01);
	A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0091, 0x01);

	if(g_projector_para.vflip_valid == 0x01)
	{
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0091, (V_FLIP_Mode)(g_projector_para.vflip));
	}
	if(g_projector_para.hflip_valid == 0x01)
	{
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0090, (H_FLIP_Mode)(g_projector_para.hflip));	
	}
}

void A100_LcosSetKst(void)
{
		uint32_t i;
		uint16_t index;

		if((g_projector_para.keystone < 11)&&(g_projector_para.side_keystone<11))
		{
			;
		}
		else
		{
			g_projector_para.keystone = 5;
			g_projector_para.side_keystone =5;
		}		
		index = g_projector_para.keystone * 11 + g_projector_para.side_keystone;
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40, 0x6d);
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A41, 0x68);
		for(i = 0; i<16 ;i++)
		{
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A49 + i, g_kst[index][i]);
		}
}

void A100_LcosSetWC(void)
{
	  int i;

		if((g_projector_para.wc_valid) == 0x02)
		{
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40,0x6d);
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A42,0x30);
			for(i=0; i<438; i++)
			{ 
					A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS,0x3500 +i , g_projector_para.wc[i]);
			}
		}
		else
		{
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40,0x6d);
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A42,0x30);
			for(i=0; i<438; i++)
			{ 
				  g_projector_para.wc[i] = 0x00;
					A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS,0x3500 +i , g_projector_para.wc[i]);
			}
			memset(g_projector_para.wc_data,0x00,sizeof(struct Projector_WC)*4);
		}
		memcpy(wc_temp,g_projector_para.wc,438);
		memcpy(wc_data_temp,g_projector_para.wc_data,sizeof(struct Projector_WC)*4);	
}
/* tim ------------------------------------------------------------------*/
uint8_t A100_SetFan12Speed(uint32_t speed)
{
  if(speed > FAN_SPEED_FULL)	speed = FAN_SPEED_FULL;
	
  TIM_OC_InitTypeDef sConfigOC={0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = speed;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	
  return 0;
}

uint8_t A100_SetFan34Speed(uint32_t speed)
{
  if(speed > FAN_SPEED_FULL)	speed = FAN_SPEED_FULL;

  TIM_OC_InitTypeDef sConfigOC={0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = speed;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  return 0;
}
uint8_t A100_SetFan5Speed(uint32_t speed)
{
  if(speed > FAN_SPEED_FULL)	speed = FAN_SPEED_FULL;

  TIM_OC_InitTypeDef sConfigOC={0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = speed;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

  return 0;
}
/* adc ------------------------------------------------------------------*/
#define ADC_CONVERSION_CNT  10
#define ADC_CHANNEL_CNT  3
#define ADC_CONVERSION_NUM 	ADC_CHANNEL_CNT * ADC_CONVERSION_CNT

/* Variable to report status of DMA transfer of ADC group regular conversions */
/*  0: DMA transfer is not completed                                          */
/*  1: DMA transfer is completed                                              */
__IO   uint8_t ubDmaTransferStatus = 0; /* Variable set into DMA interruption callback */

/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  hadc: ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Update status variable of DMA transfer */
  ubDmaTransferStatus = 1;  

}

uint16_t get_ADC_DmaValue(uint16_t ch)
{
	uint16_t aResultDMA[ADC_CONVERSION_NUM], ret, i;
	uint16_t count = 1000;
	static uint8_t flag_cal = 1;
	uint32_t val_sum = 0;
	
	memset(aResultDMA, 0, sizeof(aResultDMA));
	
	if(flag_cal) {
		if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
			printf("HAL_ADCEx_Calibration_Start faild. ret=%d \r\n", ret);
		}
		flag_cal = 0;
	}
	
	/* ###- Start conversion in DMA mode ################################# */
	ret = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)aResultDMA, ADC_CONVERSION_NUM);
	if (ret != HAL_OK)
	{
		printf("HAL_ADC_Start_DMA faild. ret=%d \r\n", ret);
	}

	/* waiting convert complete */
	while(count && !ubDmaTransferStatus) {
		count--;
	}
#if 0
	printf("count=%d \r\n", count);
	if(ubDmaTransferStatus == 1) {
		for (i=0; i<ADC_CONVERSION_NUM; i++) {
			printf("aResultDMA[%d]=%d \r\n",i, aResultDMA[i]);
		}
	}
#endif
	/* Update status variable of DMA transfer */
	ubDmaTransferStatus = 0;
	HAL_ADC_Stop(&hadc1);	
	
	for (i=0; i<ADC_CONVERSION_CNT; i++)
		val_sum += aResultDMA[i * ADC_CHANNEL_CNT + ch];

	return (val_sum/ADC_CONVERSION_CNT);
}

uint32_t adc_GetTsOut(void)
{
	return get_ADC_DmaValue(2);
}

uint32_t adc_GetPsOut(void)
{
	return get_ADC_DmaValue(1);
}

uint32_t adc_GetLDTemp(void)
{
	return get_ADC_DmaValue(0);
}

void A100_Variables_Init(void)
{
	memset(&asu_rec_data,0x00,sizeof(struct asu_date));	
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
