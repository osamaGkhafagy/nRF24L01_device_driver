/*
MIT License

Copyright (c) 2020 osamaGkhafagy <osamakhafagy55@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef __SERIAL_328P_H__
#define __SERIAL_328P_H__
#include <avr/io.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define NL              0x0A
#define BAUD_RATE(x)    x


void USART_init(uint32_t baud_rate, uint8_t frame_bits, bool double_speed);
uint8_t* USART_read_data(uint8_t* data, uint8_t n_bytes);
char* USART_read_string(char* str_holder, uint8_t n_chars);
void USART_write_data(uint8_t data);
void USART_writeln_data(uint8_t data);
void USART_write_string(char* data);
void USART_writeln_string(char* data);
uint8_t USART_send_byte(uint8_t);
uint8_t USART_receive_byte();
uint8_t USART_send_string(char*);
void USART_flush();

#endif
