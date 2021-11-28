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
#ifndef __SPI_H__
#define __SPI_H__

#define 	SPI_DDR				DDRB
#define		SPI_PORT			PORTB
#define		SS					2
#define		MOSI				3
#define		MISO				4
#define		SCK					5
#define     MASTER              0
#define     SLAVE               1
#define     SPEED_MBPS(x)       x
#define     bool                _Bool
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <math.h>

void spi_init(bool mode, uint8_t speed, bool clk_polarity, bool clk_phase);
void spi_start_connection();
void spi_exchange_byte(uint8_t data);
void spi_send_byte(uint8_t dataByte);
void spi_stop_connection();
uint8_t spi_get_buffer();
void spi_flush_buffer();
#endif
