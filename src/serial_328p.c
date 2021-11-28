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
#include "serial_328p.h"

void USART_init(uint32_t baud_rate, uint8_t frame_bits, bool double_speed)
{
    /*
     *  This function initializes the USART module of atmega328p device
     *  Parameters:
     *  -----------
     *  baud_rate:      2400, 4800, 9600, 19200, 38400, 115200,..., 250kbps 
     *  frame_bits:     5:9 bit frame
     *  double_speed:   true or false
     */
	//Configuring the UBRR Register Which Controls The Baud Rate Generator
	uint16_t UBRR_register;
	if (double_speed)
		UBRR_register = lrint( (F_CPU / (8.0 * baud_rate)) - 1 );
	else	
        UBRR_register = lrint( (F_CPU / (16.0 * baud_rate)) - 1 );
	
	UBRR0L = (uint8_t)(UBRR_register);
	UBRR0H = (uint8_t)(UBRR_register >> 8);
	//Enabling The USART Transmitter And Receiver
	UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
	
	//Setting Up The Character Size
	switch(frame_bits)
	{
		case 5: 
			UCSR0C = 0x80;
			UCSR0B &= ~(1<<UCSZ02);
			break;
		case 6: 
			UCSR0C = 0x82;
			UCSR0B &= ~(1<<UCSZ02);
			break;
		case 7: 
			UCSR0C = 0x84;
			UCSR0C |= (1<<UCSZ01);
			UCSR0B &= ~(1<<UCSZ02);
			break;
		case 8: 
			UCSR0C = 0x86;
			UCSR0B &= ~(1<<UCSZ02);
			break;
		case 9: 
			UCSR0C = 0x86;
			UCSR0B |= (1<<UCSZ02);
			break;
	}
	
}
uint8_t* USART_read_data(uint8_t* data, uint8_t n_bytes)
{
	for (int i=0;i<n_bytes;i++)
	{
		data[i]=USART_receive_byte();
	}
	return data;
}

char* USART_read_string(char* str_holder, uint8_t n_chars)
{
	// str_holder must be at least of length (n_chars+1)
	for (int i = 0; i < n_chars; i++)
	{
		str_holder[i] = USART_receive_byte();
	}
	str_holder[n_chars]='\0';
	return str_holder;
}

void USART_write_data(uint8_t data)
{
	USART_send_byte(data);
}

void USART_write_string(char* data)
{
	USART_send_string(data);
}


void USART_writeln_data(uint8_t data)
{
	USART_send_byte(data);
	USART_send_byte(NL);
}

void USART_writeln_string(char* data)
{
	USART_send_string(data);
	USART_send_byte(NL);
}

uint8_t USART_send_byte(uint8_t data)
{
	while(!(UCSR0A & (1 << UDRE0)));		//if transmission is in progress
	UDR0 = data;
	return 1;				//return 1 to indicate that transmission is completed
}

uint8_t USART_receive_byte()
{
	while (!(UCSR0A & ( 1<< RXC0)));			// no data in RX buffer
	return UDR0;
}

uint8_t USART_send_string(char* ch)
{
	while((*ch) > 0)
	{
		while(!(UCSR0A & ( 1<< UDRE0)));
		UDR0 = *ch;
		ch++;
	}
	
	return 1;
}

void USART_flush()
{
	while(!( UCSR0A & (1 << RXC0)));
	char dummy = UDR0;
}
