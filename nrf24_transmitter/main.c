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
#define F_CPU	8000000UL
#define INTERRUPT_ENABLED
#include "RF24L01.h"


int main()
{
	_delay_ms(100);
	NRF_init(PRX_MODE, SPEED_MBPS(4), AIR_DR_2MBPS, DYNAMIC_PYLD);
	while(1)
	{
		USART_writeln_string("Waiting for a packet...");
		while(reception_flag == 0);
		USART_writeln_string(rx_pyld);
		// once you read the payload from RX FIFO, set reception_flag to 0
		reception_flag = 0;
	}

}
