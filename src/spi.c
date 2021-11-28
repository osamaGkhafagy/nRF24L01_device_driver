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
#include "spi.h"

void spi_init(bool mode, uint8_t speed, bool clk_polarity, bool clk_phase)
{
	/*
     * This function initializes the SPI modeule for atmega328p mcu
     * Parameters:
     * ------------
     * mode:            0 for Master, 1 for Slave
     * speed:           1, 2, 4Mbps(MAX = F_CPU/2)
     * clk_polarity:    Specifies the clock polarity(or the clock base PGT or NGT ).0 for Rising Edge, 1 for Falling Edge.
	 * clk_phase : Specifies the clock phase(or the sampling instance).0 for the leading edge, 1 for the trailing edge.
	 *
     *
     */
	//Enable SPIF interrupt
	//sei();
	//SPCR|=(1<<SPIE);
	//Set the clock polarity
	clk_polarity ? (SPCR |= (1<<CPOL)) : (SPCR &= ~(1<<CPOL));
	//Set the clock phase
	clk_phase ? (SPCR |= (1<<CPHA)) : (SPCR &= ~(1<<CPHA));
	//Set the I/O drivers based on the SPI mode
	if(mode == MASTER)
	{
		//Master mode
		SPCR |= (1<<MSTR);
		SPI_DDR |= (1<<MOSI)|(1<<SCK)|(1<<SS);
		SPI_DDR &= ~(1<<MISO);
	}
	else
	{
		//Slave mode
		SPI_DDR |= (1<<MISO);     //MISO as OUTPUT
		SPI_DDR &= ~(1<<SCK)|(1<<SS);
	}
    uint8_t clock_division_factor = (uint8_t)lrint((F_CPU/1000000.0) / speed);
	switch(clock_division_factor)
	{
		case 2:	SPSR |= (1<<SPI2X);
				SPCR &= ~(1<<SPR0)|(1<<SPR1);
				break;
		case 4:
				SPSR &= ~(1<<SPI2X);
				SPCR &= ~(1<<SPR0)|(1<<SPR1);
				break;
		case 8:
				SPSR |= (1<<SPI2X);
				SPCR |= (1<<SPR0);
				SPCR &= ~(1<<SPR1);
				break;
		case 16:
				SPSR &= ~(1<<SPI2X);
				SPCR |= (1<<SPR0);
				SPCR &= ~(1<<SPR1);
				break;
		case 32:
				SPSR |= (1<<SPI2X);
				SPCR &= ~(1<<SPR0);
				SPCR |= (1<<SPR1);
				break;
		case 64:
				SPSR &= ~(1<<SPI2X);
				SPCR &= ~(1<<SPR0);
				SPCR |= (1<<SPR1);
				break;
		case 128:
				SPSR &= ~(1<<SPI2X);
				SPCR |= (1<<SPR0)|(1<<SPR1);
				break;
		default:
				SPSR &= ~(1<<SPI2X)|(1<<SPR0)|(1<<SPR1);
	}
		//Enable the SPI module
		SPCR|=(1<<SPE);
}

void spi_start_connection()
{
	SPI_PORT &= ~(1<<SS);
}

void spi_exchange_byte(uint8_t data)
{
	SPDR=data;
	while(!(SPSR & (1<<SPIF)));
}

void spi_send_byte(uint8_t dataByte)
{
	spi_start_connection();
	spi_exchange_byte(dataByte);
	spi_stop_connection();
}

void spi_stop_connection()
{
	SPI_PORT |= (1<<SS);
}

uint8_t spi_get_buffer()
{
	return SPDR;
}

void spi_flush_buffer()
{
	SPDR = 0;
}

