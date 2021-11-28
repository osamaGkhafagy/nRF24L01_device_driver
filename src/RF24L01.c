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

#include "RF24L01.h"


void power_led_on()
{
	PWR_LED_PORT |= (1<<POWER_LED);
}

void power_led_off()
{
	PWR_LED_PORT &= ~(1<<POWER_LED);
}

void transmission_led_on()
{
	TXRX_LED_PORT |= (1<<TRANSMISSION_LED);
}

void transmission_led_off()
{
	TXRX_LED_PORT &= ~(1<<TRANSMISSION_LED);
}

void blink_transmission_led(uint8_t blinks)
{
	for(uint8_t i = 0; i < blinks; i++)
	{
		transmission_led_on();
		_delay_ms(50);
		transmission_led_off();
		_delay_ms(50);
	}
}

void blink_power_led(uint8_t blinks)
{
	for(uint8_t i = 0; i < blinks; i++)
	{
		power_led_on();
		_delay_ms(50);
		power_led_off();
		_delay_ms(50);
	}
}


uint8_t NRF_init(bool mode, uint8_t spi_speed, uint8_t air_data_rate, uint8_t payload_width)
{
    /*
     * This function initializes nRF24L01 devices
     * Parameters:
     * ------------
     * mode:            PTX or PRX
     * spi_speed:       1, 2, 4Mbps
     * air_data_rate:   1Mbps or 2Mbps
     * payload_width:   Either fixed with specified number of bytes. e.g. FIXED_PYLD(8) for 8 bytes payload
     *                  or DYNAMIC_PYLD for any number of bytes
     * Returns:
     * ------------
     * error:           0 for no error
     *                  1 if no nRF24L01 device connected
     *                  2 if some registers cannot be configured

    */
	spi_init(MASTER, SPEED_MBPS(4), 0, 0);
	// configure chip enable pin
	CE_DDR |= (1<<CE);
	CE_PORT	&= ~(1<<CE);
	// initialize serial terminal
	USART_init(BAUD_RATE(19200), 8, false);
	// configure indicators
	PWR_LED_DDR |= (1<<POWER_LED);
	TXRX_LED_DDR |= (1<<TRANSMISSION_LED);
	// set communication mode to either PTX or PRX
	if(mode == PRX_MODE)
	{
		while(!NRF_write_register_w_check_by_name("CONFIG", (NRF_read_register_by_name("CONFIG") | (1<<PRIM_RX))));
		// enter RX mode
		CE_PORT |= (1 << CE);
	}
	// setting data rate to air_data_rate
	if(air_data_rate == AIR_DR_1MBPS)
		while(!NRF_write_register_w_check_by_name("RFSTUP", 0x07));
	else
		// If air_data_rate = 2MBPS
		while(!NRF_write_register_w_check_by_name("RFSTUP", 0x0F));
	// setting payload width
	if(payload_width == DYNAMIC_PYLD)
	{
		NRF_ACTIVATE_cmd();
		// Enable dynamic payload length on all pipes
		while(!NRF_write_register_w_check_by_name("DYNPLD", 0X3F));
		// Enable dynamic payload length
		while(!NRF_write_register_w_check_by_name("FEATURE", (1<<EN_DPL)));
	}
	else
	{
		// set pipe 0 width to payload_width
		while(!NRF_write_register_w_check(RX_PW_P0, payload_width));
		// set pipe 1 width to payload_width
		while(!NRF_write_register_w_check(RX_PW_P1, payload_width));
	}
	// set Shock Burst parameters (autoack, autoret)
	// enable acknowledgment after packet transmission
	NRF_write_register_by_name("ENAACK", 0x3F);
	// Set the auto-retransmission delay (ARD) in SETUP_RETR register
	// setting auto-retransmission count initially to 3
	while(!NRF_write_register_w_check(SETUP_RETR, ((ARD << 4) & ARC)));

	// clear interrupts
	NRF_clear_interrupts();
	if ((NRF_read_register_by_name("STATUS") & ((1<<TX_DS) | (1<<RX_DR) | (1<<MAX_RT))) == 0X00)
		USART_writeln_string("All Interrupts are cleared!");
	
	// interrupt enabled configuration
	#ifdef INTERRUPT_ENABLED
	// Configuring microcontroller interrupt on INT0
	// EIMSK set to 0x01 (Enable INT0)
	EIMSK |= (1<<INT0);
	// EICRA set to 10 (falling edge on INT0 generates interrupt)
	EICRA &= ~(1<<ISC00);
	EICRA |= (1<<ISC01);
	// SREG set to 0x80 (Enable global interrupt)
	SREG |= (1<<7);
	// Configuring NRF interrupt
	// by default, NRF devices reflect interrupts on IRQ pin

	#endif
	// power-up nrf24l01 and check if it's powered-up
	while(!NRF_write_register_w_check_by_name("CONFIG", (NRF_read_register_by_name("CONFIG") | (1<<PWR_UP) | (1<<EN_CRC) | (1<<CRC0))));
	_delay_ms(2);
	// indicate that nrf24l01 powered on and entered standby mode
	blink_power_led(3);
	power_led_on();
}

uint8_t NRF_write_payload(uint8_t* payload_holder, uint8_t len)
{
	uint8_t n_missed_packets = 0;
	// Ensure that TX_DS, RX_DR and MAX_RT are not set
	NRF_clear_interrupts();
	#ifdef SERIAL_INTERFACE
	USART_writeln_string("Sending Data Wirelessly!");
	#endif
	if(len > CHUNK_SIZE)
	{
		// initially reset counter
		counter = 0;	
		uint8_t data_chunks = floor(len / CHUNK_SIZE);
		int index = 0;
		uint8_t  n_missed_packets = 0;
		char data_str[3];
		for(uint8_t i = 0; i < data_chunks; i++)
		{
			index = i * 32;
			// sending regular chunks
			NRF_write_payload_chunk((payload_holder + index), CHUNK_SIZE);
			#ifndef INTERRUPT_ENABLED
			NRF_clear_interrupts();
			#endif
			counter++;
		}
		// now sending the rest of data
		index += CHUNK_SIZE;
		NRF_write_payload_chunk((payload_holder + index), (len % CHUNK_SIZE));
	}
	else
	{
		NRF_write_payload_chunk(payload_holder, len);
		// delay for 1 ms cautiously
		_delay_ms(1);
	}
	NRF_flush_tx();
	#ifndef INTERRUPT_ENABLED
	NRF_clear_interrupts();
	#endif
	#ifdef SERIAL_INTERFACE
	n_missed_packets = get_missed_packets();
	if(n_missed_packets > 0)
	{
		char str_holder[3];
		USART_write_string("There ");
		USART_write_string(int_to_str(n_missed_packets, str_holder, 10));
		USART_writeln_string(" packets not received at RX end!");
	}
	#endif
	// indicate success of transmission if n_missed_packets = 0
	if(n_missed_packets == 0)
		blink_transmission_led(5);
	return n_missed_packets;
}
// Write data chunk to TX FIFO
uint8_t NRF_write_payload_chunk(uint8_t* payload_chunk_holder, uint8_t len)
{
	// make sure the given data chunk is < 32 bytes
	if(len > 32)
	{
		#ifdef SERIAL_INTERFACE
		USART_writeln_string("The given data chunk is > 32 bytes!");
		#endif
		return 0;
	} 
	uint16_t total_retr_delay = ARC * (250 + 250 * ARD);
	spi_start_connection();
	spi_exchange_byte(W_TX_PAYLOAD);
	for(uint8_t i = 0; i < len; i++)
	{
		spi_exchange_byte(*(payload_chunk_holder + i));
	}
	spi_stop_connection();
	// enter TX mode 
	CE_PORT |= (1 << CE);
	// wait for at least 10us and 130us for TX settling
	_delay_ms(1);
	// enter standby I mode 
	CE_PORT &= ~(1 << CE);
	// wait for retransmission to happen ARC times, each (ARD+250) microseconds
	
	#ifdef INTERRUPT_ENABLED
	_delay_us(total_retr_delay);
	#else
	// wait until packet is transmitted or max retransmission is reached
	while(!NRF_isSent() & !NRF_isMaxRT());
	if(NRF_isSent())
		return 1;
	else if(NRF_isMaxRT())
		return 0;
	#endif
	
}

bool NRF_read_payload(uint8_t* payload_holder)
{
	// initially, clear rx_pyld holder
	clear_str(rx_pyld, CHUNK_SIZE + 1);
	#ifdef SERIAL_INTERFACE
	char rx_pld_str[3];
	USART_writeln_string("Reading received Data...");
	#endif
	#ifdef SERIAL_INTERFACE
	itoa(pyld_width, rx_pld_str, 10);
	USART_write_string("Found ");
	USART_write_string(rx_pld_str);
	USART_writeln_string(" Bytes!");
	#endif
	NRF_read_data(R_RX_PAYLOAD, payload_holder, pyld_width);
	// clear RX_DR
	NRF_clear_interrupts();
	// acknowledge that data is received
	return true;
}

#ifdef SERIAL_INTERFACE
void NRF_serial_interface(uint8_t rx_pld[CHUNK_SIZE], uint8_t tx_pld[CHUNK_SIZE])
{
	USART_writeln_string("**********************");
	received_command[9] = 0;
	char data_str[10];
	char command_holder[BYTES_NUM + 1] = {"\0"};
	// receive the command using USART
	received_command = receive_command(command_holder);
	// show the received value
	USART_write_string("Received Command: ");
	USART_writeln_string(received_command);
	// split the command into three parts: command type, register name, register value
	split_command(received_command, &command_type, register_name, &data);
	// show the split parts
	USART_write_string("the split parts: ");
	USART_write_data(command_type); USART_write_string(", ");
	USART_write_string(register_name); USART_write_string(", ");
	int_to_str(data, data_str, 'd');
	USART_writeln_string(data_str);
	USART_writeln_string("---------------------");

	// assuring that the register name is correct
	if(get_reg_addr(register_name) == 0XFF)
	{
		USART_write_string("register ");
		USART_write_string(register_name);
		USART_writeln_string("is invalid!");
	}
		
	// writing only commands
	if(command_type == 'W')
	{
		USART_writeln_string("Register Writing Attempt!");
		USART_writeln_string("command_type = 'W'");
		if((get_reg_addr(register_name) != 0xFF) & abs((strcmp(register_name, "TXADDR")))
		& (abs(strcmp(register_name, "RXP0AD"))) & abs((strcmp(register_name, "RXP1AD"))))
		{
			NRF_write_register_by_name(register_name, data);
		}

		else if(!strcmp(register_name, "FLUSHT"))
		{
			USART_writeln_string("Flushing TX Buffer!");
			spi_send_byte(FLUSH_TX);
		}
		else if(!strcmp(register_name, "FLUSHR"))
		{
			USART_writeln_string("Flushing RX Buffer!");
			spi_send_byte(FLUSH_RX);
		}
		else if(!strcmp(register_name, "NOP000"))
		{
			USART_writeln_string("Sending NOP!");
			spi_send_byte(NOP);
			USART_write_string("STATUS Register content: ");
			int_to_str(spi_get_buffer(), data_str, 'h');
			USART_writeln_string(data_str);
		}
		else if(!strcmp(register_name, "TXPYLD"))
		{
			NRF_write_payload((uint8_t*)data, 1);
		}
		else if(!strcmp(register_name, "TXADDR"))
		{
			NRF_write_tx_addr(tx_pld);
		}
		else if(!strcmp(register_name, "RXP0AD"))
		{
			NRF_write_tx_addr(tx_pld);
		}
		else if(!strcmp(register_name, "RXP1AD"))
		{
			NRF_write_tx_addr(tx_pld);
		}
	}
	// writing and reading commands
	if(command_type == 'R')
	{
		USART_writeln_string("Register Reading Attempt!");
		if((get_reg_addr(register_name) != 0xFF) & (abs((strcmp(register_name, "TXADDR")))) 
			& abs((strcmp(register_name, "RXP0AD"))) & (abs((strcmp(register_name, "RXP1AD")))))
		{
			USART_write_string("Register Content(hex): ");
			int_to_str(NRF_read_register_by_name(register_name), data_str, 'h');
			USART_writeln_string(data_str);
		}
		else if(!strcmp(register_name, "RXPLDW"))
		{
			USART_writeln_string("Reading RX Payload Width!");
			USART_write_string("RX Payload Width(bytes): ");
			int_to_str(NRF_read_byte(R_RX_PL_WID), data_str,'d');
			USART_writeln_string(data_str);
		}
		else if(!strcmp(register_name, "RXPYLD"))
		{
			NRF_read_payload(rx_pld);
		}
		else if(!strcmp(register_name, "TXADDR"))
		{
			uint8_t tx_addr_holder[5];
			NRF_read_tx_addr(tx_addr_holder);
			NRF_display_data(tx_addr_holder, 5, 2);
		}
		else if(!strcmp(register_name, "RXP0AD"))
		{
			uint8_t rx_addr_holder[5];
			NRF_read_rx_addr(rx_addr_holder);
			NRF_display_data(rx_addr_holder, (data <= 1? 5: 1), 2);
		}
		else if(!strcmp(register_name, "RXP1AD"))
		{
			uint8_t rx_addr_holder[5];
			NRF_read_rx_addr(rx_addr_holder);
			NRF_display_data(rx_addr_holder, (data <= 1? 5: 1), 2);
		}

	}

}
#endif
void NRF_write_register(uint8_t reg_addr, uint8_t data)
{
	NRF_write_data(W_REGISTER(reg_addr), &data, 1);
}

bool NRF_write_register_w_check(uint8_t reg_addr, uint8_t data)
{
	NRF_write_register(reg_addr, data);
	if(NRF_read_register(reg_addr) == data)
		return true;
	else
		return false;
}

void NRF_write_register_by_name(char* reg_name, uint8_t data)
{
	NRF_write_data(W_REGISTER(get_reg_addr(reg_name)), &data, 1);
}

bool NRF_write_register_w_check_by_name(char* reg_name, uint8_t data)
{
	NRF_write_register_by_name(reg_name, data);
	if((NRF_read_register_by_name(reg_name) == data) | (!strcmp(reg_name, "STATUS")))
	{
		#ifdef SERIAL_INTERFACE
		char data_str[10];
		USART_write_string("Register ");
		USART_write_string(reg_name);
		USART_write_string(" Successfully loaded with ");
		int_to_str(data, data_str, 'h');
		USART_writeln_string(data_str);
		#endif
		return true;
	}
	else
	{
		#ifdef SERIAL_INTERFACE
		USART_write_string("Writing register ");
		USART_write_string(reg_name);
		USART_writeln_string(" failed!");
		#endif
		return false;
	}
}

uint8_t NRF_read_register(uint16_t reg_addr)
{
	return NRF_read_byte(R_REGISTER(reg_addr));
}

uint8_t NRF_read_register_by_name(char* reg_name)
{
	return NRF_read_byte(R_REGISTER(get_reg_addr(reg_name)));
}

uint8_t NRF_get_pyld_width()
{
	uint8_t rx_pld_width = 0;
	#ifdef SERIAL_INTERFACE
	USART_writeln_string("Reading RX Payload Width!");
	#endif
	rx_pld_width = NRF_read_byte(R_RX_PL_WID);
	if(rx_pld_width == 0)	return 0;
	return rx_pld_width;
}

#ifdef SERIAL_INTERFACE
void NRF_display_data(uint8_t data_sequence[], uint8_t sequence_width, uint8_t formats)
{
	// formats: decimal, hex, ascii (1: decimal, 2: decimal and hex, 3: decimal, hex and ascii)
	char reps[3][10] = {"Decimal", "Hex", "ASCII"};
	char data_str[5];
	USART_writeln_string("Data Sequence: ");

	for(uint8_t k = 0; k < 3; k++)
	{
		if(k < formats)
		{
			USART_write_string(reps[k]);
			USART_writeln_string(": ");
		}
		for(int8_t m = (sequence_width - 1); m > -1; m--)
		{
			if(k <= 1)
			{
				if(k < 1)
					int_to_str(data_sequence[m], data_str, 'd');
				else if((k == 1) & (formats >= 2))
					int_to_str(data_sequence[m], data_str, 'h');
				USART_write_string(data_str);
			}
			else if((k > 1) & (formats == 3))
				USART_write_data(data_sequence[m]);
			if(k < formats)
				USART_write_string(" ");
		}
		if(k < formats)
			USART_writeln_string("");
	}
}
#endif

void NRF_read_tx_addr(uint8_t* tx_addr)
{
	NRF_read_data(R_REGISTER(get_reg_addr(register_name)), tx_addr, 5);
}

void NRF_read_rx_addr(uint8_t* rx_addr)
{

	//if(pipe_no == 0 | pipe_no == 1)
	NRF_read_data(R_REGISTER(get_reg_addr(register_name)), rx_addr, 5);
	//else
	{
		// for future development
		// including pipes 2 to 5
	}
}

void NRF_write_tx_addr(uint8_t* tx_addr)
{
	NRF_write_data(W_REGISTER(get_reg_addr(register_name)), tx_addr, 5);
}

void NRF_write_rx_addr(uint8_t* rx_addr)
{
	NRF_write_data(W_REGISTER(get_reg_addr(register_name)), rx_addr, 5);
}

// sending command only to NRF24l01
void NRF_write_byte(uint8_t command)
{
	spi_send_byte(command);
}
// sending a command and byte(s) to the NRF24l01
void NRF_write_data(uint8_t command, uint8_t* data, uint8_t data_length)
{
	if(data_length == 1)
	{
		spi_start_connection();
		spi_exchange_byte(command);
		spi_exchange_byte(*data);
		spi_stop_connection();
	}
	else
	{
		spi_start_connection();
		spi_exchange_byte(command);
		for(uint8_t i = 0; i < data_length; i++)
		{
			spi_exchange_byte(data[i]);
		}
		spi_stop_connection();
	}
}

// sending command to receive one byte of data
uint8_t NRF_read_byte(uint8_t command)
{
	spi_start_connection();
	spi_exchange_byte(command);
	spi_exchange_byte(0x00);
	spi_stop_connection();
	return spi_get_buffer();
}
// sending command to receive multiple bytes
void NRF_read_data(uint8_t command, uint8_t* data_holder, uint8_t data_length)
{
	spi_start_connection();
	spi_exchange_byte(command);
	spi_flush_buffer();
	for(uint8_t i = 0; i < data_length; i++)
	{
		spi_exchange_byte(0);
		data_holder[i] = spi_get_buffer();
	}
	spi_stop_connection();
}
// receive a command through USART
char* receive_command(char* command_holder)
{
	return USART_read_string(command_holder, BYTES_NUM);
}

void NRF_clear_interrupts()
{
	// to reset an interrupt flag, we must set this flag to 1
	// so whatever we get of fags, we write them again, so they get resetted
	NRF_write_register(STATUS, NRF_read_register(STATUS));
}

uint8_t	get_reg_addr(char* reg_name)
{
	if(!strcmp(reg_name, "CONFIG"))
	{
		//USART_writeln_string("CONFIG Register Targeted!");
		return CONFIG;
	}
	if(!strcmp(reg_name, "ENAACK"))
	{
		//USART_writeln_string("ENAACK Register Targeted!");
		return EN_AA;
	}
	else if(!strcmp(reg_name, "ENRXAD"))
	{
		//USART_writeln_string("ENRXAD Register Targeted!");
		return EN_RXADDR;
	}
	else if(!strcmp(reg_name, "ADWDTH"))
	{
		//USART_writeln_string("ADWDTH Register Targeted!");
		return SETUP_AW;
	}
	else if(!strcmp(reg_name, "AUTRET"))
	{
		//USART_writeln_string("AUTRET Register Targeted!");
		return SETUP_RETR;
	}
	else if(!strcmp(reg_name, "RFSTUP"))
	{
		//USART_writeln_string("RFSTUP Register Targeted!");
		return RF_SETUP;
	}
	else if(!strcmp(reg_name, "STATUS"))
	{
		//USART_writeln_string("STATUS Register Targeted!");
		return STATUS;
	}
	else if(!strcmp(reg_name, "RXP0AD"))
	{
		//USART_writeln_string("RXP0AD Register Targeted!");
		return RX_ADDR_P0;
	}
	else if(!strcmp(reg_name, "RXP1AD"))
	{
		//USART_writeln_string("RXP1AD Register Targeted!");
		return RX_ADDR_P1;
	}
	else if(!strcmp(reg_name, "P0WDTH"))
	{
		//USART_writeln_string("P0WDTH Register Targeted!");
		return RX_PW_P0;
	}
	else if(!strcmp(reg_name, "P1WDTH"))
	{
		//USART_writeln_string("P0WDTH Register Targeted!");
		return RX_PW_P1;
	}
	else if(!strcmp(reg_name, "TXADDR"))
	{
		//USART_writeln_string("TXADDR Register Targeted!");
		return TX_ADDR;
	}
	else if(!strcmp(reg_name, "FIFOST"))
	{
		//USART_writeln_string("FIFOST Register Targeted!");
		return FIFO_STATUS;
	}
	else if(!strcmp(reg_name, "DYNPLD"))
	{
		//USART_writeln_string("FIFOST Register Targeted!");
		return DYNPD;
	}
	else if(!strcmp(reg_name, "FEATURE"))
	{
		//USART_writeln_string("FIFOST Register Targeted!");
		return FEATURE;
	}
	else
	{
		//USART_writeln_string("Invalid Register Address!");
		return 0xFF;
	}
}

void NRF_flush_tx()
{
	NRF_write_byte(FLUSH_TX);
}

void NRF_flush_rx()
{
	NRF_write_byte(FLUSH_RX);
}

bool NRF_isSent()
{
	return (NRF_read_register_by_name("STATUS") & (1<<TX_DS));
}

bool NRF_isMaxRT()
{
	return (NRF_read_register_by_name("STATUS") & (1<<MAX_RT));
}

bool NRF_isReceived()
{
	return (NRF_read_register_by_name("STATUS") & (1<<RX_DR));
}

bool NRF_isAvailable()
{
	return reception_flag;
}

void NRF_ACTIVATE_cmd()
{
	spi_start_connection();
	spi_exchange_byte(ACTIVATE);
	spi_exchange_byte(0x73);
	spi_stop_connection();
}

ISR(INT0_vect)
{
	// check which interrupt has got you here
	if(NRF_isReceived())
	{
		// once a payload is received, set reception_flag to 1
		reception_flag = 1;
		pyld_width = NRF_get_pyld_width();
		#ifdef SERIAL_INTERFACE
		USART_writeln_string("Packet received!");
		#endif
		NRF_read_payload(rx_pyld);
	}
	else
	{
		if(NRF_isSent())
		{
			trans_flags[counter] = 0;
			#ifdef SERIAL_INTERFACE
			USART_writeln_string("Packet sent!");
			#endif
		}
		if(NRF_isMaxRT())
		{
			trans_flags[counter] = 1;
			#ifdef SERIAL_INTERFACE
			USART_writeln_string("Packet timed out!");
			#endif
		} 
	}
	NRF_clear_interrupts();
	// clear INTO flag
	if(EIFR & (1 << INTF0))
		EIFR |= (1 << INTF0);
	
	USART_writeln_string("------------------------");
}
// helper functions
void split_command(char* cmd, char* cmd_type, char* reg_name, int* data)
{
    *cmd_type = cmd[0];
    for(uint8_t i = 0; i < 6; i++)
    {
        cmd++;
        reg_name[i] = *cmd;
    }
    reg_name[6] = 0;
    cmd++;
    *data = hexstring_to_int(cmd);
}

int hexstring_to_int(char* data)
{
	int lower, upper;
	lower = data[1];
	upper = data[0];

	(lower > 57) ? (lower -= 55) : (lower -= 48);
	(upper > 57) ? (upper -= 55) : (upper -= 48);

	return (lower * pow(16, 0) + upper * pow(16, 1));
}

char* int_to_str(int data, char* str_holder,  char base)
{
	itoa(data, str_holder, base == 'd'? 10: 16);
	return str_holder;
}

uint8_t get_missed_packets()
{
	uint8_t count = 0;
	for(uint8_t i = 0; i < MAX_CHUNKS; i++)
	{
		if(trans_flags[i] == 1)
			count++;
			trans_flags[i] = 0;
	}
	return count;
}

void clear_str(char* str, uint8_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
		str[i] = '\0';
	}
}

