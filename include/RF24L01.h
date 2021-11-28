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

#ifndef __RF24L01__
#define __RF24L01__

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <avr/interrupt.h>
#include "spi.h"
#include "serial_328p.h"
/*------nRF24L01 Commands-------*/
#define		NOP						0XFF
#define		R_REGISTER(REG_ADDR)	(0X00 | REG_ADDR)
#define		W_REGISTER(REG_ADDR)	(0X20 |	REG_ADDR)
#define		R_RX_PAYLOAD			0X61
#define		W_TX_PAYLOAD			0XA0
#define		FLUSH_TX				0XE1
#define		FLUSH_RX				0XE2
#define		R_RX_PL_WID				0X60
#define		W_TX_PAYLOAD_NOACK		0XB0
#define 	ACTIVATE				0X50
/*------nRF24L01 Register Map----*/
#define		CONFIG					0X00
#define		EN_AA					0x01
#define		EN_RXADDR				0X02
#define		SETUP_AW				0X03
#define		SETUP_RETR				0X04
#define		RF_SETUP				0X06
#define		STATUS					0X07
#define     OBSERVE_TX              0X08
#define		RX_ADDR_P0				0X0A
#define		RX_ADDR_P1				0X0B
#define		RX_ADDR_P2				0X0C
#define		RX_ADDR_P3				0X0D
#define		RX_ADDR_P4				0X0E
#define		RX_ADDR_P5				0X0F
#define		TX_ADDR					0X10
#define		RX_PW_P0				0X11
#define		RX_PW_P1				0X12
#define		RX_PW_P2				0X13
#define		RX_PW_P3				0X14
#define		RX_PW_P4				0X15
#define		RX_PW_P5				0X16
#define		FIFO_STATUS				0X17
#define		DYNPD					0X1C
#define		FEATURE					0X1D

/*-------------Individual bits-----------*/
#define MASK_RX_DR				6
#define MASK_TX_DS				5
#define MASK_MAX_RT				4
#define EN_CRC                  3
#define CRC0                    2
#define PWR_UP					1
#define PRIM_RX					0
#define ERX_P0                  0
#define ERX_P1                  1
#define ERX_P2					2
#define ERX_P3					3
#define ERX_P4					4
#define ERX_P5					5
#define AW0						0
#define AW1						1
#define RF_DR					3
#define RF_PWR0					1
#define RF_PWR1					2
#define TX_DS					5
#define RX_DR					6
#define MAX_RT					4
#define TX_FULL					0
#define TX_REUSE				6
#define TX_EMPTY				4
#define RX_FULL					1
#define RX_EMPTY				0
#define DPL_P5					5
#define DPL_P4					4
#define DPL_P3					3
#define DPL_P2					2
#define DPL_P1					1
#define DPL_P0					0
#define EN_DPL					2
#define EN_ACK_PAY				1
#define EN_DYN_ACK				0
/*--------------------------------*/
#define BYTES_NUM				10
#define DYNAMIC_PYLD            0
#define ARD                     15
#define ARC                     15
#define FIXED_PLD(x)            x
#define AIR_DR_1MBPS            1
#define AIR_DR_2MBPS            2
#define	PAYLOAD_WIDTH			1
#define MAX_CHUNKS              10
#define CHUNK_SIZE              32
#define PTX_MODE				0
#define PRX_MODE				1
#define CTRL_PORT				PORTC
#define CE_DDR					DDRB
#define	CE_PORT					PORTB
#define	CE						1
#define PWR_LED_DDR			    DDRB
#define PWR_LED_PORT            PORTB
#define TXRX_LED_DDR		    DDRD
#define TXRX_LED_PORT			DDRD
#define POWER_LED				0
#define	TRANSMISSION_LED		7

#define INTERRUPT_ENABLED

void power_led_on();
void power_led_off();
void transmission_led_on();
void transmission_led_off();
void blink_transmission_led(uint8_t blinks);
void blink_power_led(uint8_t blinks);


/* High level functions */
uint8_t NRF_init(bool mode, uint8_t spi_speed, uint8_t air_data_rate, uint8_t payload_width);
uint8_t NRF_write_payload(uint8_t* payload_holder, uint8_t len);

bool NRF_read_payload(uint8_t* payload_holder);
void NRF_serial_interface(uint8_t rx_payload[32], uint8_t tx_payload[32]);
/*-------------------*/
void NRF_write_register(uint8_t reg_addr, uint8_t data);
bool NRF_write_register_w_check(uint8_t reg_addr, uint8_t data);
void NRF_write_register_by_name(char* reg_name, uint8_t data);
bool NRF_write_register_w_check_by_name(char* reg_name, uint8_t data);
uint8_t NRF_read_register(uint16_t reg_addr);
uint8_t NRF_read_register_by_name(char* reg_name);
uint8_t NRF_get_pyld_width();
void NRF_display_data(uint8_t data_sequence[], uint8_t sequence_width, uint8_t formats);
void NRF_read_tx_addr(uint8_t* tx_address);
void NRF_read_rx_addr(uint8_t* rx_address);
void NRF_write_tx_addr(uint8_t* tx_address);
void NRF_write_rx_addr(uint8_t* rx_address);
void NRF_write_byte(uint8_t command);
void NRF_write_data(uint8_t command, uint8_t* data, uint8_t data_length);
uint8_t NRF_write_payload_chunk(uint8_t* payload_chunk_holder, uint8_t len);
uint8_t NRF_write_large_payload(uint8_t command, uint8_t* large_data, uint8_t data_length); // deprecated
uint8_t NRF_read_byte(uint8_t command);
void NRF_read_data(uint8_t command, uint8_t* data_holder, uint8_t data_length);
char* receive_command();
void NRF_clear_interrupts();
uint8_t	get_reg_addr(char* reg_name);
void NRF_flush_tx();
void NRF_flush_rx();
bool NRF_isSent();
bool NRF_isMaxRT();
bool NRF_isReceived();
bool NRF_isAvailable();
void NRF_ACTIVATE_cmd();
// helper functions
void split_command(char* cmd, char* cmd_type, char* reg_name, int* data);
int hexstring_to_int(char* data);
char* int_to_str(int data, char* str_holder, char base);
uint8_t get_missed_packets();
void clear_str(char* str, uint8_t len);

uint8_t rx_pyld[100];
volatile uint8_t pyld_width;
// reception flag must be zeroed manually after reading data from rx_pyld
volatile bool reception_flag;
/*
 * variables dedicated to the serial interface mode
 */
char received_value_str[4], data_str[4];
volatile uint8_t received_value;
char* received_command;
char register_name[7];
volatile char command_type;
// 100 is arbitrary for any number of data chunks
bool trans_flags[MAX_CHUNKS];
// counter used to track index of data chunk to be able to identify which
// packet has been MAX_RTed and which is successfully sent
volatile uint8_t counter;
volatile int data;
#endif
