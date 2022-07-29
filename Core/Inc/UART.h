#ifndef _UART_H
#define _UART_H

#include <cstdint>
#include "stm32f3xx.h"
#include "GPIO.h"
namespace periph{
	namespace UART{
		/*
		* Initializates the selected UART from the cmsis lib
		* @param uart, USART structure you want to use (ex: USART1, USART2 etc)
		* @param baud, baud rate of the communication (ex: 9600)
		* @param freq, frequency which your system works (ex: 8000000 (8MHz))
		*/
		void Init(USART_TypeDef * uart, std::uint32_t baud, std::uint32_t freq); 
		/*
		* Send a byte of data
		* @param uart, USART structure you want to use (ex: USART1, USART2 etc)
		* @param byte, 8 bits data you want to send (ex: 0x33, 'H')
		*/
		void send_byte(USART_TypeDef * uart, std::uint8_t byte);
		void send_byte(USART_TypeDef * uart, const char byte);		
		void write(USART_TypeDef * uart, const char * buffer, std::uint32_t buffer_size);	
		void print(USART_TypeDef * uart, std::uint8_t msg);
		void println(USART_TypeDef * uart, std::uint8_t msg);
		void write(USART_TypeDef * uart, std::uint8_t * buffer, std::uint32_t buffer_size);	
	
		/*DMA*/
		/*Using circular mode to send the same buffer size each time i send data*/
		void dma_tx_init_circular(USART_TypeDef * uart, std::uint32_t baud, std::uint32_t freq, DMA_Channel_TypeDef * DMA_CH,
		std::uint8_t * buffer, std::uint32_t buffer_length);
		void dma_write_cirular(DMA_TypeDef * DMA, DMA_Channel_TypeDef * DMA_CH); 
	}
	
}

#endif