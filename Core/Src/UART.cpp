#include "UART.h"

#define UART_TXE (1U << 7U)

namespace periph{
	namespace UART{
		void Init(USART_TypeDef * uart, std::uint32_t baud, std::uint32_t freq){
			const std::uint32_t baudrate = freq/baud; 
			uart->BRR = baudrate; 
			
			uart->CR1 |= USART_CR1_RE; 
			uart->CR1 |= USART_CR1_TE; 
			
			uart->CR3 = 0; 
			uart->CR1 |= USART_CR1_UE; 
		}
	
		void send_byte(USART_TypeDef * uart, std::uint8_t byte){
			while(!(uart->ISR & UART_TXE)){}
			uart->TDR = byte & 0xFF; 
		}

		void send_byte(USART_TypeDef * uart, const char byte){
 			while(!(uart->ISR & UART_TXE)){}
			uart->TDR = byte & 0xFF; 
		}	

		void write(USART_TypeDef * uart, const char * buffer, std::uint32_t buffer_size){
			for(int i = 0; i < buffer_size; i++){
				send_byte(uart, buffer[i]); 
			}
		}	
		
		void write(USART_TypeDef * uart, std::uint8_t * buffer, std::uint32_t buffer_size){
			for(int i = 0; i < buffer_size; i++){
				send_byte(uart, buffer[i]); 
			}
		}
	
	
		void dma_tx_init(USART_TypeDef * uart, std::uint32_t baud, std::uint32_t freq, DMA_Channel_TypeDef * DMA_CH,
			std::uint8_t * buffer, std::uint32_t buffer_length){
				const std::uint32_t baudrate = freq/baud; 
				uart->BRR = baudrate; 
			 
				uart->CR1 |= USART_CR1_TE; 
			
				uart->CR3 = 0; 
				
				DMA_CH->CNDTR = (std::uint16_t)buffer_length; 
				DMA_CH->CCR |= (DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_DIR); 
				DMA_CH->CCR |= (DMA_CCR_EN);
				DMA_CH->CCR |= DMA_CCR_TCIE; 
				
				DMA_CH->CPAR = (std::uint32_t)&uart->TDR; 
				DMA_CH->CMAR = (std::uint32_t)buffer; 
				
				uart->CR3 |= USART_CR3_DMAT; 
				uart->CR1 |= USART_CR1_UE;
		}
		void dma_write(DMA_TypeDef * DMA, DMA_Channel_TypeDef * DMA_CH){
			DMA_CH->CCR |= DMA_CCR_EN; 
			while((DMA1->ISR & DMA_ISR_TCIF2) == 0){}
			DMA->IFCR = DMA_IFCR_CTCIF2; 
			DMA_CH->CCR &= ~DMA_CCR_EN;
					
		}
	}
	
	
}