#include "ADC.h"

namespace periph{
	namespace ADC{
		void Init(ADC_TypeDef * adc, std::uint8_t length){
			if(length > 0xFU){
				//TODO: some error pls
			}
			
			adc->SQR1 = 0; 
			if( length != ADC_LENGTH_1){
				adc->SQR1 |= (length << 0U); 
			}
			Enable_regulator(adc); 
			Enable(adc); 
			Get_Value(adc); 
			Calib(adc); 
			Enable(adc); 
				
		}
		//This methods implement circular mode by default
		void DMA_Init(ADC_TypeDef * adc, DMA_Channel_TypeDef * dma_ch, std::uint16_t * buffer, std::uint32_t buffer_length,std::uint8_t length){
			adc->CFGR |= ADC_CFGR_CONT; 
			adc->CFGR |= ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
			adc->SMPR1 |= (7U << 3U) | (7U << 6U); //max sampling rate 
	
			
			dma_ch->CPAR = (std::uint32_t)(&adc->DR); 
			dma_ch->CMAR = (std::uint32_t)(buffer); 
			dma_ch->CNDTR = buffer_length; 	
			dma_ch->CCR |= DMA_CCR_EN;				
			
			Enable_regulator(adc); 
			Enable(adc); 
			Calib(adc); 
			Enable(adc); 
			
		}
		
		void SetChannelSequence(std::uint32_t adc_sqr, std::uint32_t channel, std::int32_t sequence){
			*reinterpret_cast<volatile std::uint32_t *>(adc_sqr) |= (channel << sequence); 
		}
		
		void Enable(ADC_TypeDef * adc){
			adc->CR |= ADC_CR_ADEN; 
			while(!(adc->ISR & ADC_ISR_ADRDY)){}
		}
		
		void Disable(ADC_TypeDef * adc){
			adc->CR |= ADC_CR_ADDIS; 
			while((adc->CR & ADC_CR_ADEN)){}
		}
		
		void Enable_regulator(ADC_TypeDef * adc){ 
			adc->CR &= ~(ADC_CR_ADVERGEN_CLEAR); 
			adc->CR |= ADC_CR_ADVERGEN_ENABLE; 
			utils::delay::us(25); 
		}
		
		void Calib(ADC_TypeDef * adc){
			Disable(adc); 
			adc->CR &= ~ADC_CR_ADCALDIF; 
			adc->CR |= ADC_CR_ADCAL; 
			
			while((adc->CR & ADC_CR_ADCAL) != 0){}
		}
		
		 std::uint16_t Get_Value(ADC_TypeDef * adc){
			adc->CR |= ADC_CR_ADSTART;
			while(!(adc->ISR & ADC_ISR_EOC)){}
				return (std::uint16_t)adc->DR; 
		}
	}
}