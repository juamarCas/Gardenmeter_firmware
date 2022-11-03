#ifndef _ADC_H
#define _ADC_H

#include <cstdint>
#include "stm32f3xx.h"
#include "Utils.h"

#define ADC_CR_ADVERGEN_ENABLE  (0x1U << 28U)
#define ADC_CR_ADVERGEN_DISABLE (0x2U << 28U)
#define ADC_CR_ADVERGEN_CLEAR   (0x3U << 28U)

/*ADC channel number macros*/
#define ADC_CH_1 (0x1U)
#define ADC_CH_2 (0x2U)
#define ADC_CH_3 (0x3U)
#define ADC_CH_4 (0x4U)
#define ADC_CH_5 (0x5U)

/*Position of each sequence bits of the sqr register macros*/
#define ADC_SQ1_1 (0x06U)
#define ADC_SQ1_2 (0x0CU)
#define ADC_SQ1_3 (0x12U)
#define ADC_SQ1_4 (0x18U)

/*Register of sqr macros*/
#define ADC1_SQR1 0x50000030U
#define ADC1_SQR2 0x50000034U
#define ADC1_SQR3 0x50000038U
#define ADC1_SQR4 0x5000003CU

/*number of conversions macros*/
#define ADC_LENGTH_1 (0x0U << 0x0U)
#define ADC_LENGTH_2 (0x1U << 0x0U)
#define ADC_LENGTH_3 (0x2U << 0x0U) 
#define ADC_LENGTH_4 (0x3U << 0x0U) 
#define ADC_LENGTH_5 (0x4U << 0x0U) 
#define ADC_LENGTH_6 (0x5U << 0x0U) 
#define ADC_LENGTH_7 (0x6U << 0x0U) 


namespace periph{
	namespace ADC{
		
		void Calib(ADC_TypeDef * adc); 
		std::uint16_t Get_Value(ADC_TypeDef * adc); 
		/*single conversion*/
		void Init(ADC_TypeDef * adc, std::uint8_t length = ADC_LENGTH_1);
		void DMA_Init(ADC_TypeDef * adc, DMA_Channel_TypeDef * dma_ch,std::uint16_t * buffer, std::uint32_t buffer_length ,std::uint8_t length = ADC_LENGTH_1);
		/**
		*Set a adc sequence and channel
		*@param adc_sqr refers to the register, look at Register of sqr macros to see the options
		*@param channel, refers to ADC channel, look at  ADC channel number
		*@param sequece, sequence number of that channel of the adc refer to position of each sequence bits of the sqr register
		*/
		void SetChannelSequence(std::uint32_t adc_sqr, std::uint32_t channel, std::int32_t sequence); 
		void Config_Sequence(ADC_TypeDef * adc, std::uint32_t Sequence); 
		void Disable(ADC_TypeDef * adc); 
		void Enable(ADC_TypeDef * adc); 
		void Enable_regulator(ADC_TypeDef * adc); 
	
	}
}

#endif