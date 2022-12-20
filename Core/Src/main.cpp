/*
* Firmware made by: Juan D. Martín 
* JDMC
*/
#include "main.h"
#include "GPIO.h"
#include "Utils.h"
#include "SHTC3.h"
#include "UART.h"
#include "ADC.h"
#include "Payload.h"
#include "State.h"


#define DMA_ADC_BUFFER_LENGTH 3
#define ADC_BUFFER_LENGTH DMA_ADC_BUFFER_LENGTH * 2
#define START_ADC_CONVERTION ADC1->CR |= ADC_CR_ADSTART
#define STOP_ADC_CONVERTION ADC1->CR |= ADC_CR_ADSTP

void SystemClock_Config(void);
static void MX_I2C1_Init(void);
void UART3_init(void);
void ConfigureI2CPins(periph::GPIO::config& scl_pin, periph::GPIO::config& sda_pin);
void ConfigUSARTPins(periph::GPIO::config& tx_pin, periph::GPIO::config& rx_pin);

I2C_HandleTypeDef hi2c1;

periph::GPIO * gpioc13 = new periph::GPIO();
SHTC3 _shtc3(&hi2c1);


std::uint16_t adc_dma_buffer[DMA_ADC_BUFFER_LENGTH] = {0,0,0};

void ConfigureTIMER3();

void State_MeasureSHTC3();
void State_ReadADC();
void State_SendData();
void State_InitTimer();

/*
structure of each element of array:
{
  currentState,
  NextState
}
*/
states sm_states[4] = {
  {
    &State_MeasureSHTC3,
    &sm_states[1]
  },
  {
    &State_ReadADC,
    &sm_states[2]
  },
  {
    &State_SendData,
    &sm_states[3]
  },
  {
    &State_InitTimer,
    &sm_states[0]
  }

};
states * states_ptr;
Payload _payload;

std::uint16_t moist_res;
std::uint16_t soilTemp_res;
std::uint16_t light_res;

bool finishedConvertion = false;
bool _takeData = false; 
std::uint16_t counter = 0;

bool _passNextState = true;
extern "C"{
  void DMA1_Channel1_IRQHandler(void){
    if(DMA1->ISR & DMA_ISR_TCIF1){
      STOP_ADC_CONVERTION;
       moist_res    = adc_dma_buffer[0];
       soilTemp_res = adc_dma_buffer[1];
       light_res    = adc_dma_buffer[2];
      _passNextState = true;
      DMA1->IFCR |= DMA_IFCR_CTCIF1;
      

    }
  }

  void  TIM3_IRQHandler(void){
    
    TIM3->SR &= ~TIM_SR_UIF;
    counter++;
    //counter ticks each second, so 3600 second each hour
    if(counter >= 5){
      TIM3->CR1 &= ~TIM_CR1_CEN;
      TIM3->DIER &= ~TIM_DIER_UIE;
      gpioc13->Toggle();
      _takeData = true;
      counter = 0;
      TIM3->CR1 &= ~TIM_CR1_CEN;
    }
  }

}


int main(void)
{
  __disable_irq();
  HAL_Init();
  SystemClock_Config();
  
  periph::GPIO::config gpioa0_config;
  periph::GPIO::config gpioa1_config;
  periph::GPIO::config gpioa2_config;
  
  periph::GPIO::config gpiob6_config;
  periph::GPIO::config gpiob7_config;

  periph::GPIO::config gpioc4_config;
  periph::GPIO::config gpioc5_config;
  periph::GPIO::config gpioc13_config;

  utils::delay::Init();

  //RCC configuration
  RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
  RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
	RCC->AHBENR  |= RCC_AHBENR_ADC12EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  //prescale the ADC clock dividing it by 2!
  ADC12_COMMON->CCR |= (1U << 17U);

  //gpio configuration
  gpioc13_config.mode = periph::GPIO::Mode::Output;
  gpioc13_config.gpio = GPIOC;
  gpioc13_config.pin  = 13;
  gpioc13->SetConfig(gpioc13_config);

  //ConfigureTIMER3();
  ConfigureI2CPins(gpiob6_config, gpiob7_config);
  ConfigUSARTPins(gpioc4_config, gpioc5_config);
 
  //pin configuration for the ADC
  gpioa0_config.mode = periph::GPIO::Analog;
  gpioa0_config.gpio = GPIOA;
  gpioa0_config.pin  = 0;

  gpioa1_config.mode = periph::GPIO::Analog;
  gpioa1_config.gpio = GPIOA;
  gpioa1_config.pin  = 1;

  gpioa2_config.mode = periph::GPIO::Analog;
  gpioa2_config.gpio = GPIOA;
  gpioa2_config.pin  = 2;

  periph::GPIO::SetGPIO(gpioa0_config);
  periph::GPIO::SetGPIO(gpioa1_config);
  periph::GPIO::SetGPIO(gpioa2_config);

  DMA1_Channel1->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0;
  DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;
  
  periph::ADC::SetChannelSequence(ADC1_SQR1, ADC_CH_1, ADC_SQ1_1);
  periph::ADC::SetChannelSequence(ADC1_SQR1, ADC_CH_2, ADC_SQ1_2);
  periph::ADC::SetChannelSequence(ADC1_SQR1, ADC_CH_3, ADC_SQ1_3);
  periph::ADC::DMA_Init(ADC1, DMA1_Channel1, adc_dma_buffer, DMA_ADC_BUFFER_LENGTH, ADC_LENGTH_3);
  //periph::ADC::Init(ADC1, ADC_LENGTH_3);
  MX_I2C1_Init();
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  // NVIC_EnableIRQ(TIM3_IRQn);
  // NVIC_SetPriority(TIM3_IRQn, 0);
  __enable_irq();

  //Sensor initialization
  _shtc3.begin();
  

  states_ptr = &sm_states[0];
  while (1)
  {
   // if(!_takeData) continue;
    (states_ptr->action_function)();
    while(!_passNextState){}
    states_ptr = states_ptr->next_state;
    
  }
  
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
}


static void MX_I2C1_Init(void) 
{


  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
 
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
   
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
   
  }

}

void ConfigUSARTPins(periph::GPIO::config& tx_pin, periph::GPIO::config& rx_pin){
  tx_pin.gpio       = GPIOC;
  tx_pin.pin        = 4;
  tx_pin.mode       = periph::GPIO::Mode::Alternate;
  tx_pin.afrl_value = 0x7U;
  tx_pin.afrl_bit_position   = 0x10U;

  rx_pin.gpio       = GPIOC;
  rx_pin.pin        = 5;
  rx_pin.mode       = periph::GPIO::Mode::Alternate;
  rx_pin.afrl_value = 0x7U;
  rx_pin.afrl_bit_position   = 0x14U;


  periph::GPIO::SetGPIO(tx_pin);
  periph::GPIO::SetGPIO(rx_pin);

  periph::UART::Init(USART1, 9600, 8000000);
}

void ConfigureI2CPins(periph::GPIO::config& scl_gpio, periph::GPIO::config& sda_gpio){
  scl_gpio.mode       = periph::GPIO::Mode::Alternate;
  scl_gpio.gpio       = GPIOB;
  scl_gpio.pin        = 6;
  scl_gpio.afrl_bit_position   = 0x18U;
  scl_gpio.afrl_value = 0x4U;

  sda_gpio.mode       = periph::GPIO::Mode::Alternate;
  sda_gpio.gpio       = GPIOB;
  sda_gpio.pin        = 7;
  sda_gpio.afrl_bit_position   = 0x1CU;
  sda_gpio.afrl_value = 0x4U;

  periph::GPIO::SetGPIO(scl_gpio);
  periph::GPIO::SetGPIO(sda_gpio);
}

void ConfigureTIMER3(){
  TIM3->DIER |= TIM_DIER_UIE;
  TIM3->PSC = 8000;
  TIM3->ARR = 1000;
  TIM3->CR1 |= TIM_CR1_CEN;
}

//STATES

void State_MeasureSHTC3(){
    float temp;
    float hum;
    if(_shtc3.Read_sensor(temp, hum, SHTC3_NORMAL_MEASUREMENT_CMD)){
      _payload.env_temp = temp;
      _payload.env_hum  = hum;
    }
}

void State_ReadADC(){
  _passNextState = false; 
  START_ADC_CONVERTION;  
}

void State_SendData(){

  //assuming linear behaviour in moist sensor and LDR
  //values taken by experimenting
  //1230 min moist pretty wet
  //2720 max pretty dry
  //light with 10k res 3900 most light
  //light with 10k res 1090 less light
  moist_res = moist_res > 2720 ? 2720: moist_res;

  _payload.id = 1;
  _payload.soil_temp = soilTemp_res;
  _payload.light     = (std::uint8_t)utils::math::map(light_res, 1090, 3900, 0, 100);
  _payload.moist     = (std::uint8_t)utils::math::map(moist_res, 1230, 2720, 100, 0);
  const char * data = reinterpret_cast<char*>(&_payload);
  periph::UART::write(USART1, data, sizeof(Payload));
}

void State_InitTimer(){
  // _takeData = false;
  // TIM3->CNT = 0;
  // TIM3->DIER |= TIM_DIER_UIE;
  // TIM3->CR1 |= TIM_CR1_CEN;
  utils::delay::ms(2000);
}
 

