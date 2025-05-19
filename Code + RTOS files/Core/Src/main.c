#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "math.h"

SPI_HandleTypeDef hspi1;

/* Definitions for taskWifi */
osThreadId_t taskWifiHandle;
const osThreadAttr_t taskWifi_attributes = {
  .name = "taskWifi",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for taskSensors */
osThreadId_t taskSensorsHandle;
const osThreadAttr_t taskSensors_attributes = {
  .name = "taskSensors",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};



void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void startTaskWifi(void *argument);
void startTaskSensors(void *argument);
 

uint16_t x,y,z;				//acceleration data storage.	
uint8_t data_rec[10]; //raw accwleration data

//all quatities to be sent

float vib = 0.0 , temp = 0.0 , snd = 0.0 , crnt = 0.0;
float peakVib = 0.0 , peakTemp = 0.0 , peakSnd = 0.0 , peakCrnt = 0.0;
int samp = 0;
float meanVib = 0.0 , meanTemp = 0.0 , meanSnd = 0.0 , meanCrnt = 0.0;

//functions for adxl345 working

void adxl_write (uint8_t address, uint8_t value);
void adxl_read (uint8_t address);
void adxl_init (void);
void dataCon(void);

//functons for uart and debug terminal 

void uartEnable(void);
void USART2_SendFloat(float f);
void DMA_USART_TO_ESP(char msg[]);

//init and string length function 

void myInit(void);
int lengthstr(char ch[]);

//analog initializtion and analogRead function

void ADC_Init(void);
uint16_t ADC_Read(uint8_t channel);


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
	
	myInit();
	
  osKernelInitialize();
  taskWifiHandle = osThreadNew(startTaskWifi, NULL, &taskWifi_attributes);
  taskSensorsHandle = osThreadNew(startTaskSensors, NULL, &taskSensors_attributes);
  osKernelStart();		//start exceuting the os functions 

	// code never reaches this place
	
  while (1)
  {
		
  }
}

void adxl_write (uint8_t address, uint8_t value)
{
	uint8_t data[2];
	data[0] = address|0x40;  // multibyte write
	data[1] = value;
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // pull the cs pin low
	HAL_SPI_Transmit (&hspi1, data, 2, 100);  // write data to register
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // pull the cs pin high
}

void adxl_read (uint8_t address)
{
	address |= 0x80;  // read operation
	address |= 0x40;  // multibyte read
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // pull the pin low
	HAL_SPI_Transmit (&hspi1, &address, 1, 100);  // send address
	HAL_SPI_Receive (&hspi1, data_rec, 6, 100);  // receive 6 bytes data
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // pull the pin high
}


void adxl_init (void)
{
	RCC->APB2ENR |= 0x8;			//port b clock enable
	GPIOB->CRL &= 0xF0FFFFFF;	//chip selsct pin enable
	GPIOB->CRL |= 0x02000000;
	adxl_write (0x31, 0x01);  // data_format range= +- 4g
	adxl_write (0x2d, 0x00);  // reset all bits
	adxl_write (0x2d, 0x08);  // power_cntl measure and wake up 8hz
}

void uartEnable(void)
{
	
	RCC->APB2ENR |= 0x4005;
	GPIOA->CRH &= 0xFFFFFF0F;
	GPIOA->CRH |= 0x000000B0;
	USART1->BRR = 625;
	USART1->CR1 = 0x200C;
	
//  //for debug purpose. enables usart2 at 9600 bps for 72 mhz master clock
//	RCC->APB1ENR |= (1<<17);
//	GPIOA->CRL &= 0xFFFFF0FF;
//	GPIOA->CRL |= 0x00000B00;
//	USART2->BRR = 3750;
//	USART2->CR1 = 0x200C;
}

/*
void USART2_SendFloat(float f) {
    char buffer[20];
    sprintf(buffer, "%f\n", f);
    for (int i = 0; buffer[i] != '\0'; i++) {
        // Wait until data register is empty
        while (!(USART2->SR & USART_SR_TXE));
        // Send character
        USART2->DR = buffer[i];
    }
}
*/

//builds data out of the raw received data from adxl345 and store it in 'vib'

void dataCon(void)
{
	x = (data_rec[1]<<8)|data_rec[0];
	y = (data_rec[3]<<8)|data_rec[2];
	z = (data_rec[5]<<8)|data_rec[4];
	float tempp = (x*x)+(y*y)+(z*z);
	if (tempp<0.0)tempp = tempp*(-1);
	vib = sqrt(tempp);
}

//initialize the gpio ports, pins and sensors

void myInit(void)
{
	RCC->AHBENR |= (1<<0); 					//DMA1 clock enable 
	RCC->APB2ENR |= 0xFC | (1<<14); //enable GPIO clocks//USART1 init.
	GPIOA->ODR |= (1<<10); 					// pull-up PA10 
	uartEnable();
	USART1->CR3 = (1<<7); 					// DMA Trans. enable 
	
	adxl_init();
	ADC_Init();
	
	//ESP-01 wifi init
	
	HAL_Delay(1000);
	DMA_USART_TO_ESP("AT+RST\r\n");
	HAL_Delay(1);
	DMA_USART_TO_ESP("AT+CWMODE=1\r\n");
	HAL_Delay(1);
	DMA_USART_TO_ESP("AT+CWJAP=\"hollowpurple\",\"password\"\r\n");  // SSID PASS
	HAL_Delay(5000);
	DMA_USART_TO_ESP("AT+CIPMUX=0\r\n");
	HAL_Delay(3000);
}

void ADC_Init(void) {
	
    RCC->APB2ENR |= (1 << 9) | (1 << 2)  | RCC_APB2ENR_AFIOEN;	//adc1, port a, afio clock enables							
    GPIOA->CRL &= 0xFFFFF000;						//set pins A0, A1, A2 to analog input

		RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; 	//from datasheet, div master clock by 6 to get it under 14 mhz 
    ADC1->CR2 |= (1 << 0);							//adc on
    ADC1->CR2 |= (1 << 3);							//calibration
    while (ADC1->CR2 & (1 << 3)); 			//wait till calibrated
    ADC1->SMPR2 |= (7 << (3 * 0)) | (7 << (3 * 1)) | (7 << (3 * 2)); // Sample time 55.5 cycles for channels 0, 1, 2
}

//this function works like arduino analogRead()

uint16_t ADC_Read(uint8_t channel) {
    ADC1->SQR3 = channel;							//Provide channel number 
    ADC1->CR2 |= 1;										//start conversion by rewriting 1 on adc on bit
    while (!(ADC1->SR & (1 << 1)));		//EOC FLAG CHECK
    return ADC1->DR;									//RETURN 12 BIT VAL
}


int lengthstr(char ch[])
{
	for (int c = 0;;++c)
	{
		if(ch[c]=='\0')return c;
	}
}

//just to write less code
// sending data from memory to usart, using dma

void DMA_USART_TO_ESP(char msg[])
{
	
	DMA1_Channel4->CCR = 0;
	DMA1_Channel4->CPAR = (uint32_t) &USART1->DR; 
	DMA1_Channel4->CMAR = (uint32_t) msg; 
	DMA1_Channel4->CNDTR = lengthstr(msg);
	DMA1_Channel4->CCR = (1<<7)|(1<<4); 
	DMA1_Channel4->CCR |= 1; 							
	while(((DMA1->ISR)&(1<<13)) == 0);
}

//once data is sent to ESP-01, refresh all the necessary variables

void refreshAfterSend(void)
{
	samp = 0;
	meanVib = 0;
	peakVib = 0;
	meanTemp = 0;
	peakTemp = 0;
	meanSnd = 0;
	peakSnd = 0;
	meanCrnt = 0;
	peakCrnt = 0;
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}


static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}



void startTaskWifi(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		float vv = meanVib/samp; 			//calculate the mean values
		float cc = meanCrnt / samp;
		float ss = meanSnd / samp;
		float tt = meanTemp / samp;
		char c[50];
		char temp[200];
		
		DMA_USART_TO_ESP("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
		osDelay(1000);
		sprintf(temp,"GET /update?api_key=Z6OVTTT9X7GCKGVM&field1=%f&field2=%f&field3=%f&field4=%f&field5=%f&field6=%f&field7=%f\r\n",vv,peakVib,cc,peakCrnt,ss,peakSnd,peakTemp);
		sprintf(c,"AT+CIPSEND=%d\r\n",lengthstr(temp));
		DMA_USART_TO_ESP(c);
		osDelay(500);
		DMA_USART_TO_ESP(temp);  // write API-Key here
		osDelay(5000);
		DMA_USART_TO_ESP("AT+CIPCLOSE=0\r\n");
		HAL_Delay(1);
		
		refreshAfterSend();						// refresh vars sfter sending data
		
    osDelay(15000);
  }
  /* USER CODE END 5 */
}

//read values and collect 1600 samples in about 20 seconds then send them through esp-01
//collects peak and accumulative values (to divide by samples and take mean)

void startTaskSensors(void *argument)
{
  /* USER CODE BEGIN startTaskSensors */
  /* Infinite loop */
  for(;;)
  {
		++samp;
		adxl_read(0x32);
		dataCon();
		meanVib = meanVib + vib;
		if(vib>peakVib)peakVib = vib;
		
		float val = (ADC_Read(0))*3.3/4095;
		crnt = (val - 1.66) / 0.185;
		meanCrnt = meanCrnt + crnt;
		if(crnt>peakCrnt)peakCrnt = crnt; 
		
		snd = ADC_Read(1);
		meanSnd = meanSnd + snd;
		if(snd>peakSnd)peakSnd = snd;
		
		temp = (ADC_Read(2))*3300/4095;
		temp = temp / 10;
		meanTemp = meanTemp + temp;
		if(temp>peakTemp)peakTemp = temp;
		
		
    osDelay(10);
  }
  /* USER CODE END startTaskSensors */
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
