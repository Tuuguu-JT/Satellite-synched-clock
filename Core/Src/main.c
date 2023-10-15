/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"math.h"
#include"gps.h"
#include"lcd16x2_i2c.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern GPS_t GPS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2) GPS_UART_CallBack();
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define timer_freq 4.0
#define T0H 2000
#define T1H 5000
#define T0L 8000
#define T1L 5000
#define TMH 8000
#define TML 2000
#define Treset 10000
//Time data from NTP
uint8_t Rxdata[10];
int rxcount = 0;

//RTC data

#define RTC_address 0xD0
#define decToBcd(val) ((uint8_t) ((val / 10 * 16) + (val % 10)))
uint8_t RTCdata[10];


//IRIG-B data definition part
uint8_t data[13];

uint16_t pos;
uint8_t mask = 0x80;
uint8_t lastbit;
uint8_t ind = 0;
uint8_t mark_ind = 0;

int s[8] = {1,2,4,8,0,10,20,40};
int m[8] = {1,2,4,8,0,10,20,40};
int h[8] = {0,1,2,4,8,0,10,20};
int d0[8] = {0,0,1,2,4,8,0,10};
int d1[8] = {20,40,80,100,200,0,0,0};
int y_0[8] = {0,0,0,0,1,2,4,8};
int y_1[8] = {0,10,20,40,80,0,0,0};
int days_of_month[13] = {0,31,59,90,120,151,181,212,243,273,304,334,365};
int days_of_month_leap_year[13] = {0,31,60,91,121,152,182,213,244,274,305,335,366};

int years, months, days, last_year, days_on_LCD;

int hours_to_send, minutes_to_send, seconds_to_send, days_to_send = 0;

int seconds_bit = 0, minutes_bit = 0, hours_bit = 0, days_bit0 = 0, days_bit1 = 0, years_bit0 = 0, years_bit1 = 0;

long double period;
uint16_t low_CCR1, low_ARR, high_CCR1, high_ARR, treset_ARR, mark_CCR1, mark_ARR;

int sec = 0;

int is_leap_year = 0, last_leap_year = 20;

int timer_stopped = 1, second_counter = 0, disconnected = 0;

uint8_t RTC_hour, RTC_minute, RTC_second, RTC_day, RTC_month, RTC_year, last_RTC_second;

int sign, eight_hour, four_hour, two_hour, one_hour, half_hour, ntp_on;

int add_sub_hour = 0, add_sub_min = 0;

int hours_lcd,minutes_lcd,seconds_lcd, years_lcd, months_lcd;

bool transmission_fin = 1;
int prev_sec;

int temp1, temp2;


void IRIG_setup(void){
	period = 1 / timer_freq;
	low_CCR1 = round(T0H / period);
	low_ARR = round((T0H + T0L) / period);
	high_CCR1 = round(T1H / period);
	high_ARR = round((T1H + T1L) / period);

	mark_CCR1 = round(TMH / period);
	mark_ARR = round((TMH + TML) / period);
	treset_ARR = ceil(Treset / period);

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER |= GPIO_MODER_MODER12_1;
	GPIOD->AFR[1] = ((GPIOD->AFR[1] & (0xf<<(4*(12-8)))) | 0x2<<(4*(12-8)));

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->PSC = 0;
	TIM4->CCMR1 = (TIM4->CCMR1 & ~(0x6<<4)) | (0x6<<4);
	TIM4->CCR1 = 0;
	TIM4->CCER |= TIM_CCER_CC1E;
	TIM4->CR1 |= TIM_CR1_CEN;
	TIM4->CR1 |= TIM_CR1_ARPE;
	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;
	TIM4->DIER &= ~TIM_DIER_UIE;
	TIM4->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TIM4_IRQn);
}

void give_data(){
	data[0] = seconds_bit;
	data[1] = minutes_bit;
	data[2] = hours_bit;
	data[3] = days_bit0;
	data[4] = days_bit1;
	data[5] = years_bit0;
	data[6] = years_bit1;
	data[8] = 0;
	data[9] = 0;
	data[10] = 0;
	data[11] = 0;
	data[12] = 0;

	seconds_bit = 0;
	minutes_bit = 0;
	hours_bit = 0;
	days_bit0 = 0;
	days_bit1 =0;
	years_bit0 = 0;
	years_bit1 = 0;
}

void time_val_to_bits(int years, int days, int hours, int minutes, int seconds){
	int set = 0;
	years_lcd = years;
	hours_lcd = hours;
	minutes_lcd = minutes;
	seconds_lcd = seconds;

	for(int i = 0; i < 8; i++){
		set = seconds - s[7-i];
		if(set >= 0 && s[7-i] != 0){
			seconds = set;
			seconds_bit = seconds_bit | (1 << i);
		}

		set = minutes - m[7-i];
		if(set >= 0 && m[7-i] != 0){
			minutes = set;
			minutes_bit = minutes_bit | (1 << i);
		}

		set = hours - h[7-i];
		if(set >= 0 && h[7-i] != 0){
			hours = set;
			hours_bit = hours_bit | (1 << i);
		}
		set = days - d1[7-i];
		if(set >= 0 && d1[7-i] != 0){
			days = set;
			days_bit1 = days_bit1 | (1 << i);
		}

		set = years - y_1[7-i];
		if(set >= 0 && y_1[7-i] != 0){
			years = set;
			years_bit1 = years_bit1 | (1 << i);
		}

	}
	for(int i = 0; i < 8; i++){
		set = days - d0[7-i];
		if(set >= 0 && d0[7-i] != 0){
			days = set;
			days_bit0 = days_bit0 | (1 << i);
		}

		set = years - y_0[7-i];
		if(set >= 0 && y_0[7-i] != 0){
			years = set;
			years_bit0 = years_bit0 | (1 << i);
		}
	}

	give_data();
}

void send_data(){
	pos = 0;
	lastbit = 0;
	ind = 0;
	mask = 0x80;

	TIM4->SR &= ~TIM_SR_UIF;
	TIM4->DIER |= TIM_DIER_UIE;
}


void TIM4_IRQHandler(void){

	TIM4->SR &= ~TIM_SR_UIF;

	if(ind < 100){
		if(ind%10 != 8 && ind < 99){
			if(data[pos] & mask){
				TIM4->CCR1 = high_CCR1;
				TIM4->ARR = high_ARR;
			}
			else{
				TIM4->CCR1 = low_CCR1;
				TIM4->ARR = low_ARR;
			}

			if(mask==1){mask = 0x80; pos+=1;}
			else{mask = mask >> 1;}
		}
		else{
			TIM4->CCR1 = mark_CCR1;
			TIM4->ARR = mark_ARR;
		}
		ind++;
	}

	else{
		TIM4->CCR1 = 0;
		TIM4->DIER &= ~TIM_DIER_UIE;
		transmission_fin = 1;
	}
}

void separate_data(int utctime, int date, bool is_from_GPS){
	if(is_from_GPS){
		hours_to_send = utctime / 10000;
		minutes_to_send = (utctime - hours_to_send * 10000)/100;
		seconds_to_send = (utctime - hours_to_send * 10000 - minutes_to_send * 100) + 1;


		days = date / 10000;
		days_on_LCD = days;
		months = (date - days * 10000)/100;
		years = (date - days * 10000 - months * 100);
	}
	else{
		years = Rxdata[0];
		months = Rxdata[1];
		days = Rxdata[2];
		days_on_LCD = days;
		hours_to_send = Rxdata[3];
		minutes_to_send = Rxdata[4];
		seconds_to_send = Rxdata[5] + 1;
	}

	months_lcd = months;
	hours_to_send += add_sub_hour;
	minutes_to_send += add_sub_min;

	if(seconds_to_send == 60) {seconds_to_send = 0; minutes_to_send+=1;}

	if(minutes_to_send >= 60){
		minutes_to_send -= 60;
		hours_to_send++;
	}
	else if(minutes_to_send < 0){
		minutes_to_send += 60;
		hours_to_send--;
	}

	if(hours_to_send >= 24){
		hours_to_send -= 24;
		days++;
	}
	else if(hours_to_send < 0){
		hours_to_send += 24;
		days--;
	}


	if(years%400 == 0 || (years%100 != 0 && years%4 == 0)) is_leap_year = 1;
	else is_leap_year = 0;

	if(is_leap_year){
		days_to_send = days_of_month_leap_year[months-1]+days;
		if(days_to_send > 366){years++; days -= 366;}
	}
	else{
		days_to_send = days_of_month[months-1]+days;
		if(days_to_send > 365){years++; days -= 365;}
	}
}

void setting_time(){
	sign = !HAL_GPIO_ReadPin(Sign_GPIO_Port, Sign_Pin);
	eight_hour = !HAL_GPIO_ReadPin(Eight_hour_GPIO_Port, Eight_hour_Pin);
	four_hour = !HAL_GPIO_ReadPin(Four_hour_GPIO_Port, Four_hour_Pin);
	two_hour = !HAL_GPIO_ReadPin(Two_hour_GPIO_Port, Two_hour_Pin);
	one_hour = !HAL_GPIO_ReadPin(One_hour_GPIO_Port, One_hour_Pin);
	half_hour = !HAL_GPIO_ReadPin(Half_hour_GPIO_Port, Half_hour_Pin);
	ntp_on = !HAL_GPIO_ReadPin(NTP_On_GPIO_Port, NTP_On_Pin);

	add_sub_hour = 0;
	add_sub_min = 0;

	if(eight_hour){add_sub_hour += 8;}
	if(four_hour){add_sub_hour += 4;}
	if(two_hour){add_sub_hour += 2;}
	if(one_hour){add_sub_hour += 1;}
	if(half_hour){add_sub_min += 30;}

	if(!sign){
		add_sub_hour *= -1;
		add_sub_min *= -1;
	}

}

bool RTC_init(I2C_HandleTypeDef *pI2cHandle){
	RTCdata[0] = 0x00;
	RTCdata[1] = 0x7F;

	HAL_I2C_Master_Transmit(pI2cHandle, RTC_address, RTCdata, 2, 10);
	return 0;
}

void RTC_read(I2C_HandleTypeDef *pI2cHandle,uint8_t* year,uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* minute, uint8_t* second){
	RTCdata[0] = 0x06;
	HAL_I2C_Master_Transmit(pI2cHandle, RTC_address, RTCdata, 1, 10);
	HAL_I2C_Master_Receive(pI2cHandle, RTC_address, &RTCdata[1], 1, 10);
	*year = ((RTCdata[1] & 0xf0) >> 4)*10 +  (RTCdata[1] & 0x0f);

	RTCdata[0] = 0x05;
	HAL_I2C_Master_Transmit(pI2cHandle, RTC_address, RTCdata, 1, 10);
	HAL_I2C_Master_Receive(pI2cHandle, RTC_address, &RTCdata[1], 1, 10);
	*month = ((RTCdata[1] & 0xf0) >> 4)*10 +  (RTCdata[1] & 0x0f);

	RTCdata[0] = 0x04;
	HAL_I2C_Master_Transmit(pI2cHandle, RTC_address, RTCdata, 1, 10);
	HAL_I2C_Master_Receive(pI2cHandle, RTC_address, &RTCdata[1], 1, 10);
	*day = ((RTCdata[1] & 0xf0) >> 4)*10 +  (RTCdata[1] & 0x0f);

	RTCdata[0] = 0x02;
	HAL_I2C_Master_Transmit(pI2cHandle, RTC_address, RTCdata, 1, 10);
	HAL_I2C_Master_Receive(pI2cHandle, RTC_address, &RTCdata[1], 1, 10);
	*hour = ((RTCdata[1] & 0xf0) >> 4)*10 +  (RTCdata[1] & 0x0f);

	RTCdata[0] = 0x01;
	HAL_I2C_Master_Transmit(pI2cHandle, RTC_address, RTCdata, 1, 10);
	HAL_I2C_Master_Receive(pI2cHandle, RTC_address, &RTCdata[1], 1, 10);
	*minute = ((RTCdata[1] & 0xf0) >> 4)*10 +  (RTCdata[1] & 0x0f);

	RTCdata[0] = 0x00;
	HAL_I2C_Master_Transmit(pI2cHandle, RTC_address, RTCdata, 1, 10);
	HAL_I2C_Master_Receive(pI2cHandle, RTC_address, &RTCdata[1], 1, 10);
	*second = ((RTCdata[1] & 0xf0) >> 4)*10 +  (RTCdata[1] & 0x0f);
}

int dayofweek(int d, int m, int y)
{
    static int t[] = { 0, 3, 2, 5, 0, 3,
                       5, 1, 4, 6, 2, 4 };
    y -= m < 3;
    return ( y + y / 4 - y / 100 +
             y / 400 + t[m - 1] + d) % 7;
}

void RTC_settime(I2C_HandleTypeDef *pI2cHandle,uint8_t year,uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second){
	uint8_t set_time[7];
	set_time[0] = decToBcd(second);
	set_time[1] = decToBcd(minute);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dayofweek(day, month, year));
	set_time[4] = decToBcd(day);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);

	HAL_I2C_Mem_Write(pI2cHandle, RTC_address, 0x00, 1, set_time, 7, 100);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  GPS_Init();
  IRIG_setup();
  setting_time();
  lcd16x2_i2c_init(&hi2c1);
  //RTC_init(&hi2c1);

  //RTC_settime(&hi2c1, 23, 10, 9, 14, 43, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(ntp_on){
		HAL_I2C_Master_Receive(&hi2c1, 1<<1, Rxdata,6, 100);
		if(Rxdata[5] != prev_sec && transmission_fin){
			separate_data(GPS.utc_time, GPS.date,0);
			time_val_to_bits(years,days_to_send,hours_to_send, minutes_to_send, seconds_to_send);

			send_data();
			lcd16x2_i2c_setCursor(0, 0);
			lcd16x2_i2c_printf("   %d:%02d:%02d  ", 2000+RTC_year,RTC_month,RTC_day);
			lcd16x2_i2c_setCursor(1, 0);
			lcd16x2_i2c_printf("    %02d:%02d:%02d  ", RTC_hour,RTC_minute,RTC_second);
			prev_sec = Rxdata[5];
			transmission_fin = 0;
		}
	}
	else if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)){
		RTC_read(&hi2c1, &RTC_year, &RTC_month, &RTC_day, &RTC_hour, &RTC_minute, &RTC_second);

		lcd16x2_i2c_setCursor(0, 0);
		lcd16x2_i2c_printf("   %d:%02d:%02d  ", 2000+RTC_year,RTC_month,RTC_day);
		lcd16x2_i2c_setCursor(1, 0);
		lcd16x2_i2c_printf("    %02d:%02d:%02d  ", RTC_hour,RTC_minute,RTC_second);
	}

	else{
		lcd16x2_i2c_setCursor(0, 0);
		lcd16x2_i2c_printf("   %d:%d:%d  ", 2000+years_lcd,months_lcd,days_on_LCD);
		lcd16x2_i2c_setCursor(1, 0);
		lcd16x2_i2c_printf("    %d:%d:%d %d %d", hours_lcd,minutes_lcd,seconds_lcd,second_counter,disconnected);
	}

	if(disconnected){
	  if(RTC_second - last_RTC_second != 0){
		  time_val_to_bits(RTC_year, days_of_month_leap_year[RTC_month-1]+RTC_day, RTC_hour, RTC_minute, RTC_second);
		  last_RTC_second = RTC_second;
	  }
	}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin){
		case RST_Pin:
			setting_time();
			break;
		case PPS_Pin:
			if(!ntp_on){
				second_counter = 0;

				separate_data(GPS.utc_time, GPS.date,1);
				time_val_to_bits(years,days_to_send,hours_to_send, minutes_to_send, seconds_to_send);
				send_data();

				if(timer_stopped){
					disconnected = 0;
					HAL_TIM_Base_Start_IT(&htim11);
					timer_stopped = 0;
				}
			}
			break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	second_counter++;

	if(second_counter > 5){
		disconnected = 1;
		HAL_TIM_Base_Stop_IT(&htim11);
		timer_stopped = 1;
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
