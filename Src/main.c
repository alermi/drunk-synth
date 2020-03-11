/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Drivers/BSP/STM32F4-Discovery/stm32f4_discovery_audio.h"
#ifndef __USBH_MIDI_CORE_H
#include "usbh_MIDI.h"
#endif
#include "MIDI_application.h"
#include "mtof.h"
#include "pitchbend1024.h"
#include "complexfreqangle.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define VOICE_NUM 2
#define HARMONIC_NUM 8
#define DRUNK_RANGE 2
#define STEP_PERIOD 44100
//struct rando
//{
//	uint32_t seed;
//};
typedef struct osc_t
{
	float phinc;	// phase increment
	float phase;	// phase
	float drunk_multiplier;
	uint32_t seed;
}	osc_t;

typedef struct synthVoice
{
	int note;
	int active;
	int velocity;
	float volume;
	float frequency;
	osc_t osc[HARMONIC_NUM];
}	synthVoice;

typedef enum
{
	INACTIVE = 0,
	ATTACK,
	DECAY,
	SUSTAIN,
	RELEASE
} voiceState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

uint32_t count_init;
uint32_t drunk_period_count;
int16_t codecBuffer[64];    // 32 samples X 2 channels
extern I2S_HandleTypeDef       hAudioOutI2s;
extern I2S_HandleTypeDef       hAudioInI2s;
// Sound globals
USBH_HandleTypeDef hUSBHost; /* USB Host handle */
MIDI_ApplicationTypeDef Appli_state = MIDI_APPLICATION_IDLE;
void audioBlock(float *input, float *output, int32_t samples);
float mtoinc[128];
int pitchbend;
int buttonState;
uint8_t runningStatus;
float inBuffer[1024], outBuffer[1024]; // interleaved - LRLRLRLRLRLRLRLRLRLRLR - inBuffer[frame <<
synthVoice voice[VOICE_NUM];
int keyboard[128];
float wavetable[4096];
// synth globals
float masterVolume;	// set by controller 20
// ADSR
float attack, decay, sustain, release;
// table lookup oscillator
int32_t tablesize, tablemask;
// sinewave creation
float sinTaylor(float phase);
// wave table creation
void tableInit(int32_t type);
// 2 point interpolated lookup
float tableSamp(float phase);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void  USBH_UserProcess_callback  (USBH_HandleTypeDef *pHost, uint8_t vId);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int i, j, waveType, deglitch;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  count_init = 0;
  drunk_period_count = 0;
  pitchbend = 8192;
  for(i = 0; i < 128; i++)
  {
	  mtoinc[i] = mtof[i] * 0.0000208333f; // 1/48000
	  keyboard[i] = -1;
  }
  for(i = 0; i < VOICE_NUM; i++)
  {
	  voice[i].active = INACTIVE;
	  voice[i].volume = 0.0f;
	  for(j = 0; j < HARMONIC_NUM; j++){
		  voice[i].osc[j].phase = 0.0f;
		  voice[i].osc[j].drunk_multiplier = ((float) (DRUNK_RANGE/2.0f));
		  voice[i].osc[j].seed = (uint32_t) ((i*HARMONIC_NUM) + j)*16777216;
	  }

  }
  masterVolume = 1.0f;
  attack = 0.01;
  release = 0.005;
  sustain = 0.8;
  decay = 0.1;
//		  release = sustain = decay = 1.0f;
  deglitch = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//  MX_I2C1_Init();
//  MX_I2S3_Init();
//  MX_SPI1_Init();
//  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
	buttonState = 0;
	waveType = 0;
  tableInit(waveType);
  /*## Init Host Library ################################################*/
  USBH_Init(&hUSBHost, USBH_UserProcess_callback, 0);

  /*## Add Supported Class ##############################################*/
  USBH_RegisterClass(&hUSBHost, USBH_MIDI_CLASS);

  /*## Start Host Process ###############################################*/
  USBH_Start(&hUSBHost);

  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 90, 48000);
  BSP_AUDIO_OUT_Play((uint16_t *)codecBuffer, 128);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1);
	  if(count_init < 100){
		  count_init++;
	  }
//	  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
//	  HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
    /* USER CODE END WHILE */
//    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
	  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
		  deglitch++;
	  else
		  deglitch--;
	  if(deglitch > 40) deglitch = 40;
	  if(deglitch < 0) deglitch = 0;

	MIDI_Application();
	// check button B1
	if(buttonState != HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) && deglitch > 30)
	{
		buttonState = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
		if(buttonState == 1)
		{
			waveType++;
			if(waveType > 2)
				waveType = 0;
			tableInit(waveType);
		}
	}
	//USBH_Delay(1);

	/* USBH_Background Process */
	USBH_Process(&hUSBHost);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ProcessMIDI(midi_package_t pack)
{
	if(count_init < 100){
		return;
	}
	int i,j;
	uint8_t status;

	// Status messages that start with F, for all MIDI channels
	// None of these are implemented - though we will flash an LED
	// for the MIDI clock
	status = pack.evnt0 & 0xF0;
	if(status == 0xF0)
	{
		switch(pack.evnt0)
		{
		case 0xF0:	// Start of System Exclusive
		case 0xF1:	// MIDI Time Code Quarter Fram
		case 0xF2:	// Song Position Pointer
		case 0xF3:	// Song Select
		case 0xF4:	// Undefined
		case 0xF5:	// Undefined
		case 0xF6:	// Tune Request
		case 0xF7:	// End of System Exclusive
			status = runningStatus = 0x00;
			break;
		case 0xF8:	// Timing Clock (24 times a quarter note)
			HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin); // RED LED
			break;
		case 0xF9:	// Undefined
		case 0xFA:	// Start Sequence
		case 0xFB:	// Continue Sequence
		case 0xFC:	// Pause Sequence
		case 0xFD:	// Undefined
		case 0xFE:	// Active Sensing
		case 0xFF:	// Reset all synthesizers to power up
			break;

		}
	}

// MIDI running status (same status as last message) doesn't seem to work over this USB driver
// code commented out.

//	else if((pack.evnt0 & 0x80) == 0x00)
//		status = runningStatus;
	else
		runningStatus = status = pack.evnt0 & 0xF0;



	switch(status)
	{
	case 0x80:	// Note Off
		// turn off all voices that match the note off note
		for(i = 0; i<VOICE_NUM; i++)
		{
			if(voice[i].note == pack.evnt1)
			{
				if(voice[i].active!=INACTIVE){
					voice[i].active = RELEASE;
					keyboard[voice[i].note] = -1;
				}
			}
		}
		break;
	case 0x90:	// Note On
		if(pack.evnt2 == 0) // velocity 0 means note off
			// turn off all voices that match the note off note
			for(i = 0; i<VOICE_NUM; i++)
			{
				if((voice[i].note == pack.evnt1) && (voice[i].active != INACTIVE))
				{
					voice[i].active = RELEASE;
					keyboard[voice[i].note] = -1;
				}
			}
		else
		{
			// if this key is already on, end the associated note and turn it off
			if(keyboard[pack.evnt1] != -1)
			{
				voice[keyboard[pack.evnt1]].active = RELEASE;
				keyboard[pack.evnt1] = -1;
			}
			// find an inactive voice and assign this key to it
			for(i = 0; i<VOICE_NUM; i++)
			{
				if(voice[i].active == INACTIVE)
				{
					voice[i].active = ATTACK;
					voice[i].note = pack.evnt1;
					voice[i].velocity = pack.evnt2;
					voice[i].volume = 0.0f;
					for(j=0; j<HARMONIC_NUM; j++){
						voice[i].osc[j].phase = 0.0f;
					}
					keyboard[pack.evnt1] = i;
					break;
				}
			}
		}
		break;
	case 0xA0:	// Polyphonic Pressure
		break;
	case 0xB0:	// Control Change
		switch(pack.evnt1) // CC number
		{
		case 1:
			break;
		case 20 	:
			break ;
		case 21 	:
			break ;
		case 22:
			break;
		case 23:
			break;
		case 24:
//			attack = 1.0f/(((pack.evnt2 * 2.0f) + 2.0f) * 48.0f) ;	// 2 to 256 ms - this is wrong
			break;
		case 25:
//			decay = 1.0f/(((pack.evnt2 * 2.0f) + 2.0f) * 48.0f) ;	// 2 to 256 ms - this is wrong
			break;
		case 26:
//			sustainKnob = pack.evnt2 * 0.0078125f;
			break;
		case 27:
//			release = 0.25f/(((pack.evnt2 * 2.0f) + 2.0f) * 48.0f); // 8 to 1024 ms
			break;
		case 7 :
			break;	// master volume
		}
		break;
	case 0xC0:	// Program Change
		break;
	case 0xD0:	// After Touch
		break;
	case 0xE0:	// Pitch Bend
		pitchbend = pack.evnt2 * 128 + pack.evnt1;
		break;
	}
}
float sinTaylor(float phase)
{
	// sin x = x - x^3/3! + x^5/5! - x7/7! + .....
	// adapted from phil burk's code on musicdsp.org
	float tp, yp, x, x2;

	while(phase > 1.0f)
		phase -= 1.0f;

	tp = phase * 2.0f - 1.0f;
	// as the taylor series is an approximation, this uses the more accurate data

	if(tp > 0.5f)
		yp = 1.0f - tp;
	else
	{
		if(tp < -0.5f)
	       yp = -1.0f - tp;
	    else
	       yp = tp;
	}
	x = yp * 3.141592653589793f;
	x2 = x*x;

	//Taylor expansion out to x**9/9! factored  into multiply-adds
	return(x*(x2*(x2*(x2*(x2*(0.000002755731922f)
	            - (0.000198412698413f))
	            + (0.008333333333333f))
	            - (0.166666666666667f))
	            + 1.0f));
}
void tableInit(int32_t type)
{
	// 0 - sine init, 1 - sawtooth init
	int i;
	float max;
	tablesize = 4096;
	tablemask = tablesize - 1;
	switch(type)
	{
	case 0:
		for(i=0; i<tablesize; i++)
			wavetable[i] = sinTaylor((float)i * 0.000244140625f);
		break;
	case 1:
		for(i=0; i<tablesize; i++)
		{
			wavetable[i] = sinTaylor((float)i * 0.000244140625f);
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 2.0f) / 2.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 3.0f) / 3.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 4.0f) / 4.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 5.0f) / 5.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 6.0f) / 6.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 7.0f) / 7.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 8.0f) / 8.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 9.0f) / 9.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 10.0f) / 10.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 11.0f) / 11.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 12.0f) / 12.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 13.0f) / 13.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 14.0f) / 14.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 15.0f) / 15.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 16.0f) / 16.0f;
		}
		max = 0.00001f;
		for(i=0; i<tablesize; i++)
		{
			if(wavetable[i]>max)
				max = wavetable[i];
		}
		for(i=0; i<tablesize; i++)
			wavetable[i] /= max;
		break;
	case 2:
		for(i=0; i<tablesize; i++)
		{
			wavetable[i] = sinTaylor((float)i * 0.000244140625f);
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 3.0f) / 3.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 5.0f) / 5.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 7.0f) / 7.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 9.0f) / 9.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 11.0f) / 11.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 13.0f) / 13.0f;
			wavetable[i] += sinTaylor((float)i * 0.000244140625f * 15.0f) / 15.0f;
		}
		max = 0.00001f;
		for(i=0; i<tablesize; i++)
		{
			if(wavetable[i]>max)
				max = wavetable[i];
		}
		for(i=0; i<tablesize; i++)
			wavetable[i] /= max;
		break;
	}

}
float table_samp(float phase)
{
	float frac;
	int32_t phint;

	// calculate the fractional part
	phint = (int32_t)(phase*tablesize);
	frac = phase*tablesize - phint;

	return(wavetable[phint&tablemask] * (1.0f - frac) + wavetable[(phint+1)&tablemask] * frac);
}
float osc_samp(osc_t *o)
{
	o->phase += o->phinc;
	if(o->phase >= 1.0f) o->phase -= 1.0f;
	if(o->phase < 0.0f) o->phase += 1.0f;
	return(table_samp(o->phase));
}


inline uint32_t fast_rand(uint32_t seed){
	seed = (seed * 196314165) + 907633515;
	return seed;
}


inline void step_osc(osc_t *o){
	uint32_t new_seed = fast_rand(o->seed);
	float new_volume = ((float) new_seed) / (4294967295.0f / DRUNK_RANGE);
	o->drunk_multiplier = new_volume;
	o->seed = new_seed;
}


void audioBlock(float *input, float *output, int32_t samples)
{
	int i, f, q, v;
	float wave, masterInc, sustainInc, c, c1, c2, d;

/*	1 - add second oscillator
	2 - frequency offset
	3 - phase modulation
	4 - using the button
	5 - ADSR knobs
	6 - dezippering of external controls
	7 - using the ADC inputs*/

	for(i = 0; i < samples; i += 2)
	{
		int step_this_sample;
		drunk_period_count++;
		if (drunk_period_count == STEP_PERIOD){
			step_this_sample = 1;
			drunk_period_count = 0;
		}
		else{
			step_this_sample = 0;
		}
		// all voice synth units
		output[i] = output[(i) + 1] = 0.0f;

		// per voice synth units
		for(v = 0; v < VOICE_NUM; v++)
		{
			int j;
			if(step_this_sample == 1){
				for(j=0; j<HARMONIC_NUM; j++){
					step_osc(&voice[v].osc[j]);
				}
			}
			if(voice[v].active != INACTIVE)
			{
//				float waveSum = 0.0f;
				voice[v].frequency = mtoinc[voice[v].note] * pitchbend1024[pitchbend>>4];

				// sawtooth oscillator phase increment calculation
//				voice[v].osc[0].phinc = voice[v].frequency;
//				voice[v].osc[1].phinc = voice[v].frequency;

//				wave = osc_samp(&(voice[v].osc[0]));// + osc_samp(&(voice[v].osc[1]));

				//Square Wave Shape
				wave = 0.0f;
				for(j=0; j<HARMONIC_NUM; j++){
					int power = (j*2) + 1;
					voice[v].osc[j].phinc = (voice[v].frequency*power);
					//TODO: Get rid of the divide
					wave += (osc_samp(&(voice[v].osc[j]))/power) * voice[v].osc[j].drunk_multiplier;
				}




				// use voice volume to control the simplest filter
				f = voice[v].volume * 128.0f;
				if(f > 127) f = 127;
				if(f < 0) f = 0;

				output[i] += wave * voice[v].volume * voice[v].velocity * 0.0009765625f * masterVolume;

				// update the simple envelope
 				if(voice[v].active == ATTACK)
				{
					voice[v].volume += attack;
					if(voice[v].volume >= 1.0f)
						voice[v].active = DECAY;
				}
 				else if(voice[v].active == DECAY)
				{
					voice[v].volume -= decay;
					if(voice[v].volume <= sustain)
						voice[v].active = SUSTAIN;
				}
 				else if(voice[v].active == SUSTAIN)
					voice[v].volume = sustain;
				else if(voice[v].active == RELEASE)
				{
					voice[v].volume -= release;
					if(voice[v].volume <= 0.0f)
						voice[v].active = INACTIVE;
				}
			}
			output[(i) + 1] = output[i];
		}
	}
}
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
	int i;
	audioBlock(inBuffer, outBuffer, 32);
	for(i = 0; i < 32; i+=2)
	{
		codecBuffer[i+0] = (int16_t)((outBuffer[i]) * 32767.0f);
		codecBuffer[i+1] = (int16_t)((outBuffer[i+1]) * 32767.0f);
	}
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
	int i;
    BSP_AUDIO_OUT_ChangeBuffer((uint16_t *)codecBuffer, 64);
	audioBlock(inBuffer, outBuffer, 32);
	for(i = 0; i < 32; i+=2)
	{
		codecBuffer[i+32] = (int16_t)((outBuffer[i]) * 32767.0f);
		codecBuffer[i+33] = (int16_t)((outBuffer[i+1]) * 32767.0f);
	}
}

void BSP_AUDIO_OUT_Error_CallBack(void)
{
    /* Stop the program with an infinite loop */
    while (1) {}
}
/**
  * @brief  This function handles main I2S interrupt.
  * @param  None
  * @retval 0 if correct communication, else wrong communication
  */
void I2S3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hAudioOutI2s.hdmatx);
}

/**
  * @brief  This function handles DMA Stream interrupt request.
  * @param  None
  * @retval None
  */
void I2S2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}
/*====================================================================================================*/
/**
 * @brief  User Process function callback
 * @param  phost: Host Handle
 * @param  id: Host Library user message ID
 * @retval none
 */
static void USBH_UserProcess_callback (USBH_HandleTypeDef *pHost, uint8_t vId)
{
	switch (vId)
	{
	case HOST_USER_SELECT_CONFIGURATION:
		break;

	case HOST_USER_DISCONNECTION:
		Appli_state = MIDI_APPLICATION_DISCONNECT;
		// 4 - green, 3 - orange, 5 - red, 6 - blue
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
		break;

	case HOST_USER_CLASS_ACTIVE:
		Appli_state = MIDI_APPLICATION_READY;
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		break;

	case HOST_USER_CONNECTION:
		Appli_state = MIDI_APPLICATION_START;
		HAL_GPIO_WritePin(GPIOD, 0x8000, GPIO_PIN_SET);
		break;

	default:
		break;

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
