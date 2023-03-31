/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void(*ptr_code)(void);	//���庯��ָ���������ڴ�����ת
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define size_boot_code_area						(28) //????
#define size_app_code_area						(482)//????
#define size_backup_area						(2)
#define address_boot_code_area					(FLASH_BASE)
#define address_app_code_area					(address_boot_code_area + size_boot_code_area*1024)
#define address_backup_area						(address_app_code_area + size_app_code_area*1024)
#define backup_app_valid_offset				(4)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//�ع���printf����ָ��USART1
int fputc(int byte, FILE *f)
{
	while((USART1->SR&0x40) == 0){};
	USART1->DR = (unsigned char)byte;
	return byte;
}

//���SD���Ƿ����
bool tf_card_inserted(void)
{
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) ? false : true;
}

//���´��뵽APP������
int update_app_code(FIL *fp, const char *firmware_version_fname)
{
	FLASH_EraseInitTypeDef erase;
	uint32_t error;
	uint16_t i, flash_page;
	
	UINT br;
	
	static uint8_t buffer[FLASH_PAGE_SIZE];
	
	if(fp == NULL) return -1;
	
	printf("-------------start erasing-------------\r\n");
	//����Flash
	HAL_FLASH_Unlock();
	//��ҳ����APP�����������Ϣ��������
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = address_backup_area;
	erase.NbPages = size_backup_area * 1024 / FLASH_PAGE_SIZE;
	if(HAL_FLASHEx_Erase(&erase, &error) != HAL_OK) {
		//��Ϣ�����������ʧ��
		HAL_FLASH_Lock();
		return -1;
	}
	erase.PageAddress = address_app_code_area;
	erase.NbPages = size_app_code_area * 1024 / FLASH_PAGE_SIZE;
	if(HAL_FLASHEx_Erase(&erase, &error) != HAL_OK) {
		//APP�����������ʧ��
		HAL_FLASH_Lock();
		return -1;
	}
	//����APP��������
	printf("-------------start programming-------------\r\n");
	flash_page = 0;
	do {
		if(f_read(fp, (void*)buffer, FLASH_PAGE_SIZE, &br) != FR_OK) {
			//��ȡAPP��������ʧ��
			HAL_FLASH_Lock();
			return -1;
		}
		//�����ֵķ�ʽ�����´��븴�Ƶ�APP��������ע�������FLASH�����ŵĴ�С��ģʽ.
		for(i = 0; i < ((br%2) ? (br+1)/2 : br/2); i++) 
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address_app_code_area + flash_page*FLASH_PAGE_SIZE + i*2, (uint64_t)((buffer[i*2+1]<<8)|buffer[i*2]));
		printf("%dKb code has been programmed...\r\n", (++flash_page)*2);
	}while(br == FLASH_PAGE_SIZE);
	//���´����ȡ��ϣ��ж��Ƿ����ļ�ĩβ
	if(!f_eof(fp)) {
		//�ļ�û�н�����֤�������ȡ���´���Ĺ��̳�������
		HAL_FLASH_Lock();
		printf("-------------programming aborted-------------\r\n");
		return -1;
	}
	//������ȷ���µ�APP���򣬱��APP��Ч
	/*----------------------------------------------------------*/
	//HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address_backup_area, 0x55555555);
	/*----------------------------------------------------------*/

	/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address_backup_area, 0x55555555);
	for(i = 0; i < (strlen(firmware_version_fname)%2 ? strlen(firmware_version_fname)+2:strlen(firmware_version_fname)+3); i+=2)//���汾�Ŵ���Flash
	{
		HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD, address_backup_area + backup_app_valid_offset + i, (uint16_t)((firmware_version_fname[i+1]<<8)|firmware_version_fname[i]));
	}
	/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	
	HAL_FLASH_Lock();
	printf("-------------programming finished-------------\r\n");
	return 0;
}


//��ת��APP
void jump_to_app(void)
{
	printf("info:jump to app\r\n");
	
	ptr_code ptr_app = (ptr_code)*(unsigned int*)(address_app_code_area +4);
	__set_MSP(*(unsigned int*)address_app_code_area);
	SCB->VTOR = address_app_code_area;
	ptr_app();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t tick;
	uint32_t app_valid_val;
	FATFS	fatfs;//�ļ�ϵͳ ����ṹ��
	FIL		file; //�ļ� ����ṹ��
	
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
  MX_USART1_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
#ifdef __ENABLE_DEBUG__
	printf("debug >> bootloader starting...\r\n");
#endif
	//���TF��ǰ����ʱһ��ʱ��ȴ�ϵͳ��ƽ�ȶ�
	tick = HAL_GetTick();
	while(HAL_GetTick() - tick < 500){};
	
	
	//����Ƿ����TF��     !!!!!
	if(!tf_card_inserted()) {
		//û�м�⵽TF�������ж�APP�������Ƿ��п��ô���
		app_valid_val = *(uint32_t*)(address_backup_area);
		if(app_valid_val == 0x55555555) {
			//APP�������д��룬����ת
			jump_to_app();
		}
		else {
			//APP�������޴��룬���������ʾ
			printf("error:no app available,please follow the tips to download app!\r\n");
			printf("1.compile the marlin project and find firmware.bin in the project folder.\r\n");
			printf("2.copy firmware.bin to root directory of the TF card.\r\n");
			printf("3.insert the TF card to the slot and reboot the board.\r\n");
			printf("4.wait for app update finish information.\r\n");
			while(1);
		}
	}
	
	//��TF�����룬�����ļ�ϵͳ
	if(f_mount(&fatfs, "0:", 1) != FR_OK) {
		//�ļ�ϵͳ����ʧ�ܣ����������������APP�������Ƿ��п��ô���
		printf("warning:file system mounted failed, you can try format the TF card with FAT option!\r\n");
		printf("warning:now we try directly jumping to app!\r\n");
		app_valid_val = *(uint32_t*)(address_backup_area);
		if(app_valid_val == 0x55555555) {
			//APP�������д��룬ֱ����ת��APP
			jump_to_app();
		}
		else {
			//APP�������޴��룬���������ʾ
			printf("error:no app available,please follow the tips to download app!\r\n");
			printf("1.compile the marlin project and find firmware.bin in the project folder.\r\n");
			printf("2.format your TF card with FAT option\r\n");
			printf("3.copy firmware.bin to root directory of the TF card.\r\n");
			printf("4.insert the TF card to the slot and reboot the board.\r\n");
			printf("5.wait for app update finish information.\r\n");
			while(1);
		}
	}
	
	//�ļ�ϵͳ���سɹ�������Ŀ¼���Ƿ���ڸ����ļ�firmware.bin
#ifdef __ENABLE_DEBUG__
	printf("debug >> filesystem mounted successfully\r\n");
#endif

	/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	DIR     dir;  //·�� ����ṹ��
	FILINFO fno;  //�ļ���Ϣ�ṹ�壬���磺�ļ���С���ļ���..
	FRESULT fres; //�ļ�������ط���ֵ

	unsigned char fname_buff[_MAX_LFN] = {0}; //�洢Ŀ���ļ������ַ�����
	unsigned char vname_buff[_MAX_LFN] = {0}; //��ǰ���ع̼��İ汾��
	unsigned char freebuff[_MAX_LFN] = {0};
	fno.lfsize = _MAX_LFN;  //���ļ�������󳤶� 255
	fno.lfname = freebuff;  //���ļ����Ĵ洢�ռ�

	if(f_opendir(&dir,"") != FR_OK) //�����ļ�Ŀ¼ʧ��
	{
		printf("1111111 warning:bin file directory open failed, there might be something wrong with it!\r\n");
		printf("1111111 warning:now we try directly jumping to app!\r\n");
		app_valid_val = *(uint32_t*)(address_backup_area);
		if(app_valid_val == 0x55555555) {
			//APP�������д��룬ֱ����ת��APP
			jump_to_app();
		}
		else {
			//APP�������޴��룬���������ʾ
			printf("error:no app available,please follow the tips to download app!\r\n");
			printf("1.compile the marlin project and find firmware.bin in the project folder.\r\n");
			printf("2.format your TF card with FAT option\r\n");
			printf("3.copy firmware.bin to root directory of the TF card.\r\n");
			printf("4.insert the TF card to the slot and reboot the board.\r\n");
			printf("5.wait for app update finish information.\r\n");
			while(1);
		}
	}

	for(;;)  //������Ŀ¼�µ������ļ��������ļ�������
	{
		fres = f_readdir(&dir,&fno);
		
#if _USE_LFN
		if(fres != FR_OK || (fno.lfname[0] == 0 ? fno.fname[0]:fno.lfname) == 0) {
#else
		if(fres != FR_OK || fno.fname[0] == 0) {
#endif
			printf("2222222 warning:bin file directory read name failed, there might be something wrong with it!\r\n");
			printf("2222222 warning:now we try directly jumping to app!\r\n");
			app_valid_val = *(uint32_t*)(address_backup_area);
			if(app_valid_val == 0x55555555) {
				//APP�������д��룬ֱ����ת��APP
				jump_to_app();
			}
			else {
				//APP�������޴��룬���������ʾ
				printf("error:no app available,please follow the tips to download app!\r\n");
				printf("1.compile the marlin project and find firmware.bin in the project folder.\r\n");
				printf("2.format your TF card with FAT option\r\n");
				printf("3.copy firmware.bin to root directory of the TF card.\r\n");
				printf("4.insert the TF card to the slot and reboot the board.\r\n");
				printf("5.wait for app update finish information.\r\n");
				while(1);
			}
		}
		
#if _USE_LFN
		if(strstr((fno.lfname[0] == 0 ? fno.fname:fno.lfname),".bin") == NULL) 
				continue;
#else
		if(strstr(fno.fname[0] == 0 ,".bin") == NULL) 
				continue;
#endif
		else {
#if _USE_LFN
			strcpy(fname_buff,(fno.lfname[0] == 0 ? fno.fname:fno.lfname));
#else
			strcpy(fname_buff,fno.fname);
#endif
			printf("New firmwork version��%s\r\n",fname_buff);
			for(int i = 0; i < 255; i++)//��flash�еĹ̼��汾�Ÿ�����������������
			{
				vname_buff[i] = *(uint8_t*)(address_backup_area + backup_app_valid_offset+i);
			}
			printf("Old firmwork version��%s\r\n",vname_buff);
			break;
		}
	}

	//�жϱȶ� �̼��� firmware version  addr:address_backup_area + backup_app_valid_offset
	if(strcmp(fname_buff,vname_buff) == FR_OK)
	{
		printf("333333 warning:The two bin files have the same file name with it\r\n");
		printf("333333 warning:now we try directly jumping to app!\r\n");
		app_valid_val = *(uint32_t*)(address_backup_area);
		if(app_valid_val == 0x55555555) {
			//APP�������д��룬ֱ����ת��APP
			jump_to_app();
		}
		else {
			//APP�������޴��룬���������ʾ
			printf("error:no app available,please follow the tips to download app!\r\n");
			printf("1.compile the marlin project and find firmware.bin in the project folder.\r\n");
			printf("2.format your TF card with FAT option\r\n");
			printf("3.copy firmware.bin to root directory of the TF card.\r\n");
			printf("4.insert the TF card to the slot and reboot the board.\r\n");
			printf("5.wait for app update finish information.\r\n");
			while(1);
		}
	}
	/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	//���̼��汾�űȶԽ��Ϊ����ͬ����������ִ��
	
	if(f_stat(fname_buff, NULL) != FR_OK) {
		//δ��⵽firmware.bin��������ת��APP
		app_valid_val = *(uint32_t*)(address_backup_area);
		if(app_valid_val == 0x55555555) {
			//APP�������д��룬ֱ����ת��APP
			jump_to_app();
		}
		else {
			//APP�������޴��룬���������ʾ
			printf("error:no app available,please follow the tips to download app!\r\n");
			printf("1.compile the marlin project and find firmware.bin in the project folder.\r\n");
			printf("2.copy firmware.bin to root directory of the TF card.\r\n");
			printf("3.insert the TF card to the slot and reboot the board.\r\n");
			printf("4.wait for app update finish information.\r\n");
			while(1);
		}
	}

	//��⵽.bin�ļ������������µ�APP������
#ifdef __ENABLE_DEBUG__
	printf("debug >> update firmware.bin to app code area...\r\n");
#endif
	if(f_open(&file, fname_buff, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
		//�����ļ���ʧ�ܣ����������������APP�������Ƿ��п��ô���
		printf("warning:firmware.bin open failed, there might be something wrong with it!\r\n");
		printf("warning:now we try directly jumping to app!\r\n");
		app_valid_val = *(uint32_t*)(address_backup_area);
		if(app_valid_val == 0x55555555) {
			//APP�������д��룬ֱ����ת��APP
			jump_to_app();
		}
		else {
			//APP�������޴��룬���������ʾ
			printf("error:no app available,please follow the tips to download app!\r\n");
			printf("1.compile the marlin project and find firmware.bin in the project folder.\r\n");
			printf("2.format your TF card with FAT option\r\n");
			printf("3.copy firmware.bin to root directory of the TF card.\r\n");
			printf("4.insert the TF card to the slot and reboot the board.\r\n");
			printf("5.wait for app update finish information.\r\n");
			while(1);
		}
	}
	//�ɹ���firmware.bin����ȡ���ݲ����´���
	if(update_app_code(&file, fname_buff) != 0) {
		//�ر�firmware.bin
		f_close(&file);
		//�������ʧ�ܣ����APP�������Ƿ��п��ô���
		printf("warning:firmware.bin update failed, an error may occured!\r\n");
		printf("warning:now we try directly jumping to app!\r\n");
		app_valid_val = *(uint32_t*)(address_backup_area);
		if(app_valid_val == 0x55555555) {
			//APP�������д��룬ֱ����ת��APP
			jump_to_app();
		}
		else {
			//APP�������޴��룬���������ʾ
			printf("error:no app available,please follow the tips to download app!\r\n");
			printf("1.format your TF card with FAT option\r\n");
			printf("2.recompile the marlin project and find firmware.bin in the project folder.\r\n");
			printf("3.try update the firmware.bin agiin.\r\n");
			while(1);
		}
	}
	//APP���³ɹ�,�޸�TF�ڵ�firmware.binΪfirmware.CUR,��ת��APP
	f_close(&file);

	/*----------------------------------------------------------*/
	//f_rename("firmware.bin", "firmware.CUR"); //!!!!!!
	/*----------------------------------------------------------*/

	jump_to_app();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_ENABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 18;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
