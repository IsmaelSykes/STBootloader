/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "retarget.h"
#include <inttypes.h>
#include "memoryMap.h"
#include "image.h"
#include "my_memory.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define loader_rom  (0x08046800)
#define app_rom     (0x0800C800)

#define REV32(x) ( ((x&0xff000000)>>24) | (((x&0x00ff0000)<<8)>>16) | (((x&0x0000ff00)>>8)<<16) | ((x&0x000000ff) << 24) )
#define REV16(y) (REV32((uint32_t) y)>>16)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
static void MX_GPIO_Init_user(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t * RDAddr = NULL;
uint64_t double_word = 0;
bool dato_recivido=false, bootloader = false;
uint32_t counter = 0, pages = 0,offset = 0, n_bytes=0,rows = 0;
uint8_t rx_buff[520]={0x00};//
uint32_t FW_SIZE = 0, CRC_16 = 0, index_page = 0;
uint16_t crc = 0,crc_temp = 0, flag_timer = 0;
uint16_t a = 0, boot = 0,timer_flag=0, flag_break = 0;
uint32_t crc_part = 0, crc_rec;

#define STRINGIFY(x) #x
#define ADD_QUOTES(y) STRINGIFY(y)


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	  HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buff, sizeof rx_buff);
	  //memcpy(message,rx_buff,4);
	  dato_recivido = true;
	  //counter++;
}

static const uint16_t crc16Table[] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t CRC16_X25(const void * data, uint16_t sizeOfData, uint16_t startCrc)
{
	  const uint8_t * dataPtr = (const uint8_t *) data;

	  startCrc = startCrc ^ 0xffff;

	  while (sizeOfData--) {
	    startCrc = crc16Table[(startCrc ^ *dataPtr++) & 0xFF] ^ (startCrc >> 8);
	  }

	  return (startCrc ^ 0xffff);
}


uint32_t write(uint8_t *data,uint32_t begin)
{
	uint32_t end = begin+64;// return the address to next 512 bytes
	uint32_t _index;
	uint16_t k = 0;
	HAL_FLASH_Unlock();
	for(uint32_t i = begin; i<end; i++)// 64*8 = 512 bytes
	{
	  _index = 8*i;
	  memset(&double_word,0xFF,8);
	  memcpy(&double_word,&data[k],8);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,loader_rom+_index,double_word);
	  k+=8;
	}
	HAL_FLASH_Lock();
	return end;
}



void clear_app_rom(void)
{
	printf("\r Clean APP_Room \r\n");
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;

	HAL_FLASH_Unlock();
	FLASH->OPTR |= FLASH_OPTR_DUAL_BANK_Msk;
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.Page = 25;
	  EraseInitStruct.Banks = FLASH_BANK_1;
	  EraseInitStruct.NbPages = 103;
	  HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	  HAL_FLASH_Lock();

	  PageError = 0;
	  HAL_FLASH_Unlock();
	  FLASH->OPTR |= FLASH_OPTR_DUAL_BANK_Msk;
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.Page = 128;
	  EraseInitStruct.Banks = FLASH_BANK_2;
	  EraseInitStruct.NbPages = 13;
	  HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	  HAL_FLASH_Lock();
}

void clear_loader_rom(void)
{
	printf("\r Clean Loader_Room \r\n");
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;
	  PageError = 0;
	  HAL_FLASH_Unlock();
	  FLASH->OPTR |= FLASH_OPTR_DUAL_BANK_Msk;
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.Page = 141;
	  EraseInitStruct.Banks = FLASH_BANK_2;
	  EraseInitStruct.NbPages = 115;
	  HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	  HAL_FLASH_Lock();
}

void print_double_word(uint32_t Address,uint64_t *doubleword)
{
	uint8_t *ptr = (uint8_t*)doubleword;
	  	  printf("\r Address: %08lX, Data:  ",Address);
	  	  for(uint8_t k = 0; k < 8; k++)
	  		  printf("%02X",*(ptr++));
	  	  printf("|\r\r\n");
}

uint32_t clone_rom(uint32_t Firmware_zise)
{
	uint32_t rows = Firmware_zise/8;
	uint32_t offset = Firmware_zise%8;
	uint32_t _index = 0;

	for(int k = 0; k <10; k++)
	{
		printf("\033\143");
		printf("Clear app rom");
		for(int q = 0; q<=k; q++)
		{
			printf(".");
			HAL_Delay(100);
		}
		printf("\r\n");
	}

	clear_app_rom();

	printf("\r ------ Clone to slot 1 ---------- \r\n");
	printf("\r rows: %ld \r\n",rows);
	printf("\r offset: %ld \r\n",offset);
	for(int k = 0; k <10; k++)
	{
		printf("\033\143");
		printf(" waiting ");
		for(int q = 0; q<=k; q++)
		{
			printf(".");
			HAL_Delay(100);
		}
		printf("\r\n");
	}

	HAL_FLASH_Unlock();
    for(uint32_t i = 0; i<=rows-1; i++)// 12288 /8 -1 = =rows,   11532/8 =
    {
    	_index = 8*i;
  	  RDAddr = (uint64_t *)(loader_rom + _index);
  	  memset(&double_word,0xFF,8);
  	  memcpy(&double_word,RDAddr,8);
  	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,app_rom+_index,double_word);
  	  crc_temp = CRC16_X25(RDAddr, 8, crc_temp);
	  //printf(" \r crc parts_app_rom: %04X \n",crc_temp);
	  //print_double_word(loader_rom+_index,&double_word);
    }
    if(offset!=0)
    {
    	memset(&double_word,0xFF,8);
		_index+= 8;
		RDAddr = (uint64_t *)(loader_rom + _index);
		memset(&double_word,0xFF,8);
		memcpy(&double_word,RDAddr,offset);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,app_rom+_index,double_word);
		crc_temp = CRC16_X25(&double_word,offset, crc_temp);
		//printf(" \r crc parts_app_rom: %04X \n",crc_temp);
		//print_double_word(loader_rom+_index,&double_word);
    }
    HAL_FLASH_Lock();


    if(crc_temp == CRC_16)
   		return 0;
    else
    	return 1;
}

void wait(void)
{
	 while(dato_recivido == false)
	 {
		 printf("\r waiting... \r\n");
		 HAL_Delay(100);
		 if(flag_break)
		 {
			 break;
		 }
	 }
	HAL_TIM_Base_Stop_IT(&htim15);

}

uint32_t update_firmware (void)
{
	 uint32_t err = 0;
	 //uint8_t len=0;
	 uint8_t buffer[10];
	 uint8_t strnum[10];
	 uint8_t OK[3] = {0x4F,0x4B,0x20};
	 uint8_t ERR[4] = {0x45,0x52,0x52,0x20};
	 uint32_t cmd7 [2] = {0x00,0x00};
	 clear_loader_rom();
	 HAL_UART_Transmit(&huart2,"FZ\n", (sizeof("FZ\n")-1),500);// begin
	 printf("\r Send FZ ... \r\n");
	 wait();
	 if(flag_break)
	 {
		 return 1;
	 }
	 dato_recivido = false;

	memcpy(&FW_SIZE,&rx_buff[0],4);
	memcpy(&CRC_16,&rx_buff[4],4);
	pages = (uint32_t)(FW_SIZE/512);
	n_bytes = 512*pages;
	offset = FW_SIZE - n_bytes;
	printf(" \r fw_size: %lX \n",FW_SIZE);
	printf(" \r crc_app: %lX \n",CRC_16);
	printf(" \r pages: %ld \n",pages);
	printf(" \r n_bytes: %ld \n",n_bytes);
	printf(" \r offset: %ld \n",offset);
	HAL_UART_Transmit(&huart2,"OK\n", (sizeof("OK\n")-1),500);
	memset(rx_buff,'\0',sizeof(rx_buff));
	wait();
	 dato_recivido = false;

	 printf(" \r Starting loader.......... \r\n");
	int i =0;
	while( i<=pages-1)
	{
		memcpy(&index_page,&rx_buff[0],4);
		memcpy(&crc_part,&rx_buff[4],4);
		printf(" \r index: %lX \n",index_page);
		printf(" \r crc_part: %lX \n",crc_part);
		crc = CRC16_X25(&rx_buff[8], 512, 0);
		crc_rec = CRC16_X25(&rx_buff[8], 512, crc_rec);
		printf(" \r crc computed: %X \r\n",crc);
		printf(" \r crc_rec: %lX \r\n",crc_rec);

		if(crc == crc_part)
		{
			a = write(&rx_buff[8],a);

		  memset(buffer,'\0',sizeof(buffer));
		  memset(strnum,'\0',sizeof(strnum));
		  snprintf(strnum,sizeof(strnum), "%ld",index_page);
		  //printf("strnum: %s, len: %d \n",strnum,strlen(strnum));
		  memcpy(&buffer[0], OK, sizeof(OK));
		  memcpy(&buffer[sizeof(OK)], strnum,strlen(strnum));
		  printf(" \r %s\r\n",buffer);
		  HAL_UART_Transmit(&huart2,(uint8_t*)buffer, strlen(buffer),500);
			wait();
			dato_recivido = false;
			err = 0;
			i++;
		}
		else
		{
			err = 1;
			i = i;
			memset(buffer,'\0',sizeof(buffer));
		  memset(strnum,'\0',sizeof(strnum));
		  snprintf(strnum,sizeof(strnum), "%ld",index_page);
		  memcpy(&buffer[0], ERR, sizeof(ERR));
		  memcpy(&buffer[sizeof(ERR)], strnum,strlen(strnum));
		  printf(" \r buffer: %s\r\n",buffer);
		  HAL_UART_Transmit(&huart2,(uint8_t*)buffer, strlen(buffer),500);
		  //return -1;
		}

	}// end while
	if(offset!=0)
	{
		printf(" \r --------------Last Page ----------- \n");
		memcpy(&index_page,&rx_buff[0],4);
		memcpy(&crc_part,&rx_buff[4],4);
		printf(" \r index: %lX \n",index_page);
		printf(" \r crc_part: %lX \n",crc_part);
		crc = CRC16_X25(&rx_buff[8], offset, 0);//offset
		crc_rec = CRC16_X25(&rx_buff[8], offset, crc_rec);
		printf(" \r crc computed: %X \r\n",crc);
		printf(" \r crc_rec: %lX \r\n",crc_rec);


		if(crc == crc_part)
		{
			a = write(&rx_buff[8],a);

		  memset(buffer,'\0',sizeof(buffer));
		  memset(strnum,'\0',sizeof(strnum));
		  snprintf(strnum,sizeof(strnum), "%d",index_page);
		  //printf("strnum: %s, len: %d \n",strnum,strlen(strnum));
		  memcpy(&buffer[0], OK, sizeof(OK));
		  memcpy(&buffer[sizeof(OK)], strnum,strlen(strnum));
		  printf(" \r buffer: %s\r\n",buffer);
		  HAL_UART_Transmit(&huart2,(uint8_t*)buffer, strlen(buffer),500);
		  cmd7[0] = crc_rec;
		  cmd7[1] = 0xFFFFFFFF;
		  printf("\r ************************************* \r\n");
		  printf(" \r crc rec: %lX \r\n",crc_rec);
		  printf(" \r crc_app: %lX \n",CRC_16);
		  printf("\r ************************************* \r\n");
		  HAL_Delay(200);
		  HAL_UART_Transmit(&huart2,(uint8_t*)cmd7, sizeof(cmd7),500);
		  HAL_UART_Transmit(&huart2,(uint8_t*)cmd7, sizeof(cmd7),500);
		  HAL_UART_Transmit(&huart2,(uint8_t*)cmd7, sizeof(cmd7),500);
		//wait();
		dato_recivido = false;
		err = 0;
		}
		else{
			err = 1;
			memset(buffer,'\0',sizeof(buffer));
			  memset(strnum,'\0',sizeof(strnum));
			  snprintf(strnum,sizeof(strnum), "%d",index_page);
			  memcpy(&buffer[0], ERR, sizeof(ERR));
			  memcpy(&buffer[sizeof(ERR)], strnum,strlen(strnum));
			  printf(" \r buffer: %s\r\n",buffer);
			  HAL_UART_Transmit(&huart2,(uint8_t*)buffer, strlen(buffer),500);
			  //return -1;
		}
	}// offset

//---------------- Validate -------------------------
	const image_hdr_t *hdr = NULL;
	hdr = image_get_header(IMAGE_SLOT_2);//magic
	if (hdr == NULL)
	{
		printf("Magic incorrect \r\n");
		err =  -1;
	}
	if (image_validate(IMAGE_SLOT_2, hdr) != 0)//crc
	{
		printf("CRC incorrect \r\n");
		err = -1;
	}//*/
// -------------------------- Clone ----------------------------
	printf("Check Slot 2 \r\n");
	HAL_Delay(4000);
	printf("Ready to write to  Slot 1 \r\n");
	HAL_Delay(4000);

	if((CRC_16 == crc_rec) && (err == 0))
		err = clone_rom(FW_SIZE);
	if ( err == 0)
	{
		printf("UPDATE SUCCESSFULLY\r\n");
		return 0;
	}
	else
	{
		printf("UPDATE FAIL\r\n");
		return -1;
	}//*/

}// get_Firmware()

__attribute__( (naked, noreturn) ) static void BootJumpASM(uint32_t PC, uint32_t SP) {
	__asm("           \n\
			msr msp, r1 /* load r1 into MSP */\n\
			bx r0       /* branch to the address at r0 */\n\
	");
}

 void image_start(const image_hdr_t *hdr) {
    uint8_t i = 0;
	/* Disable interrupts */
	//Disable IRQ
	__disable_irq();

	//Disable the system timer
	SysTick->CTRL = 0;

	//Clear the exception pending bit
	SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk ;

	//Disable IRQs
    for (i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF; // disable IRQ
        NVIC->ICPR[i] = 0xFFFFFFFF; // clear pending IRQ
    }

	/* Modify vector table location */
	//Barriers
	__DSB();
	__ISB();

	//const DeviceVectors *vectors = (const DeviceVectors *) hdr->vector_addr;
	uint32_t *isr = (uint32_t *)hdr->vector_addr;
    SCB->VTOR = (uint32_t)isr & SCB_VTOR_TBLOFF_Msk;
    //SCB->VTOR = 0x0800F820;
    //0x800f820

		//Barriers
	__DSB();
	__ISB();

	/* Enable interrrupts */
	__enable_irq();

    BootJumpASM(isr[1], isr[0]);
	/* Execute application */
    __builtin_unreachable();
}

//int  __attribute__((section(".shared_mem")))contador = 0;

const image_hdr_t *hdr = NULL;

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
  //MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART5_UART_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  MX_GPIO_Init_user();
   RetargetInit(&huart5);
   /*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);// BOOT/*/

   printf("\r ------ Start Bootooader ----- \r\n");
   HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buff, sizeof rx_buff);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	shared_memory_init();
	timer_flag = 0;
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim15);


	while (1)
	{
		HAL_Delay(5);
		boot = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14);// 1/0
		printf("\r boot: %d \r\n",boot);
		bootloader = timer_flag & boot;
		if(bootloader)
		{
			printf(" App \n\r");
			HAL_TIM_Base_Stop_IT(&htim1);
			shared_mem_set_app_update_requested(false);
		}
		 if(bootloader == 0)
		{
			printf(" \r Bootloader \r\n");
			HAL_TIM_Base_Stop_IT(&htim1);
			shared_mem_set_app_update_requested(true);
		}

	    if((flag_break == 1) && (bootloader==0))
		{
			printf(" \r App Timer \r\n");
			shared_mem_set_app_update_requested(false);
		}

	if (shared_mem_is_bl_upd_requested()) {//bootloader
			hdr = image_get_header(IMAGE_SLOT_2); // get address y magic

		// Load the updater (apparom)
		if (hdr == NULL)
		{
			printf("No image found in slot 2\r\n");
		}
		else
		{
			if (image_validate(IMAGE_SLOT_2, hdr) != 0) {// addr =  &__loader_rom_start__ + sizeof(image_hdr_t), len -> crc(add, len)
				//Mismatch on crc
			}
		}
		//ioport_set_pin_level(RED_LED, false);
		//ioport_set_pin_level(GREEN_LED, false);
		//ioport_set_pin_level(BLUE_LED, false);
		printf("Jumping to updater\r\n");
		shared_mem_increment_boot_counter();
		//printf("Boot count: %d \r\n",shared_mem_get_boot_counter());
		image_start(hdr);

	}

	if (!shared_mem_is_app_upd_requested()) {//app //boot 1
		// Boot count, maybe not neccessary or implement a better thing to handle this
		const uint8_t max_boot_attemps = 4;
		if (shared_mem_get_boot_counter() >= max_boot_attemps)
		{
			shared_mem_clear_boot_counter();
			printf("App unstable, entering to DFU mode\r\n");
			break;
		}

		hdr = image_get_header(IMAGE_SLOT_1);// get address y magic

		// Load the app (apparom)
		if (hdr == NULL)
		{
			printf("No image found in slot 1\r\n");
			goto invalid;
			break;
		}

		if (image_validate(IMAGE_SLOT_1, hdr) != 0) { // addr =  &__loader_rom_start__ + sizeof(image_hdr_t), len -> crc(add, len)
			//CRC mismatch
			goto invalid;
			break;
		}

		//ioport_set_pin_level(RED_LED, false);
		//ioport_set_pin_level(GREEN_LED, false);
		//ioport_set_pin_level(BLUE_LED, false);

		printf("Jumping to application\r\n\n");
		shared_mem_increment_boot_counter();
		//printf("Boot count: %d \r\n",shared_mem_get_boot_counter());
		image_start(hdr);
	}

	else if (shared_mem_is_app_upd_requested())
	{

		if (update_firmware() == 0)
		{
			//Clean the variables
			printf("Update completed, restarting\r\n");
			shared_mem_set_update_completed();
			shared_mem_clear_boot_counter();
			shared_mem_clear_ota_info();
			shared_mem_set_app_update_requested(false);
			shared_mem_set_update();
			printf("shared_mem_get_update: %d \n\r",shared_mem_get_update());
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);//resetea el uC
			HAL_Delay(2000);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
			printf("Reset COMM \n\r");
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);//resetea el uC
			HAL_Delay(2000);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1);
			printf("Shutdown COMM 🐮 \n\r");

			HAL_NVIC_SystemReset();
		}
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }//end while
	HAL_NVIC_SystemReset(); //resetea el uC

	invalid:
		printf("\r\nFlash a valid application\r\n");
		while (true)
		{
			__asm__ __volatile__("");
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 16000-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 15000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LTE_ON_GPIO_Port, LTE_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_COMM_GPIO_Port, RST_COMM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LTE_ON_Pin */
  GPIO_InitStruct.Pin = LTE_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LTE_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_COMM_Pin */
  GPIO_InitStruct.Pin = RST_COMM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_COMM_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	  {
		timer_flag = 1 ;
	  }

   if(htim->Instance == TIM3)
     {
	   dato_recivido = 1;// offset is 0 or page integers
     }

   if(htim->Instance == TIM15)
     {
		printf("TIMER15\r\n\n");
		printf("flag_timer: %d\r\n\n",flag_timer);
		if(flag_timer)
		{
			flag_break = 1;
			printf("flag_break: %d\r\n\n",flag_break);
		}
		flag_timer = 1;

     }
}



static void MX_GPIO_Init_user(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(RST_COMM_GPIO_Port, RST_COMM_Pin, GPIO_PIN_SET);

	  GPIO_InitStruct.Pin = LTE_ON_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LTE_ON_GPIO_Port, &GPIO_InitStruct);


	  /*Configure GPIO pin : RST_COMM_Pin */
	  GPIO_InitStruct.Pin = RST_COMM_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(RST_COMM_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
  /*Configure GPIO pin : PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


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
