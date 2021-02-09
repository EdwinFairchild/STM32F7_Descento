#include <stm32f7xx_ll_bus.h>
#include <stm32f7xx_ll_gpio.h>
#include <stm32f7xx_ll_utils.h>
#include <stm32f7xx_ll_system.h>
#include <stm32f7xx_ll_pwr.h>
#include <stm32f7xx_ll_i2c.h>
#include <stm32f7xx_ll_usart.h>
#include <stm32f7xx_ll_rcc.h>
#include <stm32f7xx_ll_cortex.h>

//-----| Non ST Headers |------------
#include "CONFIG.h"
#include "CL_systemClockUpdate.h"
#include "CL_delay.h"
#include "CL_printMsg.h"
#include "CL_bfp.h"
#include "MPU6050.h"


//-----| Variables/Defines |---------
#define LED_PIN				LL_GPIO_PIN_14
#define LED_PORT			GPIOB
#define MPU6050_ADDRESS		0xD0

//MPU6050
#define ACCEL_X_HIGH_POS	 0
#define ACCEL_X_LOW_POS		 1
#define ACCEL_Y_HIGH_POS	 2
#define ACCEL_Y_LOW_POS		 3
#define ACCEL_Z_HIGH_POS	 4
#define ACCEL_Z_LOW_POS		 5
#define GYRO_X_HIGH_POS		 6
#define GYRO_X_LOW_POS		 7
#define GYRO_Y_HIGH_POS		 8
#define GYRO_Y_LOW_POS		 9
#define GYRO_Z_HIGH_POS		10
#define GYRO_Z_LOW_POS		11

uint8_t sensor_data[12] = { 0 };
long temp = 0; 

//raw values from sensors
long accel_x_raw = 0;
long accel_y_raw = 0;
long accel_z_raw = 0;

long gyro_x_raw  = 0;
long gyro_y_raw  = 0;
long gyro_z_raw  = 0;

//useful values after calibration and conversion
long accel_x_val = 0;
long accel_y_val = 0;
long accel_z_val = 0;

long gyro_x_val	 = 0;
long gyro_y_val  = 0;
long gyro_z_val  = 0;

//values used to calculate offset
long accel_x_cal = 0;
long accel_y_cal = 0;
long accel_z_cal = 0;
long gyro_x_cal  = 0;
long gyro_y_cal  = 0;
long gyro_z_cal  = 0;


float angle;


//-----| CLI |---------
CL_cli_type cli;

//-----| Protoypes |---------
//hardware
void blinkLed(uint8_t times, uint8_t delay);
void initLED(void);
//mpu
void initMPU(void);

//peripherals
void SystemClock_Config(void);
void MX_USART3_UART_Init(void);
void MX_I2C1_Init(void); 

//app functions
void i2c_mem_read(uint8_t mem_addr, uint8_t *data_destination, uint8_t len_to_read);
void i2c_write(uint8_t *data, uint8_t len);
void i2c_write_no_stop(uint8_t *data, uint8_t len);
void cmd_ok_handler(uint8_t num, char *values[]);

int main(void)
{
	CL_configClock();
	CL_delay_init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	initLED();
	initMPU();



	cli.prompt = "eddie>";
	cli.delimeter = '\r';
	cli.registerCommand("ok", ' ', cmd_ok_handler, "Prints \"ok\" if cli is ok");
	CL_cli_init(&cli);


	

	for (;;)
	{	
//		if (cli.parsePending == true)
//			cli.parseCommand(&cli);
		LL_GPIO_TogglePin(LED_PORT, LED_PIN);
		delayMS(1000);
	}
//-----------------------------------------------------------------
}
void blinkLed(uint8_t times, uint8_t delay)
{
	for (int i = 0; i < times; i++)
	{
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14);
		LL_mDelay(delay);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14);
		LL_mDelay(delay);
	}
//-----------------------------------------------------------------
}
void initLED(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_14, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_LOW);
//-----------------------------------------------------------------
}
void initMPU(void)
{

	uint8_t configs[2];


	//	//sample rate divider
	//	configs[0] = 0x19;
	//	configs[1] = 0x00; 
	//	i2c_write(configs, 2);
	//
	//	//Config register : Digital Low Pass filter 8kHz
	//	configs[0] = 0x1A;
	//	configs[1] = 0x00; 
	//	i2c_write(configs, 2);

		//gyro config and self test regiser : Full Range Scale +- 250 deg/s
		configs[0] = 0x1B;
	configs[1] = 0x00; 
	i2c_write(configs, 2);

	//accelerometer config and self test regiser : Full Range Scale +- 2g
	configs[0] = 0x1C;
	configs[1] = 0x00; 
	i2c_write(configs, 2);

	//Power management register : Power on and internal 8MHz oscillator
	configs[0] = 0x6b;
	configs[1] = 0x00;
	i2c_write(configs, 2);

//-----------------------------------------------------------------
}
void calibrate_offsets(uint32_t samples)
{
	//get acceleromter samples	
	

	for(int i = 0 ;   i < samples ; i++)
	{
		
		i2c_mem_read(MPU6050_RA_ACCEL_XOUT_H,sensor_data, 6);
		accel_x_raw += (int16_t)((sensor_data[0] << 8) | sensor_data[1]);  //shift  accel x HIGH and LOW into one number
		accel_y_raw += (int16_t)((sensor_data[2] << 8) | sensor_data[3]);   //shift  accel y HIGH and LOW into one number
		accel_z_raw += (int16_t)((sensor_data[4] << 8) | sensor_data[5]);   //shift  accel z HIGH and LOW into one number
	}

	//get gyro samples	
	
	for(int i = 0 ;   i < samples ; i++)
	{
		
		i2c_mem_read(MPU6050_RA_GYRO_XOUT_H,sensor_data, 6);
		gyro_x_raw += (int16_t)((sensor_data[6] << 8) | sensor_data[7]);   //shift  accel x HIGH and LOW into one number
		gyro_y_raw += (int16_t)((sensor_data[8] << 8) | sensor_data[9]);    //shift  accel y HIGH and LOW into one number
		gyro_z_raw += (int16_t)((sensor_data[10] << 8) | sensor_data[11]);    //shift  accel z HIGH and LOW into one number
	}

	accel_x_cal = accel_x_raw / samples;
	accel_y_cal = accel_y_raw / samples;
	accel_z_cal = accel_z_raw / samples;

	gyro_x_cal = gyro_x_raw / samples;	
	gyro_y_cal = gyro_y_raw / samples;
	gyro_z_cal = gyro_z_raw / samples;
	
	
}
int32_t get_angel(void)
{
	//############Finish this function
	i2c_mem_read(MPU6050_RA_GYRO_ZOUT_H, &sensor_data[ACCEL_Z_HIGH_POS], 2);


}
void get_all_sensor_values(void)
{
	i2c_mem_read(MPU6050_RA_ACCEL_XOUT_H, sensor_data, 6);
	i2c_mem_read(MPU6050_RA_GYRO_XOUT_H, &sensor_data[6], 6);
}
void get_gyro_values(void)
{
	
	i2c_mem_read(MPU6050_RA_GYRO_XOUT_H, &sensor_data[6], 6);

}
void get_accel_values(void)
{
	
	i2c_mem_read(MPU6050_RA_ACCEL_XOUT_H, sensor_data, 6);
}

void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	LL_I2C_InitTypeDef I2C_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	/**I2C1 GPIO Configuration  
	PB6   ------> I2C1_SCL
	PB7   ------> I2C1_SDA 
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	/** I2C Initialization 
	*/
	LL_I2C_EnableAutoEndMode(I2C1);
	LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
	LL_I2C_DisableOwnAddress2(I2C1);
	LL_I2C_DisableGeneralCall(I2C1);
	LL_I2C_EnableClockStretching(I2C1);
	I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
	I2C_InitStruct.Timing = 0x1070699E;
	I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
	I2C_InitStruct.DigitalFilter = 0;
	I2C_InitStruct.OwnAddress1 = 0;
	I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
	I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
	LL_I2C_Init(I2C1, &I2C_InitStruct);
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */
//-----------------------------------------------------------------
}
void i2c_mem_read(uint8_t mem_addr, uint8_t *data_destination, uint8_t len_to_read)
{
	//send the memory/register address to read from
	LL_I2C_ClearFlag_ADDR(I2C1);
	LL_I2C_DisableAutoEndMode(I2C1);
	LL_I2C_SetTransferSize(I2C1, 1);
	LL_I2C_SetSlaveAddr(I2C1, MPU6050_ADDRESS);
	LL_I2C_SetTransferRequest(I2C1, LL_I2C_REQUEST_WRITE);
	LL_I2C_GenerateStartCondition(I2C1);

	LL_I2C_TransmitData8(I2C1, mem_addr);
	while (!(I2C1->ISR & I2C_ISR_TXE)) ;

	LL_I2C_ClearFlag_ADDR(I2C1);
	//LL_I2C_EnableAutoEndMode(I2C1);
	LL_I2C_SetTransferSize(I2C1, len_to_read);
	LL_I2C_SetSlaveAddr(I2C1, MPU6050_ADDRESS);
	LL_I2C_SetTransferRequest(I2C1, LL_I2C_REQUEST_READ);
	LL_I2C_GenerateStartCondition(I2C1);
	
	for (int i =  0; i < len_to_read; i++)
	{	
		while (!(I2C1->ISR & I2C_ISR_RXNE)) ;
		*data_destination++ = LL_I2C_ReceiveData8(I2C1);	
	}

	LL_I2C_GenerateStopCondition(I2C1);
//-------------------------------------------------------------------
}
void i2c_write(uint8_t *data, uint8_t len)
{	
	LL_I2C_ClearFlag_ADDR(I2C1);
	LL_I2C_EnableAutoEndMode(I2C1);
	LL_I2C_SetTransferSize(I2C1, len);
	LL_I2C_SetSlaveAddr(I2C1, MPU6050_ADDRESS);
	LL_I2C_SetTransferRequest(I2C1, LL_I2C_REQUEST_WRITE);
	LL_I2C_GenerateStartCondition(I2C1);
	
		
	for (int i =  0; i < len; i++)
	{	

		while (!(I2C1->ISR & I2C_ISR_TXE)) ;
		LL_I2C_TransmitData8(I2C1, *data++);		
		
	}
	
	//autoend enabled
	//LL_I2C_GenerateStopCondition(I2C1);
//-------------------------------------------------------------------
}
void i2c_write_no_stop(uint8_t *data, uint8_t len)
{	
	LL_I2C_ClearFlag_ADDR(I2C1);
	LL_I2C_DisableAutoEndMode(I2C1);
	LL_I2C_SetTransferSize(I2C1, len);
	LL_I2C_SetSlaveAddr(I2C1, MPU6050_ADDRESS);
	LL_I2C_SetTransferRequest(I2C1, LL_I2C_REQUEST_WRITE);
	LL_I2C_GenerateStartCondition(I2C1);
	
		
	for (int i =  0; i < len; i++)
	{	

		
		LL_I2C_TransmitData8(I2C1, *data++);
		while (!(I2C1->ISR & I2C_ISR_TXE)) ;
	
	}
	
//-----------------------------------------------------------------
}
void cmd_ok_handler(uint8_t num, char *values[])
{
	//	******figure out how to handle help messages
	//	if ( !(strcmp(values[0], "?")) )
	//	{
	//		CL_printMsg("\r\nThis is the help messg\r\n");
	//	}
	//	else
	//	{
			CL_printMsg("System ok! \r\n");
	//}
//--------------------------------------------------
}
void USART3_IRQHandler(void)
{
	if ((USART3->ISR & USART_ISR_RXNE)) //if data has arrived on the uart
		{
			USART3->ISR &= ~(USART_ISR_RXNE); //clear interrupt
		
			//fetch data
			cli.charReceived = USART3->RDR;

			//if the character receieved is not the delimeter then echo the character
			if(cli.charReceived != cli.delimeter)
				USART3->TDR = cli.charReceived; 
			cli.parseChar(&cli);				
		}
	
	
}//----------------------------------------------------------------
void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
  
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	/**USART3 GPIO Configuration  
	PB10   ------> USART3_TX
	PB11   ------> USART3_RX 
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART3, &USART_InitStruct);
	LL_USART_ConfigAsyncMode(USART3);
	LL_USART_EnableIT_RXNE(USART3);
	NVIC_EnableIRQ(USART3_IRQn);
	USART3->BRR  = 0x1D4; //<----per datasheet example
	LL_USART_Enable(USART3);
	
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */
//-----------------------------------------------------------------
}