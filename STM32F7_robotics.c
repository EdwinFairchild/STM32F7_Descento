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
#include <math.h>
#include "CONFIG.h"
#include "CL_systemClockUpdate.h"
#include "CL_delay.h"
#include "CL_printMsg.h"
#include "CL_bfp.h"
#include "MPU6050.h"
#include "MadgwickAHRS.h"

//-----| Misc Variables/Defines |---------
#define LED_PIN				LL_GPIO_PIN_14
#define LED_PORT			GPIOB
CL_cli_type cli;

//------------------| MPU6050 stuff |---------------------------------
#define MPU6050_ADDRESS		0xD0
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
float angle2;
float set_point = 0.00;

//------------------| stepper stuff |---------------------------------
#define R_M_PORT		GPIOF
#define R_M_STEP_PIN		8
#define R_M_DIR_PIN			0
#define R_M_ENABLE_PIN		2

#define L_M_PORT		GPIOG
#define L_M_STEP_PIN		0 //port A pin 0  Timer 2 Channel 1
#define L_M_DIR_PIN			2
#define L_M_ENABLE_PIN		3



#define ENABLE_RIGHT_MOTOR()	(GPIOF->BSRR = 1 << (R_M_ENABLE_PIN + 16))
#define ENABLE_LEFT_MOTOR()		(GPIOG->BSRR = 1 << (L_M_ENABLE_PIN + 16))

#define DISABLE_RIGHT_MOTOR()	(R_M_PORT->BSRR = 1 << (R_M_ENABLE_PIN ))
#define DISBLE_LEFT_MOTOR()		(L_M_PORT->BSRR = 1 << (L_M_ENABLE_PIN ))

#define L_M_FORWARD()			(L_M_PORT->BSRR = 1 << (L_M_DIR_PIN + 16))
#define L_M_BACKWARDS()			(L_M_PORT->BSRR = 1 << (L_M_DIR_PIN ))
#define R_M_FORWARD()			(R_M_PORT->BSRR = 1 << (R_M_DIR_PIN + 16))
#define R_M_BACKWARDS()			(R_M_PORT->BSRR = 1 << (R_M_DIR_PIN ))

#define L_M_STEP_HIGH()          (L_M_PORT->BSRR = 1 << L_M_STEP_PIN)   
#define L_M_STEP_LOW()           (L_M_PORT->BSRR = 1 << (L_M_STEP_PIN+16))    
#define R_M_STEP_HIGH()          (R_M_PORT->BSRR = 1 << R_M_STEP_PIN)   
#define R_M_STEP_LOW()           (R_M_PORT->BSRR = 1 << (R_M_STEP_PIN+16)) 



uint16_t STEP_DELAY = 10000; 
uint16_t STEP_COUNT = 500;
int32_t motor_move_count = 0; 
uint32_t tick = 0; 
//-----| Protoypes |---------
//MCU peripherals
void SystemClock_Config(void);
void MX_USART3_UART_Init(void);
void MX_I2C1_Init(void); 
void main_timer_init(void);
void main_timer_stop(void);
void main_timer_start(void);
void onePulseMode(void);
void step_left_motor(void);
void step_right_motor(void);

//onboard hardware
void blinkLed(uint8_t times, uint8_t delay);
void initLED(void);
void init_test_point_pin(void);

//mpu
void initMPU(void);
void calibrate_offsets(float samples);
void get_all_sensor_values(void);
void get_gyro_values(void);
void get_accel_values(void);
float get_angle(void);
//i2c 
void i2c_mem_read(uint8_t mem_addr, uint8_t *data_destination, uint8_t len_to_read);
void i2c_write(uint8_t *data, uint8_t len);
void i2c_write_no_stop(uint8_t *data, uint8_t len);



void stepper_move(uint16_t step_count, uint16_t delay);
void init_stepper_pins(void);


//-----| CLI |---------

void cmd_ok_handler(uint8_t num, char *values[]);
void cmd_stepper_delay_handler(uint8_t num, char *values[]);
void cmd_stepper_count_handler(uint8_t num, char *values[]);
void cmd_stepper_move_handler(uint8_t num, char *values[]);
void cmd_set_point_handler(uint8_t num, char *values[]);


//dtuff to delete eventualkly 
float accel_x_angle, accel_x_angle_old , accel_y_angle, gyro_x_angle;
float fused_angle;
uint32_t counter = 1;




int main(void)
{
	//crap load if initilizations
	CL_configClock();
	CL_delay_init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	main_timer_init();
	initLED();
	init_test_point_pin();

	init_stepper_pins();
	initMPU();

	//calibrate gyro
	calibrate_offsets(500.0);

	//configure the command line interface
	cli.prompt = "eddie>";
	cli.delimeter = '\r';
	CL_cli_init(&cli);
	cli.registerCommand("ok", ' ', cmd_ok_handler, "Prints \"ok\" if cli is ok");
	cli.registerCommand("sDelay", ' ', cmd_stepper_delay_handler, "sets the delay between steps");
	cli.registerCommand("sCount", ' ', cmd_stepper_count_handler, "sets number of steps to take");
	cli.registerCommand("sMove", ',', cmd_stepper_move_handler, "sets number of steps and delay and then moves");
	cli.registerCommand("spt", ',', cmd_set_point_handler, "sets the setpoint angle");

	//setup wireless if i get around to it


		
	main_timer_start();



	//-------------calculate an offset average incase the gyro isnt set properly
//	float temp;
//	
//	for(int i = 0 ; i < 500 ; i++)
//	{
//		temp += get_angle();	
//	}
//	temp /= 500;
//	set_point = get_angle() - temp; //test point for PID setpoint
	//onePulseMode();
	
	for (;;)
	{	
		if (cli.parsePending == true)
		{
			cli.parseCommand(&cli);			
		}

		
			
		get_all_sensor_values();

		long accel_x_raw =(int16_t) (sensor_data[0] << 8 | sensor_data[1]) - accel_x_cal;	
		long accel_y_raw = (int16_t)(sensor_data[2] << 8 | sensor_data[3]) - accel_y_cal;	
		long accel_z_raw = (int16_t)(sensor_data[4] << 8 | sensor_data[5]) - accel_y_cal;	
		accel_x_angle = asin((accel_x_raw  / 16384.0)) * 57.296;

		

//consider improving this to have an independant variable so that we dont have to reset tick varaible 
	//	LL_GPIO_TogglePin(GPIOG, LL_GPIO_PIN_0);
		//while(tick < 1500); //100ms give or take, given fact that timer interrupts every 66uS 
		//tick = 0;
	
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
void init_test_point_pin(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);
	LL_GPIO_InitTypeDef stepper_pins;
	LL_GPIO_StructInit(&stepper_pins);
	stepper_pins.Pin   = LL_GPIO_PIN_0 ;
	stepper_pins.Mode  = LL_GPIO_MODE_OUTPUT;
	stepper_pins.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; 
	LL_GPIO_Init(GPIOG, &stepper_pins);
}
void init_stepper_pins(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
	LL_GPIO_InitTypeDef stepper_pins;
	LL_GPIO_StructInit(&stepper_pins);
	stepper_pins.Pin   = LL_GPIO_PIN_0 | LL_GPIO_PIN_2 |  LL_GPIO_PIN_8 ;
	stepper_pins.Mode  = LL_GPIO_MODE_OUTPUT;
	stepper_pins.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; 
	LL_GPIO_Init(GPIOF, &stepper_pins);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);
	LL_GPIO_StructInit(&stepper_pins);
	stepper_pins.Pin   = LL_GPIO_PIN_1 | LL_GPIO_PIN_2 |  LL_GPIO_PIN_3;
	stepper_pins.Mode  = LL_GPIO_MODE_OUTPUT;
	stepper_pins.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; 
	LL_GPIO_Init(GPIOG, &stepper_pins);

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
void USART3_IRQHandler(void)
{
//used by cli 
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
void calibrate_offsets(float samples)
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
float get_angle(void)
{
	
	//i2c_mem_read(MPU6050_RA_ACCEL_XOUT_H, &sensor_data[ACCEL_X_HIGH_POS], 2);
	get_all_sensor_values();
	long accelX = (int16_t)(sensor_data[0] << 8 | sensor_data[1]) ;
	angle = asin((accelX  / 16384.0)) *57.296;
	
	
	return angle;

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
void main_timer_init(void)
{
/*	This timer runs at 20Khz and is responsible to telling TIMER_2 to step the motors
	given PID conditions*/

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	NVIC_EnableIRQ(TIM3_IRQn);
			
	LL_TIM_InitTypeDef myTIM; 
	LL_TIM_StructInit(&myTIM);	
	
	myTIM.Autoreload  = 5400; //for micro-secondish delay 
	myTIM.Prescaler = 0;
	
	LL_TIM_SetUpdateSource(TIM3, LL_TIM_UPDATESOURCE_COUNTER);   //update even will only be set by over/undeflow of counter
	LL_TIM_EnableIT_UPDATE(TIM3);
	//	LL_TIM_GenerateEvent_UPDATE(TIMER);	
	LL_TIM_Init(TIM3, &myTIM);
}

void main_timer_start(void)
{
	LL_TIM_EnableCounter(TIM3);
}
void main_timer_stop(void)
{
	LL_TIM_DisableCounter(TIM3);
}
void TIM3_IRQHandler(void)
{
	TIM3->SR &= ~TIM_SR_UIF; //clear interrupt
	tick++;
	volatile uint32_t inLineDelay = 0;
	

	if (motor_move_count > 0)
	{
		
		


	}


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

void stepper_move(uint16_t step_count, uint16_t delay)
{
	GPIOF->BSRR = 1 << (R_M_ENABLE_PIN + 16); //ENABLE motor driver active low enable pin
	for (int i = 0; i < STEP_COUNT; i++)
	{
		GPIOF->BSRR = 1 << R_M_STEP_PIN; //set
		delayUS(2);
		GPIOF->BSRR = 1 <<( R_M_STEP_PIN + 16);  //reset
		delayUS(delay);
	}
	GPIOF->BSRR = 1 << (R_M_ENABLE_PIN);  //DIsable Motor Driver active low enable pin
}

void cmd_stepper_delay_handler(uint8_t num, char *values[])
{
	STEP_DELAY = atoi(values[0]);
	CL_printMsg("--Delay set to : %d--\r\n", STEP_DELAY);

}
void cmd_stepper_count_handler(uint8_t num, char *values[])
{
	STEP_COUNT = atoi(values[0]);
	CL_printMsg("--Steps set to : %d--\r\n", STEP_COUNT);
}

void cmd_stepper_move_handler(uint8_t num, char *values[])
{
	STEP_COUNT = atoi(values[0]);
	STEP_DELAY = atoi(values[1]);
	CL_printMsg("--Steps set to : %d --Delay set to : %d--\r\n", STEP_DELAY, STEP_COUNT);
	stepper_move(STEP_COUNT, STEP_DELAY);
}

void cmd_set_point_handler(uint8_t num, char *values[])
{

}

void onePulseMode(void)
{
	//Tim3 as one pulse mode
	//Ch1 PA6 is output signal 
	//ch2 PA7 is input trigger
	/* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = { 0 };
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	/* USER CODE BEGIN TIM21_Init 1 */

	/* USER CODE END TIM21_Init 1 */
	TIM_InitStruct.Prescaler = 50;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 100;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM2, &TIM_InitStruct);



	LL_TIM_DisableARRPreload(TIM2);


	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);


	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 50;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);




	LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);


	/* USER CODE END TIM21_Init 2 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	

	GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	LL_TIM_EnableCounter(TIM2);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);   //do i really need this?
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);    //do i really need this?
	LL_TIM_SetOnePulseMode(TIM2, LL_TIM_ONEPULSEMODE_SINGLE);
}

void step_right_motor(void)
{
	
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);      //do i really need this?
	//LL_TIM_SetOnePulseMode(TIM2, LL_TIM_ONEPULSEMODE_SINGLE);
	LL_TIM_EnableCounter(TIM2);
	

}
void step_left_motor(void)
{
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);     //do i really need this?
	     //do i really need this?
	//LL_TIM_SetOnePulseMode(TIM2, LL_TIM_ONEPULSEMODE_SINGLE);
	LL_TIM_EnableCounter(TIM2);
	

}