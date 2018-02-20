#include "i2c.h"
#include "stm32f4xx.h"
#include "main.h"

#define  ADDR_GROUND         0x90
#define  ADDR_V_PLUS 		     0x92
#define  ADDR_SDA            0x94
#define  ADDR_SDL            0x96

#define I2C_TMP116_READ_BIT  0x00
#define I2C_TMP116_WRITE_BIT 0x01


#define ADDR_TEMPERATURE     0x00
#define ADDR_CONFIGURATION   0x01
#define ADDR_HIGH_LIMIT      0x02
#define ADDR_LOW_LIMIT       0x03
#define ADDR_EEPROM_UNLOCK   0x04
#define ADDR_EEPROM_1        0x05
#define ADDR_EEPROM_2        0x06
#define ADDR_EEPROM_3        0x07
#define ADDR_EEPROM_4        0x08
#define ADDR_DEVICE_ID       0x0F

#define UNLOCK_BIT                  				0x8000

#define EEPROM_BUSY_MASK            				0x4000

#define GENERAL_RESET_CMD           				0x0006

#define RESET_MOD_MASK              				0xF3FF

#define SHUTDOWN_MOD_COMBINATION    				0x0800

#define CONTINUOUS_CONV_MOD_COMBINATION    	0x0020

#define DATA_IS_READY               				0x2000

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	


static void I2C_delay(uint32_t cnt)
{
	volatile uint32_t v = cnt;
	while(v--);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	

void I2C_configure(void)
{
	GPIO_InitTypeDef GPIO_InitializeI2C;
	I2C_InitTypeDef I2C_Initialize;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//I2C GPIO config
	GPIO_InitializeI2C.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitializeI2C.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitializeI2C.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitializeI2C.GPIO_OType = GPIO_OType_OD; 
	GPIO_InitializeI2C.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(GPIOB, &GPIO_InitializeI2C);
	
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);									
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2); 

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	
	//I2C config
	I2C_StructInit(&I2C_Initialize);
	I2C_Initialize.I2C_ClockSpeed = 100000;
	I2C_Initialize.I2C_Mode = I2C_Mode_I2C;
	I2C_Initialize.I2C_DutyCycle = I2C_DutyCycle_2;


	
	I2C_Init(I2C2, &I2C_Initialize);
	
	I2C_Cmd(I2C2, ENABLE);

}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	

int TMP116_writeConfiguration(uint16_t value)
{
	int result = 0;
	result = TMP116_writeRegister( &I2C2_Bus, ADDR_CONFIGURATION, value);
	return result;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	
uint16_t TMP116_readConfiguration(void)
{
	return I2C_RegRead16(&I2C2_Bus, ADDR_GROUND, ADDR_CONFIGURATION);	
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	


void TMP116_enterShutdownMode(void)
{
	uint16_t currentConfiguration;
	currentConfiguration = TMP116_readConfiguration();
	currentConfiguration = currentConfiguration & RESET_MOD_MASK;
	currentConfiguration = currentConfiguration | SHUTDOWN_MOD_COMBINATION;
	TMP116_writeConfiguration(currentConfiguration);
}

void TMP116_enterContinuousConversionMode(void)
{
	TMP116_writeConfiguration(CONTINUOUS_CONV_MOD_COMBINATION);
	delay_ms(2);
	delay_ms(125);
}
//-----------------------------------------------------------------------------
//currenty not used
//-----------------------------------------------------------------------------	

int TMP116_dataIsReady(void)
{
	uint16_t currentConfiguration;
	currentConfiguration = TMP116_readConfiguration();
	if((currentConfiguration & DATA_IS_READY )== DATA_IS_READY)
	{
		return 1; //ok
	}
	return 0;  //not ok
	
}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	

int TMP116_writeRegister( I2C_Bus *i2cDev, uint8_t addr, uint16_t value)
{
	uint16_t Status = 0x4000; 
	uint16_t old_value = value;
	
	I2C_RegWrite16(&I2C2_Bus, ADDR_GROUND, ADDR_EEPROM_UNLOCK, UNLOCK_BIT);
	
	I2C_RegWrite16(&I2C2_Bus, ADDR_GROUND, addr, value);
	
	while((Status & EEPROM_BUSY_MASK) == EEPROM_BUSY_MASK) //wait until EEPROM_Busy flag will be clear
	{
	delay_ms(7);
	
	Status = I2C_RegRead16(&I2C2_Bus, ADDR_GROUND, ADDR_EEPROM_UNLOCK);
		
	if (i2cDev->error) return -1;
	}
	//general call reset
	TMP116_generalCallResetCommand(&I2C2_Bus);
	delay_ms(7);
	
	if(old_value == I2C_RegRead16(&I2C2_Bus, ADDR_GROUND, addr))
	{
		return 0; //ok
	}
	else
	{
		return -1; //not ok
	}
}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	
uint16_t readTemperature(void)
{
	uint16_t temp;
	
	temp = I2C_RegRead16(&I2C2_Bus, ADDR_GROUND, ADDR_TEMPERATURE);
	
	return temp;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	
float TMP116_getTemperature(void)
{
	return (float)readTemperature()/128.0f;
	
}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	
uint16_t readID(void)
{
	uint16_t temp;
	
	temp = I2C_RegRead16(&I2C2_Bus, ADDR_GROUND, ADDR_DEVICE_ID);
	
	return temp;
}


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	

uint16_t readLowLimit(void)
{
	uint16_t temp;
	
	temp = I2C_RegRead16(&I2C2_Bus, ADDR_GROUND, ADDR_LOW_LIMIT);
	
	return temp;
}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	
uint16_t readHighLimit(void)
{
	uint16_t temp;
	
	temp = I2C_RegRead16(&I2C2_Bus, ADDR_GROUND, ADDR_HIGH_LIMIT);
	
	return temp;
}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	
I2C_Bus I2C1_Bus = { I2C1, 0 };
I2C_Bus I2C2_Bus = { I2C2, 0 };

/*
#define I2C_WaitForEvent(i2cDev, ev) do { \
		i2cDev->timeout_counter = SystemCoreClock / 1000; \
		while (!I2C_CheckEvent(i2cDev->I2Cx, ev) && !i2cDev->error && i2cDev->timeout_counter-- > 0); \
		if (i2cDev->error) return 0; 					\
		if (!i2cDev->timeout_counter) 					\
		{												\
			I2C_GenerateSTOP(i2cDev->I2Cx, ENABLE); 	\
			i2cDev->error = 3;							\
			return 0;									\
		}												\
	} while(0)*/

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	
int I2C_WaitForEvent(I2C_Bus *i2cDev, uint32_t ev)
{
	i2cDev->timeout_counter = SystemCoreClock / 1000;
	while (!I2C_CheckEvent(i2cDev->I2Cx, ev) && !i2cDev->error && i2cDev->timeout_counter-- > 0);
	if (i2cDev->error) 
		return 0; //not ok
	if (!i2cDev->timeout_counter)
	{
		I2C_GenerateSTOP(i2cDev->I2Cx, ENABLE);
		i2cDev->error = 3;
		return 0; //not ok
	}
	return 1; //ok 
}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	
/*
#define I2C_WaitWhileBusy(i2cDev) do { \
	i2cDev->timeout_counter = SystemCoreClock / 1000; \
	while (I2C_GetFlagStatus(i2cDev->I2Cx, I2C_FLAG_BUSY) && i2cDev->timeout_counter-- > 0); \
	if (!i2cDev->timeout_counter) 					\
	{												\
		i2cDev->error = 3;							\
		return 0;									\
	}												\
} while (0)
*/
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	
int I2C_WaitWhileBusy(I2C_Bus *i2cDev)
{
	i2cDev->timeout_counter = SystemCoreClock / 1000; 
	while (I2C_GetFlagStatus(i2cDev->I2Cx, I2C_FLAG_BUSY) && i2cDev->timeout_counter-- > 0);
	
	if (!i2cDev->timeout_counter)
	{
		i2cDev->error = 3;
		return 0; //not ok
	}
	return 1; //ok
}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------	

void I2C_ER_IRQHandler(I2C_Bus *i2cBus)
{
	I2C_TypeDef *i2c = i2cBus->I2Cx;
	unsigned int sr1 = i2c->SR1;
	unsigned int sr2 = i2c->SR2;
	if (sr1 & I2C_SR1_AF)
	{
		i2cBus->error = 1;
		I2C_ClearITPendingBit(i2c, I2C_IT_AF);
	}
	if (sr1 & I2C_SR1_ARLO)
	{
		i2cBus->error = 2;
		I2C_ClearITPendingBit(i2c, I2C_IT_ARLO);
	}
	if (sr1 & I2C_SR1_BERR)
	{
		I2C_ClearITPendingBit(i2c, I2C_IT_BERR);
	}
	if (sr1 & I2C_SR1_OVR)
	{
		I2C_ClearITPendingBit(i2c, I2C_IT_OVR);
	}
	I2C_GenerateSTOP(i2c, ENABLE);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandler(&I2C1_Bus);
}

void I2C2_ER_IRQHandler(void)
{
	I2C_ER_IRQHandler(&I2C2_Bus);
}

//****************************************************************************


//****************************************************************************
static int I2C_AcknowledgePolling(I2C_Bus *i2cDev, uint8_t I2C_Addr)
{
  vu16 SR1_Tmp;
  i2cDev->timeout_counter = SystemCoreClock / 1000;
  do
  {
    I2C_GenerateSTART(i2cDev->I2Cx, ENABLE);

    SR1_Tmp = I2C_ReadRegister(i2cDev->I2Cx, I2C_Register_SR1);

	I2C_Send7bitAddress(i2cDev->I2Cx, I2C_Addr, I2C_Direction_Transmitter);


  } while(!(I2C_ReadRegister(i2cDev->I2Cx, I2C_Register_SR1) & 0x0002) && i2cDev->timeout_counter-- > 0);

  I2C_ClearFlag(i2cDev->I2Cx, I2C_FLAG_AF);

  I2C_GenerateSTOP(i2cDev->I2Cx, ENABLE);

  if (i2cDev->timeout_counter <= 0) return -1;
  return 0;
}
//****************************************************************************
//dont changed yet
//****************************************************************************
int I2C_RegWrite(I2C_Bus *i2cDev, uint8_t deviceAddr, uint8_t addr, uint8_t value)
{
	int res;
	if (i2cDev->error) return -1;
	//while (I2C_GetFlagStatus(i2cDev->I2Cx, I2C_FLAG_BUSY));
	I2C_GenerateSTART(i2cDev->I2Cx, ENABLE);
	//while (!I2C_CheckEvent(i2cDev->I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(i2cDev->I2Cx, deviceAddr, I2C_Direction_Transmitter);

	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	//while (!I2C_CheckEvent(i2cDev->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && !i2c_error);
	//if (i2c_error) return;

	I2C_SendData(i2cDev->I2Cx, addr);
	//while(!I2C_CheckEvent(i2cDev->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

	I2C_SendData(i2cDev->I2Cx, value);
	//while(!I2C_CheckEvent(i2cDev->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

	I2C_GenerateSTOP(i2cDev->I2Cx, ENABLE);

	res = I2C_AcknowledgePolling(i2cDev, deviceAddr);

	I2C_delay(100);
	return res;
}
//****************************************************************************
//dont changed yet
//****************************************************************************
uint8_t I2C_RegRead(I2C_Bus *i2cDev, uint8_t deviceAddr, uint8_t addr)
{
	uint8_t value;
	if (i2cDev->error) return 0;
	I2C_WaitWhileBusy(i2cDev);

	I2C_AcknowledgeConfig(i2cDev->I2Cx, ENABLE);

	I2C_GenerateSTART(i2cDev->I2Cx, ENABLE);
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(i2cDev->I2Cx,  deviceAddr, I2C_Direction_Transmitter);
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

	I2C_SendData(i2cDev->I2Cx, addr);
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

	I2C_GenerateSTART(i2cDev->I2Cx, ENABLE);
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(i2cDev->I2Cx, deviceAddr, I2C_Direction_Receiver);
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

	I2C_AcknowledgeConfig(i2cDev->I2Cx, DISABLE);
	I2C_GenerateSTOP(i2cDev->I2Cx, ENABLE);

	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_BYTE_RECEIVED); /* EV7 */

	value = I2C_ReceiveData(i2cDev->I2Cx);

	I2C_AcknowledgeConfig(i2cDev->I2Cx, ENABLE);

	//I2C_GenerateSTOP(i2cDev->I2Cx, ENABLE);

	return value;
}
//****************************************************************************
int I2C_RegWrite16(I2C_Bus *i2cDev, uint8_t deviceAddr, uint8_t addr, uint16_t value)
{
	int res = 0;
	if (i2cDev->error) return -1;

	I2C_GenerateSTART(i2cDev->I2Cx, ENABLE); //SEND START
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_MODE_SELECT); //WAIT START sended
	
	I2C_Send7bitAddress(i2cDev->I2Cx, deviceAddr, I2C_Direction_Transmitter); //SEND DEVICE ADDR + WRITE
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);			//WAIT ACK by device
	
	I2C_SendData(i2cDev->I2Cx, addr);																					//SEND POINTER REG (register address)
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_BYTE_TRANSMITTED);							//WAIT ACK by device
	
	I2C_SendData(i2cDev->I2Cx, (uint8_t)(value >> 8));												//SEND MSb
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_BYTE_TRANSMITTED);							//WAIT ACK
	
	
	I2C_SendData(i2cDev->I2Cx, 	(uint8_t)value);															//SEND LSb
	I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_BYTE_TRANSMITTED);							//WAIT ACK
	
	
	I2C_GenerateSTOP(i2cDev->I2Cx, ENABLE);                                    // GENERATE STOP
	
	//res = I2C_AcknowledgePolling(i2cDev, deviceAddr);
	
	I2C_delay(100);
	return res;
}
//****************************************************************************
uint16_t I2C_RegRead16(I2C_Bus *i2cDev, uint8_t deviceAddr, uint8_t addr)
{
	uint16_t value;
	if (i2cDev->error) return 0;
	while(!I2C_WaitWhileBusy(i2cDev));
	
	I2C_AcknowledgeConfig(i2cDev->I2Cx, ENABLE);

	I2C_GenerateSTART(i2cDev->I2Cx, ENABLE);							//START
	while(!I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_MODE_SELECT));	//WAIT START
	
	I2C_Send7bitAddress(i2cDev->I2Cx,  deviceAddr, I2C_Direction_Transmitter);			//SEND ADDR W
	while(!I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	//SEND ADDR W
	
	I2C_SendData(i2cDev->I2Cx, addr);													//SEND REG
	while(!I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_BYTE_TRANSMITTED));			//SEND REG
	
	I2C_GenerateSTART(i2cDev->I2Cx, ENABLE);											//REPEATED START
	while(!I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_MODE_SELECT));					//REPEATED START
	
	I2C_Send7bitAddress(i2cDev->I2Cx, deviceAddr, I2C_Direction_Receiver);				//SEND ADDR R
	while(!I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));		//SEND ADDR R
	
	while(!I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_BYTE_RECEIVED)); /* EV7 */		//WAIT READ
	value = (uint16_t)I2C_ReceiveData(i2cDev->I2Cx) << 8;											//READ DATA
	
	I2C_AcknowledgeConfig(i2cDev->I2Cx, DISABLE);									//ACK=0
	I2C_GenerateSTOP(i2cDev->I2Cx, ENABLE);												//STOP
	
	while(!I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_BYTE_RECEIVED)); /* EV7 */		//WAIT READ
	value |= (uint16_t)I2C_ReceiveData(i2cDev->I2Cx);												//READ DATA
	
	return value;
	
}
//****************************************************************************


//****************************************************************************
int TMP116_generalCallResetCommand(I2C_Bus *i2cDev)																						//tmp116 function  general call reset command
{
	if (i2cDev->error) return 0;
	while(!I2C_WaitWhileBusy(i2cDev));
	
	I2C_GenerateSTART(i2cDev->I2Cx, ENABLE);							//START
	while(!I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_MODE_SELECT));	//WAIT START
	
	I2C_Send7bitAddress(i2cDev->I2Cx,  0x00, I2C_Direction_Transmitter);			//SEND ADDR 00000000 W
	while(!I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));			//WAIT ACK
	
	I2C_SendData(i2cDev->I2Cx, GENERAL_RESET_CMD);													//SEND GENERAL RESET COMMAND
	while(!I2C_WaitForEvent(i2cDev, I2C_EVENT_MASTER_BYTE_TRANSMITTED));			//WAIT ACK
	
	I2C_GenerateSTOP(i2cDev->I2Cx, ENABLE);                                    // GENERATE STOP
	
	return 0;
}


//****************************************************************************
