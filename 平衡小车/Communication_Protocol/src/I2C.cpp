#include "stm32f10x.h"  
#include "Delay.h"


class I2C
{
private:
    /* data */
    uint32_t RCC_APB2Periph;
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin_SCL;//SCL
    uint16_t GPIO_Pin_SDA;//SDA
    GPIOSpeed_TypeDef GPIO_Speed; 
    GPIOMode_TypeDef GPIO_Mode; 
public:
    I2C(char X,uint16_t GPIO_Pin);
    ~I2C();
    void MyI2C_Init(void);
    void I2C_W_SCL(uint8_t BitValue);
    void I2C_W_SDA(uint8_t BitValue);
    uint8_t I2C_R_SDA(void);
    void I2C_Start(void);
    void I2C_Stop(void);
    void I2C_SendByte(uint8_t Byte);
    uint8_t I2C_ReceiveByte(void);
    void I2C_SendAck(uint8_t AckBit);
    uint8_t I2C_ReceiveAck(void);

};

I2C::I2C(uint32_t RCC_APB2Periph,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_SCL,\
        uint16_t GPIO_Pin_SDA,GPIOSpeed_TypeDef GPIO_Speed,GPIOMode_TypeDef GPIO_Mode)
{
    RCC_APB2Periph = RCC_APB2Periph;
    GPIOx = GPIOx;
    GPIO_Pin_SCL = GPIO_Pin_SCL;
    GPIO_Pin2 = GPIO_Pin_SDA;
    GPIO_Speed = GPIO_Speed;
    GPIO_Mode = GPIO_Mode;  
}

I2C::~I2C()
{
}

void I2C::I2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SCL | GPIO_Pin2_SDA;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	GPIO_SetBits(GPIOx, GPIO_Pin_SCL | GPIO_Pin2_SDA);
}

void I2C::I2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOx, GPIO_Pin_SCL, (BitAction)BitValue);
	Delay_us(10);
}

void I2C::I2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOx, GPIO_Pin_SDA, (BitAction)BitValue);
	Delay_us(10);
}

uint8_t I2C::I2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin_SDA);
	Delay_us(10);
	return BitValue;
}

void I2C::I2C_Start(void)
{
	I2C_W_SDA(1);
	I2C_W_SCL(1);
	I2C_W_SDA(0);
	I2C_W_SCL(0);
}

void I2C::I2C_Stop(void)
{
	I2C_W_SDA(0);
	I2C_W_SCL(1);
	I2C_W_SDA(1);
}

void I2C::I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		I2C_W_SDA(Byte & (0x80 >> i));
		I2C_W_SCL(1);
		I2C_W_SCL(0);
	}
}

uint8_t I2C::I2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	I2C_W_SDA(1);
	for (i = 0; i < 8; i ++)
	{
		I2C_W_SCL(1);
		if (I2C_R_SDA() == 1){Byte |= (0x80 >> i);}
		I2C_W_SCL(0);
	}
	return Byte;
}

void I2C::I2C_SendAck(uint8_t AckBit)
{
	I2C_W_SDA(AckBit);
	I2C_W_SCL(1);
	I2C_W_SCL(0);
}

uint8_t I2C::I2C_ReceiveAck(void)
{
	uint8_t AckBit;
	I2C_W_SDA(1);
	I2C_W_SCL(1);
	AckBit = I2C_R_SDA();
	I2C_W_SCL(0);
	return AckBit;
}

