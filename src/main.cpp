/*----------------------------
S. M. 2023
----------------------------*/
/*
TWCR = Control Register
    TWINT = Interrupt Flag
    TWEA  = Enable Acknowledge Bit
    TWSTA = Start Condition Bit
    TWSTO = Stop Condition Bit
    TWEN  = Enable Bit

TWDR = Data Register
    In Transmit mode, TWDR contains the next byte to be transmitted
    In Receive mode, the TWDR contains the last byte received
*/

#include <Arduino.h>
#include <avr/io.h>

#define CTRL_REG1 0x26
#define PT_DATA_CFG 0x13

uint8_t SlaveAddress = 0xC0;
uint8_t SlaveAddressRead = 0xC1;

uint8_t STA = 0;
uint8_t OUT_P_MSB = 0;
uint8_t OUT_P_CSB = 0;
uint8_t OUT_P_LSB = 0;
uint8_t OUT_T_MSB = 0;
uint8_t OUT_T_LSB = 0;

//Send Start condition
void IIC_Start(); 

//Write SLA+R address and read byte from TWDR
uint8_t IIC_Read(uint8_t d_address, uint8_t var);

//Write SLA+W address, register address and data to slawe
void IIC_Write(uint8_t d_address, uint8_t r_address, uint8_t data);

//Write SLA+W address, register address where data will be read and do repeated start
void IIC_WriteToRead(uint8_t d_address, uint8_t r_address);

//Convert received bytes to altitude
float altitudeConvert(uint8_t high, uint8_t mid, uint8_t low);

//Convert received bytes to temperature
float temperatureConvert(uint8_t high, uint8_t low);

//Send Stop condition
void IIC_Stop();

void setup()
{
    Serial.begin(9600);
    IIC_Write(SlaveAddress, CTRL_REG1, 0xB8); //Set altimeter with an OSR = 128

    IIC_Write(SlaveAddress, PT_DATA_CFG, 0x07); //Enable data flags in PT_DATA_CFG

    IIC_Write(SlaveAddress, CTRL_REG1, 0xBA); //OST, OSR = 128, ALT mode, 
}

void loop()
{
    setup();
    do{
        STA = IIC_Read(SlaveAddressRead, 0x00);
    } while (!(STA&0x08)); //wait for data to be ready

    //Read UOT_P and OUT_T
    uint8_t OUT_P_MSB = IIC_Read(SlaveAddressRead, 0x01); //read altitude high

    uint8_t OUT_P_CSB = IIC_Read(SlaveAddressRead, 0x02); //read altitude mid

    uint8_t OUT_P_LSB = IIC_Read(SlaveAddressRead, 0x03); //read altitude low/fractional bits

    uint8_t OUT_T_MSB = IIC_Read(SlaveAddressRead, 0x04); //read temperature high

    uint8_t OUT_T_LSB = IIC_Read(SlaveAddressRead, 0x05); //read temperature low

    float altitude = altitudeConvert(OUT_P_MSB, OUT_P_CSB, OUT_P_LSB);
    float temperature = temperatureConvert(OUT_T_MSB, OUT_T_LSB);

    Serial.print("Altitude: ");
    Serial.println(altitude);

    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.println();
    delay(100);
}

void IIC_Start()
{
    //Clear TWI interrupt flag, Put start condition on SDA, Enable TWI
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); 
    while(!(TWCR & (1<<TWINT))); //ack
}

void IIC_Write(uint8_t d_address, uint8_t r_address, uint8_t data)
{
    IIC_Start();
    //device address to address/data shift register
    TWDR = d_address;
    TWCR = ((1<< TWINT) | (1<<TWEN));
    while (!(TWCR & (1 <<TWINT))); //ack

    //register address to address/data shift register
    TWDR = r_address;
    TWCR = ((1<< TWINT) | (1<<TWEN));
    while (!(TWCR & (1 <<TWINT))); //ack

    //data to address/data shift register
    TWDR = data;
    TWCR = ((1<< TWINT) | (1<<TWEN));
    while (!(TWCR & (1 <<TWINT))); //ack
    IIC_Stop();

}

void IIC_WriteToRead(uint8_t d_address, uint8_t r_address)
{
    IIC_Start();
    //device address to address/data shift register
    TWDR = d_address;
    TWCR = ((1<< TWINT) | (1<<TWEN));
    while (!(TWCR & (1 <<TWINT))); //ack

    //register address, where data will be read, to address/data shift register
    TWDR = r_address;
    TWCR = ((1<< TWINT) | (1<<TWEN));
    while (!(TWCR & (1 <<TWINT))); //ack

    IIC_Start(); //Repeated Start
}

uint8_t IIC_Read(uint8_t d_address, uint8_t r_address)
{
    IIC_WriteToRead(SlaveAddress, r_address);

    //device address/read mode to address/data shift register
    TWDR = d_address;
    TWCR = ((1<< TWINT) | (1<<TWEN));
    while (!(TWCR & (1 <<TWINT))); //ack

    TWCR = ((1<< TWINT) | (1<<TWEN));
    while ( !(TWCR & (1 <<TWINT))); //nack

    IIC_Stop();
    return TWDR;
}

void IIC_Stop()
{
    TWCR = ((1<< TWINT) | (1<<TWEN) | (1<<TWSTO));
}

float altitudeConvert(uint8_t high, uint8_t mid, uint8_t low)
{   //16 integer bits including signed bit, fractional bits
    float tmpfract = 0;
    float result = 0;
    uint16_t tmpint = (high<<8)|mid; //bit shifting high bits and adding mid bits 

    if(tmpint <= 0x7FFF){
        tmpfract = low/240.0;
        result = tmpint+tmpfract;
    }
    else if(tmpint > 0x7FFF){
        tmpfract = low /240.0;
        result = tmpint + tmpfract - 65536;
    }
    return result;
}

float temperatureConvert(uint8_t high, uint8_t low)
{   //8 integer bits including signed bit, 4 fractional bits
    float result = 0;
    float temptmp = low/240.0;
    /* or do like this
    uint8_t lowtmp = (low>>4);
    float temptmp = lowtmp/15.0; */
    if(high <= 0x7F)
    {
        result = high + temptmp;
    }
    else if(high > 0x7F)
    {
        result = high + temptmp -256;
    }
    return result;
}