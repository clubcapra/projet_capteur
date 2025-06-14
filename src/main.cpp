#include <Arduino.h>
#include <Wire.h>
#include "SparkFunBME280.h"
#include "SparkFun_SCD4x_Arduino_Library.h"
#include "STM32_CAN.h"
#include <i2c_adc_ads7828.h>

#define LED_PIN PC12 // Définit la broche de la LED
TwoWire myWire(PB7, PB6);

ADS7828 device(3, SINGLE_ENDED | REFERENCE_ON | ADC_ON, 0x0F);
ADS7828 *adc = &device;
ADS7828Channel *MQ7 = adc->channel(0);
ADS7828Channel *SEN_094 = adc->channel(1);

SCD4x SCD41_Sensor;

BME280 BME280_Sensor;

STM32_CAN Can(CAN1, DEF); // PA11/12 pins for CAN1.
static CAN_message_t CAN_RX_msg;
static CAN_message_t CAN_TX_msg;

void setup()
{

    pinMode(LED_PIN, OUTPUT); // Configure PC12 comme sortie

    Can.begin();
    Can.setBaudRate(500000); // 500KBPS

    myWire.begin();

    if (SCD41_Sensor.begin(myWire) == false)
    {
        while (1)
        {
            // peut-etre envoyer un message sur le bus CAN si le SCD41 n'arrive pas a s'initialiser.
        }
    }

    BME280_Sensor.setI2CAddress(0x77);
    if (BME280_Sensor.beginI2C(myWire) == false) // Begin communication over I2C
    {
        while (1)
        {
            // peut-etre envoyer un message sur le bus CAN si le BME280 n'arrive pas a s'initialiser.
        }
    }

    // adjust scaling on an individual channel basis
    MQ7->minScale = 30;
    MQ7->maxScale = 3000;

    SEN_094->minScale = 300;
    SEN_094->maxScale = 10000;
}

void loop()
{

    
}
