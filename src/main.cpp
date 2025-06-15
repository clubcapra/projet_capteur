/*
Projet capteur
Auteur : Thomas Rousseau-Sénécal
Date : 14/06/2025
Propriétaire : CAPRA Robotique - École de technologie supérieure

Ce code est destiner au STM32F44RET6 du projet capteur.

La logic du code est la suivente :

lorsque le STM32 reçois une trame CAN dont le ID est 1A4,
le STM32 comprend que le données de la trame lui sont destiner.

Pour obtenir les valeurs des capteurs, l'utilisateur doit envoyer
une trame contenant 6 octets, chaque octet représentant une données,
l'utilisateur doit mettre la valeur 0x11 a l'octet dont correspond
la données qu'il souhaite récupéré et 0x00 aux octet correspondant
aux données non désirer :

    Octet #0 : Méthane ppm
    Octet #1 : Co2 ppm
    Octet #2 : Co ppm
    Octet #3 : Température °C
    Octet #4 : Humidité %
    Octet #5 : Pression atmospérique kPa

Exemple:    l'utilisateur veut le méthane et la température, donc le champ
            de donner de sa trame doit être {0x11, 0x00, 0x00, 0x11, 0x00, 0x00}

Le STM32 répond de cette façon :
    Trame #1 :
        ID : 0x1A5
        Octet #0 : Méthane LSB
        Octet #1 : Méthane MSB
        Octet #2 : Co2 LSB
        Octet #3 : Co2 MSB
        Octet #4 : Co LSB
        Octet #5 : Co MSB
        Octet #6 : Température
        Octet #7 : Humidité

    Trame #2
        ID : 0x1A6
        Octet #0 : Pression atmospérique

La deuxième trame CAN sera envoyer uniquement si l'utilisateur demande a recevoir la pression atmosphérique.

Remarques :
- Si l'utilisateur veut uniquement avoir certaine données, exemple le méthane et la température, les autres octets
  de la trame auront 0xFF comme valeur
- La pression est exprimée en kilopascals (kPa),
- Les données transmises sont envoyées en tant que "uint8_t" :
  Exemple : le méthane est mesuré à 300 ppm  
  Valeur en hexadécimal : 0x012C  
  Encodée sur deux octets :
     - Octet LSB (basse valeur) : 0x2C (44 en décimal)
     - Octet MSB (haute valeur) : 0x01 (1 en décimal)
  Trame CAN contiendra donc : [0x2C, 0x01] pour le méthane
*/

// Librairies
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include "SparkFunBME280.h"
#include "SparkFun_SCD4x_Arduino_Library.h"
#include "STM32_CAN.h"
#include <i2c_adc_ads7828.h>
#include <iostream>
#include <cstdint>
#include <iomanip>

/*
 * Fonction : encoder_float_entier
 * But : Convertit un float en un uint8_t en gardant uniquement la partie entière
 * Paramètres :
 *    - valeur : la valeur flottante à convertir
 *    - memoire : tableau dans lequel stocker l'octet résultant
 *    - index : position actuelle dans le tableau (sera automatiquement incrémentée)
 */
void encoder_float_entier(float valeur, uint8_t *memoire, size_t &index)
{
    uint8_t entier = static_cast<uint8_t>(valeur); // partie entière seulement
    memoire[index++] = entier;
}

/*
 * Fonction : encoder_uint16
 * But : Encode un entier 16 bits (uint16_t) en deux octets (MSB + LSB)
 * Paramètres :
 *    - valeur : la valeur entière 16 bits à convertir
 *    - memoire : tableau dans lequel stocker les deux octets
 *    - index : position actuelle dans le tableau (sera automatiquement incrémentée de 2)
 */
void encoder_uint16(uint16_t valeur, uint8_t *memoire, size_t &index)
{
    memoire[index++] = static_cast<uint8_t>(valeur >> 8);   // octet haut (MSB)
    memoire[index++] = static_cast<uint8_t>(valeur & 0xFF); // octet bas (LSB)
}

// Définition de la broche de la LED de statut
#define LED_PIN PC12

// Initialisation du bus I2C avec des broches spécifiques
TwoWire myWire(PB7, PB6);

// Configuration des canaux ADC pour les capteurs analogiques
ADS7828 device(3, SINGLE_ENDED | REFERENCE_ON | ADC_ON, 0x0F);
ADS7828 *adc = &device;
ADS7828Channel *MQ7 = adc->channel(0);     // Capteur de CO
ADS7828Channel *SEN_094 = adc->channel(1); // Capteur de méthane

// Déclaration des objets pour les capteurs numériques
SCD4x SCD41_Sensor;
BME280 BME280_Sensor;

// Configuration CAN sur le STM32
STM32_CAN Can(CAN1, DEF); // PA11/12 pins pour CAN1
static CAN_message_t CAN_RX_msg;
static CAN_message_t CAN_TX_msg;

void setup()
{
    pinMode(LED_PIN, OUTPUT); // LED pour le statut système

    // Initialisation du CAN à 500 kbps
    Can.begin();
    Can.setBaudRate(500000);

    // Initialisation du bus I2C
    myWire.begin();

    // Démarrage du capteur CO2 SCD41
    if (SCD41_Sensor.begin(myWire) == false)
    {
        while (1)
        {
            // Possibilité : envoyer un message CAN d’erreur si init échoue
        }
    }

    // Démarrage du capteur BME280 (pression/température/humidité)
    BME280_Sensor.setI2CAddress(0x77);
    if (BME280_Sensor.beginI2C(myWire) == false)
    {
        while (1)
        {
            // Possibilité : envoyer un message CAN d’erreur si init échoue
        }
    }

    // Ajustement de l’échelle de mesure des capteurs analogiques
    MQ7->minScale = 30;
    MQ7->maxScale = 3000;
    SEN_094->minScale = 300;
    SEN_094->maxScale = 10000;
}

void loop()
{
    // Vérifie si un message CAN a été reçu
    if (Can.read(CAN_RX_msg))
    {
        // Vérifie que le message est bien destiné à ce module (ID 0x1A4 attendu)
        if (CAN_RX_msg.id == 0x1A4)
        {

            // Tableau pour stocker les données encodées (jusqu’à 9 octets)
            uint8_t donnees[9] = {0};
            size_t index = 0;

            // Parcours des octets de la trame CAN reçue
            for (int i = 0; i < CAN_RX_msg.len; i++)
            {
                // Si l'utilisateur a mis 0x11 pour cette donnée, on lit et encode
                if (CAN_RX_msg.buf[i] == 0x11)
                {
                    switch (i)
                    {
                    case 0:
                    {
                        uint16_t ch4 = SEN_094->value();
                        encoder_uint16(ch4, donnees, index);
                        break;
                    }
                    case 1:
                    {
                        uint16_t co2 = SCD41_Sensor.getCO2();
                        encoder_uint16(co2, donnees, index);
                        break;
                    }
                    case 2:
                    {
                        uint16_t co = MQ7->value();
                        encoder_uint16(co, donnees, index);
                        break;
                    }
                    case 3:
                    {
                        float temp = (BME280_Sensor.readTempC() + SCD41_Sensor.getTemperature()) / 2;
                        encoder_float_entier(temp, donnees, index);
                        break;
                    }
                    case 4:
                    {
                        float hum = (BME280_Sensor.readFloatHumidity() + SCD41_Sensor.getHumidity()) / 2;
                        encoder_float_entier(hum, donnees, index);
                        break;
                    }
                    case 5:
                    {
                        float press_pa = BME280_Sensor.readFloatPressure();
                        float press_kpa = press_pa / 1000.0f;
                        encoder_float_entier(press_kpa, donnees, index);
                        break;
                    }
                    default:
                        break;
                    }
                }
                else
                {
                    // Si la donnée n'est pas demandée, on remplit le bon nombre d'octets avec 0xFF
                    if (i <= 2 && index + 1 < sizeof(donnees))
                    {
                        donnees[index++] = 0xFF;
                        donnees[index++] = 0xFF;
                    }
                    else if (index < sizeof(donnees))
                    {
                        donnees[index++] = 0xFF;
                    }
                }
            }

            // Envoi de la première trame (0x1A5) : jusqu’à 8 octets
            CAN_TX_msg.id = 0x1A5;
            CAN_TX_msg.len = 8;
            for (int j = 0; j < 8; j++)
            {
                CAN_TX_msg.buf[j] = donnees[j];
            }
            Can.write(CAN_TX_msg);

            // Envoi de la deuxième trame (0x1A6) uniquement si la pression a été encodée
            if (donnees[8] != 0)
            {
                CAN_TX_msg.id = 0x1A6;
                CAN_TX_msg.len = 1;
                CAN_TX_msg.buf[0] = donnees[8];
                Can.write(CAN_TX_msg);
            }
        }
    }
}
