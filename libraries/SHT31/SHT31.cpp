/*************************************************** 
  This is a library for the SHT31 Digital Humidity & Temp Sensor

  Designed specifically to work with the SHT31 Digital sensor from Adafruit
  ----> https://www.adafruit.com/products/2857

  These sensors use I2C to communicate, 2 pins are required to interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/


#include "SHT31.h"

// SHT31::SHT31(int8_t SDApin, int8_t SCLpin) 
// {
	// _sda=SDApin;
	// _scl=SCLpin;
// }


boolean SHT31::begin(uint8_t i2caddr) {
  //Wire.begin(_sda, _scl);
  _i2caddr = i2caddr;
  reset();
  clearStatus();
  heater(false);
  return (readStatus() == 32784);
  }

uint16_t SHT31::readStatus(void) 
{
  writeCommand(SHT31_READSTATUS);
  Wire.requestFrom(_i2caddr, (uint8_t)3);
   if (Wire.available() != 3) 
   {
   return 999;
   }
  uint16_t stat;
  stat = Wire.read()<< 16;
  stat |= Wire.read()<<8;
  stat |= Wire.read();
  //Serial.println(stat, HEX);
  return stat;
}

void SHT31::reset(void) {
  writeCommand(SHT31_SOFTRESET);
  	  #ifdef ARDUINO_ESP32_DEV
  vTaskDelay( 10 / portTICK_PERIOD_MS);
#else
  delay(10);
#endif
}

void SHT31::clearStatus(void) {
  writeCommand(SHT31_CLEARSTATUS);
  	  #ifdef ARDUINO_ESP32_DEV
  vTaskDelay( 10 / portTICK_PERIOD_MS);
#else
  delay(10);
#endif
}

void SHT31::heater(boolean h) {
  if (h)
    writeCommand(SHT31_HEATEREN);
  else
    writeCommand(SHT31_HEATERDIS);
}


float SHT31::readTemperature(void) {
  if (! readTempHum()) return -999;

  return temp;
}
  

float SHT31::readHumidity(void) {
  if (! readTempHum()) return -999;

  return humidity;
}


boolean SHT31::readTempHum(void) {
  uint8_t readbuffer[6];

  writeCommand(SHT31_MEAS_HIGHREP);
  	  #ifdef ARDUINO_ESP32_DEV
  vTaskDelay( 500 / portTICK_PERIOD_MS);
#else
  delay(500);
#endif
  Wire.requestFrom(_i2caddr, (uint8_t)6);
  if (Wire.available() != 6) 
    return false;
  for (uint8_t i=0; i<6; i++) {
    readbuffer[i] = Wire.read();
  //  Serial.print("0x"); Serial.println(readbuffer[i], HEX);
  }
  uint16_t ST, SRH;
  ST = readbuffer[0];
  ST <<= 8;
  ST |= readbuffer[1];

  if (readbuffer[2] != crc8(readbuffer, 2)) return false;

  SRH = readbuffer[3];
  SRH <<= 8;
  SRH |= readbuffer[4];

  if (readbuffer[5] != crc8(readbuffer+3, 2)) return false;

 // Serial.print("ST = "); Serial.println(ST);
  double stemp = ST;
  stemp *= 175;
  stemp /= 0xffff;
  stemp = -45 + stemp;
  temp = stemp;
  
//  Serial.print("SRH = "); Serial.println(SRH);
  double shum = SRH;
  shum *= 100;
  shum /= 0xFFFF;
  
  humidity = shum;
  
  return true;
}

void SHT31::writeCommand(uint16_t cmd) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  Wire.endTransmission();  
}

uint8_t SHT31::crc8(const uint8_t *data, int len)
{
/*
*
 * CRC-8 formula from page 14 of SHT spec pdf
 *
 * Test data 0xBE, 0xEF should yield 0x92
 *
 * Initialization data 0xFF
 * Polynomial 0x31 (x8 + x5 +x4 +1)
 * Final XOR 0x00
 */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);
  
  for ( int j = len; j; --j ) {
      crc ^= *data++;

      for ( int i = 8; i; --i ) {
	crc = ( crc & 0x80 )
	  ? (crc << 1) ^ POLYNOMIAL
	  : (crc << 1);
      }
  }
  return crc;
}

boolean SHT31::readSerialNo(uint32_t &serialNo)
{
  uint8_t buf[6];

 writeCommand(SHT31_READ_SERIALNO);
 	  #ifdef ARDUINO_ESP32_DEV
  vTaskDelay( 500 / portTICK_PERIOD_MS);
#else
  delay(500);
#endif

  Wire.requestFrom(_i2caddr, (uint8_t)6);

 if ( Wire.available() == 6 )
 {
   for (uint8_t i = 0; i < 6; ++i )
      buf[i] = Wire.read();

    if ( buf[2] == crc8(&buf[0], 2)
          && buf[5] == crc8(&buf[3], 2) )
     {
     serialNo = ((uint32_t)buf[0] << 24)
                 | ((uint32_t)buf[1] << 16)
                  | ((uint32_t)buf[3] << 8)
                 | (uint32_t)buf[4];

      return true;
     }
  }

  return false;
}

/*********************************************************************/
