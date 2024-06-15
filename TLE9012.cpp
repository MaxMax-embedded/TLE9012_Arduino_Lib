/*
MIT License

Copyright (c) 2024 Maximilian Mönikes

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#include "TLE9012.h"

HardwareSerial* hisoUART;


TLE9012::TLE9012()  //Constructor
{

}


TLE9012::~TLE9012() //Destructor
{

}
      

      
      
void TLE9012::init(HardwareSerial* serial, uint32_t baudrate=1000000)//Driver init
{
  hisoUART = serial;
  hisoUART->begin(baudrate,SERIAL_8N1,16,17);
}

void TLE9012::wakeUp()
{

  iso_uart_status_t status;
  status = isoUART_TIMEOUT;

  uint8_t wakeupbuffer[2];
  wakeupbuffer[0] = 0xAA;
  wakeupbuffer[1] = 0xAA;
  wakeupbuffer[2] = 0xAA;


  isoUARTClearRXBUffer();
  hisoUART->write(wakeupbuffer,2);

  uint32_t starttime = millis();
  while((millis()-starttime) < ISOUART_TIMEOUT)
  {
    if(hisoUART->available() > 1)
    {
      status = isoUART_OK;
      break;
    }
  }
}
      
      //Low Level Routines for direct register Access
iso_uart_status_t TLE9012::readRegisterSingle(uint8_t nodeID, uint16_t regaddress, uint16_t* result)
{
  iso_uart_status_t status;
  uint8_t response_buffer[9];
  
  status = isoUART_TIMEOUT;

  isoUARTClearRXBUffer();
  isoUARTReadRequest(nodeID, regaddress);

  uint32_t starttime = millis();
  while((millis()-starttime) < ISOUART_TIMEOUT)
  {
    if(hisoUART->available() > 8)
    {
      hisoUART->readBytes(response_buffer,9);
      status = isoUART_OK;
      break;
    }
  }

  //Check if Timeout occured
  if(status != isoUART_OK)
  {
    status = isoUART_TIMEOUT;
    return status;
  }

  
  msb_first_converter(&(response_buffer[4]),5);
  uint8_t crc = crc8(&response_buffer[4],5);

  *result = (((uint16_t) response_buffer[6])<<8) | ((uint16_t) response_buffer[7]);

  if(crc != response_buffer[8]);
    //status = isoUART_CRC_ERROR;

  return status;
}

iso_uart_status_t TLE9012::writeRegisterSingle(uint8_t nodeID, uint16_t regaddress, uint16_t databuffer) //Write data to a single register
{

  iso_uart_status_t status;
  uint8_t response_buffer[7];
  
  status = isoUART_TIMEOUT;

  isoUARTClearRXBUffer();
  isoUARTWriteRequest(nodeID, regaddress, databuffer);
  
  uint32_t starttime = millis();
  while((millis()-starttime) < ISOUART_TIMEOUT)
  {
    if(hisoUART->available() > 6)
    {
      hisoUART->readBytes(response_buffer,7);
      status = isoUART_OK;
      break;
    }
  }

  //Check if Timeout occured
  if(status != isoUART_OK)
  {
    status = isoUART_TIMEOUT;
    return status;
  }  

  return status;
}

iso_uart_status_t TLE9012::readRegisterBroadcast(uint16_t regaddress, uint16_t* result) //Write a broadcast to all devices in the daisy chain
{
  iso_uart_status_t status;
  uint8_t response_buffer[N_DEVICES*5+4];
  
  status = isoUART_TIMEOUT;

  isoUARTClearRXBUffer();
  isoUARTReadRequest(BROADCAST_ID, regaddress);

  uint32_t starttime = millis();
  while((millis()-starttime) < ISOUART_TIMEOUT)
  {
    if(hisoUART->available() > (3+(5*N_DEVICES)))
    {
      hisoUART->readBytes(response_buffer,4+5*N_DEVICES);
      status = isoUART_OK;
      break;
    }
  }

  //Check if Timeout occured
  if(status != isoUART_OK)
  {
    status = isoUART_TIMEOUT;
    return status;
  }
  
  msb_first_converter(&response_buffer[4],N_DEVICES*5);

  for(uint8_t n = 0; n < N_DEVICES; n++)
  {
    uint8_t crc = crc8(&response_buffer[4+(n*N_DEVICES)],5);
    result[n] = (((uint16_t) response_buffer[6+(n*N_DEVICES)])<<8) | ((uint16_t) response_buffer[7+(n*N_DEVICES)]);
    if(crc != 0)
      status = isoUART_CRC_ERROR;
  }
  

  return status;
}

iso_uart_status_t TLE9012::writeRegisterBroadcast(uint16_t regaddress, uint16_t databuffer) //Read a register as broadcast from all devices in the chain
{

  iso_uart_status_t status;
  uint8_t response_buffer[7];
  
  status = isoUART_TIMEOUT;

  isoUARTClearRXBUffer();
  isoUARTWriteRequest(BROADCAST_ID, regaddress, databuffer);
  uint32_t starttime = millis();
  while((millis()-starttime) < ISOUART_TIMEOUT)
  {
    if(hisoUART->available() > 6)
    {
      hisoUART->readBytes(response_buffer,7);
      status = isoUART_OK;
      break;
    }
  }

  //Check if Timeout occured
  if(status != isoUART_OK)
  {
    status = isoUART_TIMEOUT;
    return status;
  }

  return status;
}

iso_uart_status_t TLE9012::configureMultiread(multiread_cfg_t cfg)  //Write a multiread configuration to all devices in the daisy chain
{

}

iso_uart_status_t TLE9012::multiRead(multread_result_t* databuffer) //Multiread command from all devices in the chain
{

}

//Private Functions start here
uint8_t TLE9012::crc8(uint8_t* buffer, uint16_t len)
{
  const uint8_t polynomial = 0x1D;
  uint8_t crc = 0xFF;

  for(uint16_t i = 0; i < len; i++)
  {
    crc ^= buffer[i];

    for(uint8_t j = 0;  j < 8; j++)
    {
      if((crc & 0x80) != 0)
      {
        crc = (crc<<1)^polynomial;
      }
      else
        crc = crc << 1;
    }
  }

  return crc^0xFF;
}

uint8_t TLE9012::msb_first_converter(uint8_t* data, uint16_t len)
{
  for(uint16_t n = 0;  n < len; n++)
  {
    data[n] = ((data[n] & 0xF0) >> 4) | ((data[n] & 0x0F) << 4);
    data[n] = ((data[n] & 0xCC) >> 2) | ((data[n] & 0x33) << 2);
    data[n] = ((data[n] & 0xAA) >> 1) | ((data[n] & 0x55) << 1);
  }
  return 0;
}

void TLE9012::isoUARTWriteRequest(uint8_t nodeID, uint8_t regaddress, uint16_t data)
{
  uint8_t writebuffer[6];
  writebuffer[0] = 0x1E; //Syncframe
  writebuffer[1] = nodeID |  WRITECOMMAND;
  writebuffer[2] = regaddress;
  writebuffer[3] = (uint8_t)(data >> 8);
  writebuffer[4] = (uint8_t)(data & 0xFF);
  writebuffer[5] = 0x00;

  writebuffer[5] = crc8(writebuffer,5);
  msb_first_converter(writebuffer, 6);

  hisoUART->write(writebuffer,6);
}

void TLE9012::isoUARTReadRequest(uint8_t nodeID, uint8_t regaddress)
{
  uint8_t writebuffer[4];
  writebuffer[0] = 0x1E;
  writebuffer[1] = nodeID;
  writebuffer[2] = regaddress;
  writebuffer[3] = 0x00;

  writebuffer[3] = crc8(writebuffer,3);
  msb_first_converter(writebuffer, 4);
  hisoUART->write(writebuffer,4);
}

void TLE9012::isoUARTClearRXBUffer()
{
  while(hisoUART->available())
    uint8_t null = hisoUART->read();
}