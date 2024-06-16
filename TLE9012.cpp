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
#include "TLE9012_Makros.h"

HardwareSerial* hisoUART;

/*
Constructor for TLE9012 class. Unused for now
*/
TLE9012::TLE9012()  //Constructor
{

}

/*
Destructor for TLE9012 class. Unused for now
*/
TLE9012::~TLE9012() //Destructor
{

}
      

/*

*/      
      
void TLE9012::init(HardwareSerial* serial, uint32_t baudrate=1000000,uint8_t rxpin=0,uint8_t txpin=0)//Driver init
{
  hisoUART = serial;
  if((rxpin != 0) && (txpin != 0))
    hisoUART->begin(baudrate,SERIAL_8N1,rxpin,txpin);
  else
    hisoUART->begin(baudrate,SERIAL_8N1);
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

     //High Level Routines

      //Measurement related functions
  void TLE9012::readCellVoltages(uint8_t nodeID)
  {

  }

  void TLE9012::readTemperatures(uint8_t nodeID)
  {

  }

  void TLE9012::setNumberofCells(uint8_t nodeID, uint8_t n_cells)
  {

  }

  void TLE9012::setNumberofTempSensors(uint8_t nodeID, uint8_t n_temp_sensors)
  {

  }

      //Watchdog and Power state handling
  void TLE9012::activateSleep()
  {
    (void) writeRegisterBroadcast(OP_MODE,0x0001); //Return is ignored for now
  }

  void TLE9012::resetWatchdog()
  {
    (void) writeRegisterBroadcast(WDOG_CNT,0x007F);
  }

  void TLE9012::setExtendedWatchdog(uint8_t nodeID)
  {
    uint16_t op_mode_reg = 0;
    (void) readRegisterSingle(nodeID, OP_MODE, &op_mode_reg);
    op_mode_reg |= 0x0002;
    (void) writeRegisterSingle(nodeID, OP_MODE, op_mode_reg);
  }

  void TLE9012::clearExtendedWatchdog(uint8_t nodeID)
  {
    uint16_t op_mode_reg = 0;
    (void) readRegisterSingle(nodeID, OP_MODE, &op_mode_reg);
    op_mode_reg &= 0xFFFD;
    (void) writeRegisterSingle(nodeID, OP_MODE, op_mode_reg);
  }

      //Miscallanious stuff
  uint16_t TLE9012::readICVersionandManufacturerID(uint8_t nodeID)
  {
    uint16_t id = 0;
    (void) readRegisterSingle(nodeID, ICVID, &id);
    return id;
  }

  void TLE9012::setNodeID(uint8_t oldID, uint8_t newID, uint8_t finalNode)
  {
    uint16_t cfg;
    (void) readRegisterSingle(oldID,CONFIG,&cfg);
    cfg &= 0x07C0;
    cfg |= newID & 0x3F;
    cfg |= finalNode<<11;
    (void) writeRegisterSingle(oldID,CONFIG,cfg);
  }

  void TLE9012::writeMailboxRegister(uint8_t nodeID, uint16_t value)
  {
    (void) writeRegisterSingle(nodeID,MAILBOX,value);
  }

  void TLE9012::readMailboxRegister(uint8_t nodeID)
  {
    uint16_t id = 0;
    (void) readRegisterSingle(nodeID, MAILBOX, &id);
    return id;
  }


      //Error checking and handling
  void TLE9012::checkDiagnoseResistor(uint8_t nodeID)
  {

  }

  void TLE9012::attachErrorHandler(tle9012_error_t errortype, void (*errorhandler)(uint8_t, uint16_t))
  {

  }

  void TLE9012::checkErrors(uint8_t nodeID)
  {

  }

      //Round Robin Functions

  void TLE9012::setRoundRobinErrorHandling(uint8_t nodeID, uint16_t rr_sleep_interval, uint8_t rr_temp_measurement_interval, uint8_t n_errors)
  {

  }

  void TLE9012::setRoundRobinConfig(uint8_t nodeID, uint8_t rr_counter, rr_error_mask_t errormask)
  {

  }

      //Cell Balancing Functions

  void TLE9012::setBalancingPWM(uint8_t nodeID, tle9012_balancing_pwm_t pwm_duty_cycle)
  {

  }

  void TLE9012::setBalancingCounter(uint8_t nodeID, uint8_t cell, uint8_t value)
  {

  }

  void TLE9012::startBalancing(uint8_t nodeID, uint16_t balancing_mask)
  {

  }

      //Threshold set functions
  void TLE9012::setOvervoltageThreshold(uint8_t nodeID, uint16_t fault_threshold)
  {

  }

  void TLE9012::setUndervoltageThreshold(uint8_t nodeID, uint16_t fault_threshold)
  {

  }

  void TLE9012::setOpenLoadThresholdMax(uint8_t nodeID, uint8_t open_load_threshold)
  {

  }

  void TLE9012::setOpenLoadThresholdMin(uint8_t nodeID, uint8_t open_load_threshold)
  {

  }

  void TLE9012::setExternalTemperatureThreshold(uint8_t nodeID, uint16_t external_overtemperature_threshold)
  {

  }

  void TLE9012::setInternalTemperatureThreshold(uint8_t nodeID, uint16_t internal_overtemperature_threshold)
  {

  }

  void TLE9012::setBalancingCurrentThreshold(uint8_t nodeID, uint8_t overcurrent_threshold, uint8_t undercurrent_threshold)
  {

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

  //Check if reply frame was received correctly
  msb_first_converter(&response_buffer[6],1);
  if(!crc3(response_buffer[6]))
  {
    status = isoUART_CRC_ERROR;
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

  //Check if reply frame was received correctly
  msb_first_converter(&response_buffer[6],1);
  if(!crc3(response_buffer[6]))
  {
    status = isoUART_CRC_ERROR;
    return status;
  }

  return status;
}

iso_uart_status_t TLE9012::configureMultiread(multiread_cfg_t cfg)  //Write a multiread configuration to all devices in the daisy chain
{
  return 0; //Multiread is unsuported for the moment
}

iso_uart_status_t TLE9012::multiRead(multread_result_t* databuffer) //Multiread command from all devices in the chain
{
  return 0; //Multiread is unsuported for the moment
}

//Private Functions start here

/*
  Calculate the CRC-3 of the reply Frame
  returns 1 if crc is correct and 0 if crc is false
*/
uint8_t TLE9012::crc3(uint8_t replyframe)
{
    uint8_t polynomial = 0xB0;
    uint8_t crc = replyframe;
    
    for(uint8_t n = 0; n < 5; n++)
    {
        if(crc & 0x80)
        {
            crc = crc^polynomial;
            crc = crc << 1;
        }
        else
            crc = crc << 1;
    }
    
    if(crc == 0)
        return 1;
    else
        return 0;
}

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