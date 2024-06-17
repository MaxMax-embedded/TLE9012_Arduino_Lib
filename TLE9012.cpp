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
  errorcallbacks.adc_error_callback = NULL;
  errorcallbacks.balancing_error_overcurrent_callback = NULL;
  errorcallbacks.balancing_error_undercurrent_callback = NULL;
  errorcallbacks.external_temp_error_callback = NULL;
  errorcallbacks.internal_IC_error_callback = NULL;
  errorcallbacks.internal_temp_error_callback = NULL;
  errorcallbacks.open_load_error_callback = NULL;
  errorcallbacks.overvoltage_callback = NULL;
  errorcallbacks.reg_crc_error_callback = NULL;
  errorcallbacks.undervoltage_callback = NULL;
  errorcallbacks.ps_error_sleep_callback = NULL;
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

    if(nodeID > N_DEVICES)
    {
      return; //Early return if device number is to high
    }

    uint8_t deviceID = nodeID - 1;

    if(nodeID == 0)
      deviceID = 0;
    else
      deviceID = nodeID-1;
    
    writeRegisterSingle(nodeID, MEAS_CTRL, 0xEE61); //Trigger PCVM, BVM and SCVM with PBOFF
    mcuDelay(5);

    for(uint8_t n = 0; n < devices[deviceID].n_cells; n++)
    {
      (void) readRegisterSingle(nodeID, PCVM_0 + (11-devices[deviceID].n_cells + n), &devices[deviceID].cell_voltages[n]);
    }

    (void) readRegisterSingle(nodeID,BVM,&devices[deviceID].block_voltage);
    (void) readRegisterSingle(nodeID,SCVM_HIGH,&devices[deviceID].scvm_high);
    (void) readRegisterSingle(nodeID,SCVM_LOW,&devices[deviceID].scvm_low);
  }

  void TLE9012::readTemperatures(uint8_t nodeID)
  {
    if(nodeID > N_DEVICES)
    {
      return; //Early return if device number is to high
    }

    uint8_t deviceID = nodeID - 1;

    if(nodeID == 0)
      deviceID = 0;
    else
      deviceID = nodeID-1;

    (void) writeRegisterSingle(nodeID, MEAS_CTRL, 0x0080);
    mcuDelay(5);
    
    for(uint8_t n = 0; n < devices[deviceID].n_temp_sensors; n++)
    {
      (void) readRegisterSingle(nodeID,EXT_TEMP0+n,&devices[deviceID].ntc_resistances[n]);
    }
    
  }

  void TLE9012::setNumberofCells(uint8_t nodeID, uint8_t n_cells)
  {

    if(nodeID > N_DEVICES)
    {
      return; //Early return if device number is to high
    }

    uint8_t deviceID = nodeID - 1;

    if(nodeID == 0)
      deviceID = 0;
    else
      deviceID = nodeID-1;

    devices[deviceID].n_cells = n_cells;

    uint16_t cell_mask = 0;

    for(uint8_t n = 0; n < n_cells; n++)
    {
      cell_mask |= 1<<(11-n);
    }
    
    (void) writeRegisterSingle(nodeID,PART_CONFIG,cell_mask);
    (void) writeRegisterSingle(nodeID,SCVM_CONFIG,cell_mask);
  }

  void TLE9012::setTempSensorsConfig(uint8_t nodeID, uint8_t n_temp_sensors,ntc_config_t sensorconfig)
  {

    if(nodeID > N_DEVICES)
    {
      return; //Early return if device number is to high
    }

    uint8_t deviceID = nodeID - 1;

    if(nodeID == 0)
      deviceID = 0;
    else
      deviceID = nodeID-1;

    devices[deviceID].n_temp_sensors = n_temp_sensors;
    devices[deviceID].sensorconfig = sensorconfig;

    uint16_t sens_conf = 0;
    (void) readRegisterSingle(nodeID, TEMP_CONF,&sens_conf);

    if(n_temp_sensors > 5)
      n_temp_sensors = 5;

    sens_conf &= 0x0FFF;
    sens_conf |= n_temp_sensors << 12;

    (void) writeRegisterSingle(nodeID, TEMP_CONF, sens_conf);

    uint16_t avm_sensemask = 0;
    for(uint8_t n = 0; n <n_temp_sensors; n++)
    {
      avm_sensemask |= 1<<(n+3);
    }
    avm_sensemask |= 0x7;

    (void) writeRegisterSingle(nodeID, AVM_CONFIG, avm_sensemask);
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

  uint16_t TLE9012::readMailboxRegister(uint8_t nodeID)
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
    switch(errortype)
    {
      case OVERVOLTAGE_ERROR:
        errorcallbacks.overvoltage_callback = errorhandler;
        break;
      case UNDERVOLTAGE_ERROR:
        errorcallbacks.undervoltage_callback = errorhandler;
        break;
      case ADC_ERROR:
        errorcallbacks.adc_error_callback = errorhandler;
        break;
      case INTERNAL_IC_ERROR:
        errorcallbacks.internal_IC_error_callback = errorhandler;
        break;
      case OPEN_LOAD_ERROR:
        errorcallbacks.open_load_error_callback = errorhandler;
        break;
      case REG_CRC_ERROR:
        errorcallbacks.reg_crc_error_callback = errorhandler;
        break;
      case EXTERNAL_TEMP_ERROR:
        errorcallbacks.external_temp_error_callback = errorhandler;
        break;
      case INTERNAL_TEMP_ERROR:
        errorcallbacks.internal_temp_error_callback = errorhandler;
        break;
      case BALANCING_UNDERCURRENT_ERROR:
        errorcallbacks.balancing_error_undercurrent_callback = errorhandler;
        break;
      case BALANCING_OVERCURRENT_ERROR:
        errorcallbacks.balancing_error_overcurrent_callback = errorhandler;
        break;
    }
  }

  void TLE9012::checkErrors(uint8_t nodeID)
  {
    uint16_t gen_diag = 0;
    (void) readRegisterSingle(nodeID,GEN_DIAG,&gen_diag);
    
    if(gen_diag & 0x8000)
    {
      uint16_t balancing_oc_flags = 0;
      (void) readRegisterSingle(nodeID,BAL_DIAG_OC,&balancing_oc_flags);
      if(errorcallbacks.balancing_error_overcurrent_callback != NULL)
        errorcallbacks.balancing_error_overcurrent_callback(nodeID,balancing_oc_flags);
    }

    if(gen_diag & 0x4000)
    {
      uint16_t balancing_uc_flags = 0;
      (void) readRegisterSingle(nodeID,BAL_DIAG_UC,&balancing_uc_flags);
      if(errorcallbacks.balancing_error_undercurrent_callback != NULL)
        errorcallbacks.balancing_error_undercurrent_callback(nodeID,balancing_uc_flags);
    }

    if(gen_diag & 0x2000)
    {
      uint16_t cell_ov_diag = 0;
      (void) readRegisterSingle(nodeID,CELL_OV,&cell_ov_diag);
      if(errorcallbacks.overvoltage_callback != NULL)
        errorcallbacks.overvoltage_callback(nodeID,cell_ov_diag);
    }

    if(gen_diag & 0x1000)
    {
      uint16_t cell_uv_diag = 0;
      (void) readRegisterSingle(nodeID,CELL_UV,&cell_uv_diag);
      if(errorcallbacks.undervoltage_callback != NULL)
        errorcallbacks.undervoltage_callback(nodeID,cell_uv_diag);
    }

    if(gen_diag & 0x0800)
    {
      uint16_t overtemp = 0;
      (void) readRegisterSingle(nodeID,INT_TEMP,&overtemp);
      if(errorcallbacks.internal_temp_error_callback != NULL)
        errorcallbacks.internal_temp_error_callback(nodeID,overtemp);
    }

    if(gen_diag & 0x0400)
    {
      uint16_t ext_temp_err = 0;
      (void) readRegisterSingle(nodeID,EXT_TEMP_DIAG,&ext_temp_err);
      if(errorcallbacks.external_temp_error_callback != NULL)
        errorcallbacks.external_temp_error_callback(nodeID,ext_temp_err);
    }

    if(gen_diag & 0x0200)
    {
      uint16_t reg_crc = 0;
      (void) readRegisterSingle(nodeID,REG_CRC_ERR,&reg_crc);
      if(errorcallbacks.reg_crc_error_callback != NULL)
        errorcallbacks.reg_crc_error_callback(nodeID,reg_crc);
    }

    if(gen_diag & 0x0100)
    {
      uint16_t ic_error = 0;
      if(errorcallbacks.internal_IC_error_callback != NULL)
        errorcallbacks.internal_IC_error_callback(nodeID,0x0000);
    }

    if(gen_diag & 0x0080)
    {
      uint16_t open_load = 0;
      (void) readRegisterSingle(nodeID,DIAG_OL,&open_load);
      if(errorcallbacks.open_load_error_callback != NULL)
        errorcallbacks.open_load_error_callback(nodeID,open_load);
    }

    if(gen_diag & 0x0040)
    {
      if(errorcallbacks.adc_error_callback != NULL)
        errorcallbacks.adc_error_callback(nodeID,0x0000);
    }

    if(gen_diag & 0x0020)
    {
      if(errorcallbacks.ps_error_sleep_callback != NULL)
        errorcallbacks.ps_error_sleep_callback(nodeID,0x0000);
    }
  }

  void TLE9012::resetErrors(uint8_t nodeID)
  {
    (void) writeRegisterSingle(nodeID,GEN_DIAG,0x0000);
  }
      
  void TLE9012::configFaultMasks(uint8_t nodeID, err_emm_error_mask_t err_mask)
  {
    uint16_t errormask = 0;
    errormask |= err_mask.balancing_overcurrent_error<<15;
    errormask |= err_mask.balancing_undercurrent_error<<14;
    errormask |= err_mask.overvoltage_error << 13;
    errormask |= err_mask.undervoltage_error << 12;
    errormask |= err_mask.internal_temperature_error << 11;
    errormask |= err_mask.external_termperature_error << 10;
    errormask |= err_mask.reg_crc_err << 9;
    errormask |= err_mask.int_ic_err << 8;
    errormask |= err_mask.open_load_error << 7;
    errormask |= err_mask.adc_error << 6;
    errormask |= err_mask.err_pin << 5;

    (void) writeRegisterSingle(nodeID,FAULT_MASK,errormask);
  }
      //Round Robin Functions

  void TLE9012::setRoundRobinErrorHandling(uint8_t nodeID, uint16_t rr_sleep_interval, uint8_t rr_temp_measurement_interval, uint8_t n_errors)
  {
    rr_sleep_interval = (rr_sleep_interval&0x03FF)<<6;
    rr_temp_measurement_interval = (rr_temp_measurement_interval&0x7)<<3;
    n_errors = n_errors & 0x07;

    (void) writeRegisterSingle(nodeID, RR_ERR_CNT, rr_sleep_interval | (uint16_t) rr_temp_measurement_interval | (uint16_t) n_errors);
    
  }

  void TLE9012::setRoundRobinConfig(uint8_t nodeID, uint8_t rr_counter, uint8_t rr_sync, rr_error_mask_t errormask)
  {
    uint16_t rr_cfg = (uint16_t) rr_counter & 0x7F;
    rr_cfg |= (rr_sync & 0x1) << 7;
    rr_cfg |= errormask.adc_error << 8;
    rr_cfg |= errormask.open_load_error << 9;
    rr_cfg |= errormask.external_termperature_error << 10;
    rr_cfg |= errormask.internal_temperature_error << 11;
    rr_cfg |= errormask.undervoltage_error << 12;
    rr_cfg |= errormask.overvoltage_error << 13;
    rr_cfg |= errormask.balancing_undercurrent_error << 14;
    rr_cfg |= errormask.balancing_overcurrent_error << 15;

    (void) writeRegisterSingle(nodeID, RR_CONFIG, rr_cfg);
  }

      //Cell Balancing Functions

  void TLE9012::setBalancingPWM(uint8_t nodeID, tle9012_balancing_pwm_t pwm_duty_cycle)
  {
    (void) writeRegisterSingle(nodeID,BAL_PWM,pwm_duty_cycle);
  }

  void TLE9012::setBalancingCounter(uint8_t nodeID, uint8_t cell, uint8_t value)
  {
    uint8_t regblock = cell/3;
    cell = (cell%3)*5;
    uint16_t val = (value & 0x1F)<<cell;

    switch(regblock)
    {
      case 0:
        (void) writeRegisterSingle(nodeID,BAL_CNT_0,val);
        break;
      case 1:
        (void) writeRegisterSingle(nodeID,BAL_CNT_1,val);
        break;
      case 2:
        (void) writeRegisterSingle(nodeID,BAL_CNT_2,val);
        break;
      case 3:
        (void) writeRegisterSingle(nodeID,BAL_CNT_3,val);
        break;
    }
  }

  void TLE9012::startBalancing(uint8_t nodeID, uint16_t balancing_mask)
  {
    (void) writeRegisterSingle(nodeID,BAL_SETTINGS,balancing_mask & 0x0FFF);
  }

      //Threshold set functions
  void TLE9012::setOvervoltageThreshold(uint8_t nodeID, uint16_t fault_threshold)
  {
    uint16_t ol_ov_reg = 0;
    (void) readRegisterSingle(nodeID,OL_OV_THR,&ol_ov_reg);
    ol_ov_reg &= 0xFC00;
    ol_ov_reg |= fault_threshold & 0x03FF;
    (void) writeRegisterSingle(nodeID,OL_OV_THR,ol_ov_reg);
  }

  void TLE9012::setUndervoltageThreshold(uint8_t nodeID, uint16_t fault_threshold)
  {
    uint16_t ol_uv_reg = 0;
    (void) readRegisterSingle(nodeID,OL_UV_THR,&ol_uv_reg);
    ol_uv_reg &= 0xFC00;
    ol_uv_reg |= fault_threshold & 0x03FF;
    (void) writeRegisterSingle(nodeID,OL_UV_THR,ol_uv_reg);
  }

  void TLE9012::setOpenLoadThresholdMax(uint8_t nodeID, uint8_t open_load_threshold)
  {
    uint16_t ol_thr_max = 0;
    (void) readRegisterSingle(nodeID,OL_OV_THR,&ol_thr_max);
    ol_thr_max &= 0x03FF;
    ol_thr_max |= ((uint16_t) open_load_threshold & 0x3F) << 10;
    (void) writeRegisterSingle(nodeID,OL_OV_THR,ol_thr_max);
  }

  void TLE9012::setOpenLoadThresholdMin(uint8_t nodeID, uint8_t open_load_threshold)
  {
    uint16_t ol_thr_min = 0;
    (void) readRegisterSingle(nodeID,OL_UV_THR,&ol_thr_min);
    ol_thr_min &= 0x03FF;
    ol_thr_min |= ((uint16_t) open_load_threshold & 0x3F)<<10;
    (void) writeRegisterSingle(nodeID,OL_UV_THR,ol_thr_min);
  }

  void TLE9012::setExternalTemperatureThreshold(uint8_t nodeID, uint16_t external_overtemperature_threshold)
  {

  }

  void TLE9012::setInternalTemperatureThreshold(uint8_t nodeID, uint16_t internal_overtemperature_threshold)
  {
    (void) writeRegisterSingle(nodeID, INT_OT_WARN_CONF, (internal_overtemperature_threshold & 0x3FF));
  }

  void TLE9012::setBalancingCurrentThreshold(uint8_t nodeID, uint8_t overcurrent_threshold, uint8_t undercurrent_threshold)
  {
    uint16_t bal_thr = (((uint16_t) undercurrent_threshold) << 8) | overcurrent_threshold;
    (void) writeRegisterSingle(nodeID, BAL_CURR_THR, bal_thr);
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

  #ifdef SOFT_MSB_FIRST
  msb_first_converter(&(response_buffer[4]),5);
  #endif
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
  #ifdef SOFT_MSB_FIRST
  msb_first_converter(&response_buffer[6],1);
  #endif
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
  
  #ifdef SOFT_MSB_FIRST
  msb_first_converter(&response_buffer[4],N_DEVICES*5);
  #endif

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
  #ifdef SOFT_MSB_FIRST
  msb_first_converter(&response_buffer[6],1);
  #endif

  if(!crc3(response_buffer[6]))
  {
    status = isoUART_CRC_ERROR;
    return status;
  }

  return status;
}

iso_uart_status_t TLE9012::configureMultiread(multiread_cfg_t cfg)  //Write a multiread configuration to all devices in the daisy chain
{
  return isoUART_OK; //Multiread is unsuported for the moment
}

iso_uart_status_t TLE9012::multiRead(multread_result_t* databuffer) //Multiread command from all devices in the chain
{
  return isoUART_OK; //Multiread is unsuported for the moment
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
  #ifdef SOFT_MSB_FIRST
  msb_first_converter(writebuffer, 6);
  #endif

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
  #ifdef SOFT_MSB_FIRST
  msb_first_converter(writebuffer, 4);
  #endif
  hisoUART->write(writebuffer,4);
}

void TLE9012::isoUARTClearRXBUffer()
{
  while(hisoUART->available())
    uint8_t null = hisoUART->read();
}

void TLE9012::mcuDelay(uint32_t delay_ms)
{
  delay(delay_ms);
}