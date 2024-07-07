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

HardwareSerial* hisoUART; //Pointer to the Hardware Serial Driver



//-----------------------------------------------------------------------------
//                         Initilization Functions
//-----------------------------------------------------------------------------

/**
* Constructor for TLE9012 class. Unused for now
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

/**
* Destructor for TLE9012 class. Unused for now
*/
TLE9012::~TLE9012() //Destructor
{

}
      

/**
* @brief initialization Method
*
* Call this function before making any other calls to library functions. The Hardwareserial port is also initilized here
* @note only Hardware Serial should be used because the response from the TLE9012 comes in at high baudrates
*
* @param serial pointer to a Serial handler class/struct
* @param baudrate Baudrate of the Serialport -> must be between 1Mbit and 2Mbits
* @param rxpin optional parameter to specify RX Pin under some architectures
* @param txpin optional parameter to specify TX Pin under some architectures
*/      
      
void TLE9012::init(HardwareSerial* serial, uint32_t baudrate,uint8_t rxpin,uint8_t txpin)//Driver init
{
  hisoUART = serial;
  if((rxpin != 0) && (txpin != 0))
  {
    #ifdef ARDUINO_ARCH_ESP32
    hisoUART->begin(baudrate,SERIAL_8N1,rxpin,txpin);
    #else
    hisoUART->begin(baudrate,SERIAL_8N1);
    #endif
  }
  else
  {
    hisoUART->begin(baudrate,SERIAL_8N1);
  }
}


/**
* @brief Wakeup all TLE9012 devices on the Bus
*
* This function sends a wakeup signal across the daisy chain which is forwarded by all TLE9012/9015 devices
*/

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

  //---------------------------------------------------------------------------
  //                       Functions to Configure and Perform Measurements
  //---------------------------------------------------------------------------
  
  
  /**
  * @brief Performs cell voltage measurements for a given Device in the Daisy Chain
  *
  * This function performs cell voltage measurements for all configured cells of a device with a given nodeID.
  * The results are stored in the devices[nodeID-1].cell_voltages[] array. 
  *
  * @param nodeID is the address of the node on the daisy chain
  */
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
      (void) readRegisterSingle(nodeID, PCVM_0 + (12-devices[deviceID].n_cells + n), &devices[deviceID].cell_voltages[n]);
    }

    (void) readRegisterSingle(nodeID,BVM,&devices[deviceID].block_voltage);
    (void) readRegisterSingle(nodeID,SCVM_HIGH,&devices[deviceID].scvm_high);
    devices[deviceID].scvm_high &= 0xFFE0; //Remove rolling counter
    (void) readRegisterSingle(nodeID,SCVM_LOW,&devices[deviceID].scvm_low);
    devices[deviceID].scvm_low &= 0xFFE0; //Remove rolling counter
  }

  /**
  * @brief Performs NTC/PTC resistance measurements for a given Device in the Daisy Chain
  *
  * This function performs NTC/PTC resistance measurements for all configured Temperature channels of a device with a given nodeID.
  * The results are stored in the devices[nodeID-1].ntc_resistances[] array
  *
  * @param nodeID is the address of the node on the daisy chain
  */
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
    
    devices[deviceID].ntc_results_valid = 0;
    for(uint8_t n = 0; n < devices[deviceID].n_temp_sensors; n++)
    {
      (void) readRegisterSingle(nodeID,EXT_TEMP0+n,&devices[deviceID].ntc_resistances[n]);

      if((devices[deviceID].ntc_resistances[n]>>13)&0x01)
      {
        devices[deviceID].ntc_results_valid++;
      }
    }

    if(devices[deviceID].ntc_results_valid == 0)
      devices[deviceID].ntc_results_valid = 1;
    else
      devices[deviceID].ntc_results_valid = 0;
    
  }
  
  /**
  * @brief configure number of cells for device with address nodeID
  *
  * This function configures the ADCs in the device with address nodeID to only measure and monitor a given number of cells. If
  * fever than 12 cells are used, the channels next to GND shall be shorted on the TLE9012 Board but cell numbers will still start from
  * 0 to n_cells for all arrays inside of this driver.
  *
  * @param nodeID is the address of the node on the daisy chain
  * @param n_cells is the number of cells that are connected to the TLE9012 (max. 12)
  */
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

/**
* @brief configure number and configuration of temperature sensors
*
* This function configures the Temperature measurement unit in the device with address nodeID. Note that different
* senorconfig structs can be used for different boards but only one can be used per board. This is no hardware limitation
* but instead a limitation by this driver at the moment
*
* @param nodeID is the address of the node on the daisy chain
* @param n_temp_sensors is the number of temperature sensors that shall be used
* @param sensorconfig ntc_config_t struct that holds nominal resistance and beta value of the NTCs used
*/
  void TLE9012::setTempSensorsConfig(uint8_t nodeID, uint8_t n_temp_sensors, ntc_config_t sensorconfig)
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
 
/**
 * @brief Read the internal chip temperature of internal chiptemperature sensor 1 and 2
 * 
 * Internal chip Temperatures are stored in the devices[] struct only if the temperatures are valid.
 * 
 * @param nodeID is the address of the node on the daisy chain
 */
  void TLE9012::readChipTemperatures(uint8_t nodeID)
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

    (void) readRegisterSingle(nodeID,INT_TEMP,&devices[deviceID].chiptemperature1);
    (void) readRegisterSingle(nodeID,INT_TEMP_2,&devices[deviceID].chiptemperature2);

    if(((devices[deviceID].chiptemperature1>>13)&0x01) && ((devices[deviceID].chiptemperature2>>13)&0x01))
      devices[deviceID].chiptemperatures_valid = 1;
    else
      devices[deviceID].chiptemperatures_valid = 0;
    
  }

  //---------------------------------------------------------------------------
  //                       Watchdog and Powermode Functions
  //---------------------------------------------------------------------------
  
  
  /**
   * @brief Activate Sleep mode for all devices in the daisy chain
   * 
   * Powers down all devices on the daisy chain
   * 
   */
  void TLE9012::activateSleep()
  {
    (void) writeRegisterBroadcast(OP_MODE,0x0001); //Return is ignored for now
  }

  /**
   * @brief Reset Watchdog Timer to maximum value
   * 
   * Resets the watchdog timer register to 0x7F on all devices in the daisy chain
   * 
   */

  void TLE9012::resetWatchdog()
  {
    (void) writeRegisterBroadcast(WDOG_CNT,0x007F);
  }

  /**
   * @brief Changes the watchdog mode from normal to extended mode
   * 
   * In normal mode the watchdog times out after approximatly 2s which might be undesireable during inital configuration or
   * some other reasons. Therefore the watchdog mode can be set to extended leading to a timeout after approx. 37 hours. Note
   * that it's not recommended to use the extended mode during normal operation
   * 
   * @param nodeID is the address of the device on the daisy chain
   */

  void TLE9012::setExtendedWatchdog(uint8_t nodeID)
  {
    uint16_t op_mode_reg = 0;
    (void) readRegisterSingle(nodeID, OP_MODE, &op_mode_reg);
    op_mode_reg |= 0x0002;
    (void) writeRegisterSingle(nodeID, OP_MODE, op_mode_reg);
  }

  /**
   * @brief Set watchdog to normal operation mode
   * 
   * This function resets the watchdog mode from extended to normal. For a more detailled explanation see setExtendedWatchdog
   * 
   * @param nodeID is the address of the device on the daisy chain
   */

  void TLE9012::clearExtendedWatchdog(uint8_t nodeID)
  {
    uint16_t op_mode_reg = 0;
    (void) readRegisterSingle(nodeID, OP_MODE, &op_mode_reg);
    op_mode_reg &= 0xFFFD;
    (void) writeRegisterSingle(nodeID, OP_MODE, op_mode_reg);
  }

 
  //---------------------------------------------------------------------------
  //                       Miscellaneous Functions 
  //---------------------------------------------------------------------------
  
/**
 * @brief Read IC and Manufacturing ID
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @return uint16_t id -> IC and Manufacturer Version ID. Should return 0xC140 according to user manual
 */

  uint16_t TLE9012::readICVersionandManufacturerID(uint8_t nodeID)
  {
    uint16_t id = 0;
    (void) readRegisterSingle(nodeID, ICVID, &id);
    return id;
  }

/**
 * @brief Assign new nodeID to a device in the daisychain
 * 
 *  After reset all devices on the daisy chain start with nodeID 0 and will not forward messages until a non zero node ID is
 *  assigned. During enumeration this function can be used to assign new nodeIDs(device addresses) using subsequent calls with
 *  oldID = 0 and newID = 1..N. The finalNode parameter has to be set for the last element in the daisy chain to ensure that
 *  a reply frame is send in response to a broadcast command
 * 
 * @param oldID is the current nodeID of the device
 * @param newID is the new nodeID of the device
 * @param finalNode is 0 if the device is not the final element in the daisy chain and 1 if the device is the final element
 */

  void TLE9012::setNodeID(uint8_t oldID, uint8_t newID, uint8_t finalNode)
  {
    uint16_t cfg;
    (void) readRegisterSingle(oldID,CONFIG,&cfg);
    cfg &= 0x07C0;
    cfg |= newID & 0x3F;
    cfg |= finalNode<<11;
    (void) writeRegisterSingle(oldID,CONFIG,cfg);
  }

/**
 * @brief Write a value to the mailbox register
 * 
 *  The mailbox register is a voltaile storage register which can be used to verify read and write access working correctly
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param value is the value that shall be written into the mailbox register
 */

  void TLE9012::writeMailboxRegister(uint8_t nodeID, uint16_t value)
  {
    (void) writeRegisterSingle(nodeID,MAILBOX,value);
  }

/**
 * @brief Reads the current value in the mailbox register
 * 
 *  Read Wrapper for the mailbox register to ensure read access is working correctly
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @return uint16_t is the value currently stored in the mailbox register
 */

  uint16_t TLE9012::readMailboxRegister(uint8_t nodeID)
  {
    uint16_t id = 0;
    (void) readRegisterSingle(nodeID, MAILBOX, &id);
    return id;
  }

  //---------------------------------------------------------------------------
  //                       Error Checking and Handling 
  //---------------------------------------------------------------------------

/**
 * @brief Check if the Diagnosis resistor measurement is inside the expected range. This can be used in error handlers to
 * further analyse the root cause of errors
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @return uint8_t returns 1 if diagnosis resistor is inside of the temperature range, 2 if no valid result is detected and 0 if
 * the diagnosis resistance is outside of the expected range
 */
  uint8_t TLE9012::checkDiagnoseResistor(uint8_t nodeID)
  {
    //Not implemented yet
    uint16_t reg_diag_r = 0;
    (void) readRegisterSingle(nodeID, EXT_TEMP_R_DIAG, &reg_diag_r);
    if((reg_diag_r>>13)&0x01)
      return 2;
    uint16_t result = reg_diag_r & 0x03FF;
    uint8_t intc = (reg_diag_r >> 10) &0x03;

    switch (intc)
    {
    case 3:
      if((reg_diag_r > 218) | (reg_diag_r < 107))
      {
        return 1;
      }
      else
      {
        return 0;
      }
      break;

    case 2:
      if((reg_diag_r > 358) | (reg_diag_r < 193))
      {
        return 1;
      }
      else
      {
        return 0;
      }
      break;

    case 1:
      if((reg_diag_r > 603) | (reg_diag_r < 326))
      {
        return 1;
      }
      else
      {
        return 0;
      }
      break;

    case 0:
      if((reg_diag_r > 950) | (reg_diag_r < 595))
      {
        return 1;
      }
      else
      {
        return 0;
      }
      break;
    
    default:
      break;
    }

  }


  /**
   * @brief Attach an error handler function that is called if something goes wrong
   * 
   * This function allows you to register your own errorhandler that gets called if the checkErrors function is detecting
   * an error. The available types of errors are defined in the tle9012_error_t enum and cover all error that are described in
   * the General Diagnosis register. All errorhandler functions take a nodeID as well as a uint16_t additional parameter as arguments. 
   * The additional arguments usually include the associated errorflag registers. If no associate register exists, a filler value is
   * used. 
   * 
   * @param errortype is the type of error that the handler is for 
   * @param errorhandler is a function pointer to the errorhandler
   */

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
      default:
        break;
    }
  }

/**
 * @brief Check the GEN_DIAG register for errors and call the appropriate errorhandler function if one was defined via the
 * attachErrorHandler function
 * 
 * @param nodeID is the address of the node on the daisy chain
 */
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

/**
 * @brief Reset the GEN_DIAG register
 * 
 * Clears all errorflags stored in the GEN_DIAG register
 * 
 * @param nodeID is the address of the node on the daisy chain
 */

  void TLE9012::resetErrors(uint8_t nodeID)
  {
    (void) writeRegisterSingle(nodeID,GEN_DIAG,0x0000);
  }

/**
 * @brief Configure the Fault Mask register influencing the EMM-Signal and Error-Pin
 * 
 * This function can be used to mask errors that shall not trigger an Emergency Mode (EMM) Signal or influence the Error Pin
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param err_mask defines what type of errors shall be masked out 
 * @return * void 
 */
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

  //---------------------------------------------------------------------------
  //                       Round Robin Functions 
  //---------------------------------------------------------------------------


/**
 * @brief Set parameters for the Round Robin Error detection and self check
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param rr_sleep_interval How often is the round robing triggered in rr_sleep_LSB
 * @param rr_temp_measurement_interval External temperature measurement can be skipped every n round robin cycles
 * @param n_errors Number of consecutive errors before error pin and GEN_DIAG register is triggered
 */

  void TLE9012::setRoundRobinErrorHandling(uint8_t nodeID, uint16_t rr_sleep_interval, uint8_t rr_temp_measurement_interval, uint8_t n_errors)
  {
    rr_sleep_interval = (rr_sleep_interval&0x03FF)<<6;
    rr_temp_measurement_interval = (rr_temp_measurement_interval&0x7)<<3;
    n_errors = n_errors & 0x07;

    (void) writeRegisterSingle(nodeID, RR_ERR_CNT, rr_sleep_interval | (uint16_t) rr_temp_measurement_interval | (uint16_t) n_errors);
    
  }

/**
 * @brief Configure Errormasks and Round Robin Timing
 * 
 * This function configures the round robin timing as well as the round robin sync function and errormasks. The Round Robin counter
 * defines how often a Round Robin cycle is performed. In addition the RR_SYNC function can be activated which triggers a round robin cycle
 * each time a write access to the watchdog counter is performed. 
 * 
 * In addition an errormask of type rr_error_mask_t can be passed as an argument which allows to enable which type of errors are
 * included in the round robin measurement cycle
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param rr_counter sets how often a Round Robin is performed
 * @param rr_sync if 1, the sync mode is activated 
 * @param errormask errormask defining what errors shall be monitored by the round robin. Setting flags to 1 enables the monitoring
 */

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

  //---------------------------------------------------------------------------
  //                       Balancing Functions 
  //---------------------------------------------------------------------------

/**
 * @brief 
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param pwm_duty_cycle 
 */
  void TLE9012::setBalancingPWM(uint8_t nodeID, tle9012_balancing_pwm_t pwm_duty_cycle)
  {
    (void) writeRegisterSingle(nodeID,BAL_PWM,pwm_duty_cycle);
  }

/**
 * @brief Set the balancing time for a cell
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param cell cell between 1 and 12 
 * @param value value of the cell counter between 0 and 0x1F (7.5min steps)
 */
  void TLE9012::setBalancingCounter(uint8_t nodeID, uint8_t cell, uint8_t value)
  {
    cell = cell-1;
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

/**
 * @brief Start Balancing on channels selected by bitmask
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param balancing_mask bitmask defining on what channels balancing is activated
 */
  void TLE9012::startBalancing(uint8_t nodeID, uint16_t balancing_mask)
  {
    (void) writeRegisterSingle(nodeID,BAL_SETTINGS,balancing_mask & 0x0FFF);
  }

  //---------------------------------------------------------------------------
  //                       Error Threshold Settings
  //---------------------------------------------------------------------------

/**
 * @brief Set Overvoltage Threshold for Error detection
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param fault_threshold Overvoltage Threshold in 5V/1024 * fault_threshold
 */
  void TLE9012::setOvervoltageThreshold(uint8_t nodeID, uint16_t fault_threshold)
  {
    uint16_t ol_ov_reg = 0;
    (void) readRegisterSingle(nodeID,OL_OV_THR,&ol_ov_reg);
    ol_ov_reg &= 0xFC00;
    ol_ov_reg |= fault_threshold & 0x03FF;
    (void) writeRegisterSingle(nodeID,OL_OV_THR,ol_ov_reg);
  }

/**
 * @brief Set Undervoltage Threshold for Error detection
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param fault_threshold Undervoltage Threshold in 5V/1024 * fault_threshold
 */
  void TLE9012::setUndervoltageThreshold(uint8_t nodeID, uint16_t fault_threshold)
  {
    uint16_t ol_uv_reg = 0;
    (void) readRegisterSingle(nodeID,OL_UV_THR,&ol_uv_reg);
    ol_uv_reg &= 0xFC00;
    ol_uv_reg |= fault_threshold & 0x03FF;
    (void) writeRegisterSingle(nodeID,OL_UV_THR,ol_uv_reg);
  }

/**
 * @brief Set maximum voltage drop for Open Load detection
 * 
 * Expected Voltage drop is approx. 15mA * Rf
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param open_load_threshold threshold in 19.5mV steps
 */
  void TLE9012::setOpenLoadThresholdMax(uint8_t nodeID, uint8_t open_load_threshold)
  {
    uint16_t ol_thr_max = 0;
    (void) readRegisterSingle(nodeID,OL_OV_THR,&ol_thr_max);
    ol_thr_max &= 0x03FF;
    ol_thr_max |= ((uint16_t) open_load_threshold & 0x3F) << 10;
    (void) writeRegisterSingle(nodeID,OL_OV_THR,ol_thr_max);
  }

/**
 * @brief Set minimum voltage drop for Open Load detection
 * 
 * Expected Voltage drop is approx. 15mA * Rf
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param open_load_threshold threshold in 19.5mV steps
 */
  void TLE9012::setOpenLoadThresholdMin(uint8_t nodeID, uint8_t open_load_threshold)
  {
    uint16_t ol_thr_min = 0;
    (void) readRegisterSingle(nodeID,OL_UV_THR,&ol_thr_min);
    ol_thr_min &= 0x03FF;
    ol_thr_min |= ((uint16_t) open_load_threshold & 0x3F)<<10;
    (void) writeRegisterSingle(nodeID,OL_UV_THR,ol_thr_min);
  }

/**
 * @brief Set threshold for external temperature detection
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param external_overtemperature_threshold threshold for external overtemperature warning
 */
  void TLE9012::setExternalTemperatureThreshold(uint8_t nodeID, uint16_t external_overtemperature_threshold)
  {
  
  }

/**
 * @brief Set threshold for internal overtemperature
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param internal_overtemperature_threshold internal DIE overtemperature limit 
 */
  void TLE9012::setInternalTemperatureThreshold(uint8_t nodeID, uint16_t internal_overtemperature_threshold)
  {
    (void) writeRegisterSingle(nodeID, INT_OT_WARN_CONF, (internal_overtemperature_threshold & 0x3FF));
  }

/**
 * @brief Set Balancing current thresholds
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param overcurrent_threshold maximum voltage drop over balancing resistor in 19.5mV steps 
 * @param undercurrent_threshold minimum voltage drop over balancing resistor in 19.5mV steps
 */
  void TLE9012::setBalancingCurrentThreshold(uint8_t nodeID, uint8_t overcurrent_threshold, uint8_t undercurrent_threshold)
  {
    uint16_t bal_thr = (((uint16_t) undercurrent_threshold) << 8) | overcurrent_threshold;
    (void) writeRegisterSingle(nodeID, BAL_CURR_THR, bal_thr);
  }


      
  //---------------------------------------------------------------------------
  //                       Low Level isoUART Access Functions
  //---------------------------------------------------------------------------

/**
 * @brief isoUART function to read a single register from a single node on the daisy chain
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param regaddress address of register for this operation
 * @param result result of the read operation is stored in this variable
 * @return iso_uart_status_t returns status of the bustransaction
 */
iso_uart_status_t TLE9012::readRegisterSingle(uint8_t nodeID, uint16_t regaddress, uint16_t* result)
{
  ISOUART_LOCK();
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
    ISOUART_UNLOCK();
    return status;
  }

  #ifdef SOFT_MSB_FIRST
  msb_first_converter(&(response_buffer[4]),5);
  #endif
  uint8_t crc = crc8(&response_buffer[4],4);

  *result = (((uint16_t) response_buffer[6])<<8) | ((uint16_t) response_buffer[7]);

  if(crc != response_buffer[8])
    status = isoUART_CRC_ERROR;

  ISOUART_UNLOCK();
  return status;
}

iso_uart_status_t TLE9012::readRegisterSingle_ext(uint8_t* nodeID, uint16_t* regaddress, uint16_t* result)
{
  ISOUART_LOCK();
  iso_uart_status_t status;
  uint8_t response_buffer[9];
  
  status = isoUART_TIMEOUT;

  isoUARTClearRXBUffer();
  isoUARTReadRequest(*nodeID, *regaddress);

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
    ISOUART_UNLOCK();
    return status;
  }

  #ifdef SOFT_MSB_FIRST
  msb_first_converter(&(response_buffer[4]),5);
  #endif
  uint8_t crc = crc8(&response_buffer[4],4);

  *result = (((uint16_t) response_buffer[6])<<8) | ((uint16_t) response_buffer[7]);
  *regaddress = (uint16_t) response_buffer[5];
  *nodeID = (uint8_t) response_buffer[4];

  if(crc != response_buffer[8])
    status = isoUART_CRC_ERROR;

  ISOUART_UNLOCK();
  return status;
}

/**
 * @brief isoUART function to write a value to a single register on a single node in the daisy chain
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param regaddress address of register for this operation
 * @param databuffer value that shall be written to the register
 * @return iso_uart_status_t returns status of the bustransaction
 */
iso_uart_status_t TLE9012::writeRegisterSingle(uint8_t nodeID, uint16_t regaddress, uint16_t databuffer) //Write data to a single register
{
  ISOUART_LOCK();
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
    ISOUART_UNLOCK();
    return status;
  }  

  //Check if reply frame was received correctly
  #ifdef SOFT_MSB_FIRST
  msb_first_converter(&response_buffer[6],1);
  #endif
  if(!crc3(response_buffer[6]))
  {
    status = isoUART_CRC_ERROR;
    ISOUART_UNLOCK();
    return status;
  }
  ISOUART_UNLOCK();
  return status;
}

/**
 * @brief isoUART function to read the value of a register in broadcast mode from every node on the daisy chain
 * 
 * @param regaddress address of register for this operation
 * @param result pointer to an array storing the result -> arraysize must be equal or larger than number of devices in the daisy chain
 * @return iso_uart_status_t returns status of the bustransaction
 */
iso_uart_status_t TLE9012::readRegisterBroadcast(uint16_t regaddress, uint16_t* result) //Read a broadcast to all devices in the daisy chain
{
  ISOUART_LOCK();
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
    ISOUART_UNLOCK();
    return status;
  }
  
  #ifdef SOFT_MSB_FIRST
  msb_first_converter(&response_buffer[4],N_DEVICES*5);
  #endif

  for(uint8_t n = 0; n < N_DEVICES; n++)
  {
    uint8_t crc = crc8(&response_buffer[4+(n*N_DEVICES)],4);
    result[n] = (((uint16_t) response_buffer[6+(n*N_DEVICES)])<<8) | ((uint16_t) response_buffer[7+(n*N_DEVICES)]);
    if(crc != 0)
      status = isoUART_CRC_ERROR;
  }
  
  ISOUART_UNLOCK();
  return status;
}

/**
 * @brief isoUART function to write a register in all nodes in the daisy chain
 * 
 * @param regaddress address of register for this operation
 * @param databuffer value that shall be written to the selected register
 * @return iso_uart_status_t returns status of the bustransaction
 */
iso_uart_status_t TLE9012::writeRegisterBroadcast(uint16_t regaddress, uint16_t databuffer) //Write a register as broadcast from all devices in the chain
{
  ISOUART_LOCK();
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
    ISOUART_UNLOCK();
    return status;
  }

  //Check if reply frame was received correctly
  #ifdef SOFT_MSB_FIRST
  msb_first_converter(&response_buffer[6],1);
  #endif

  if(!crc3(response_buffer[6]))
  {
    status = isoUART_CRC_ERROR;
    ISOUART_UNLOCK();
    return status;
  }

  ISOUART_UNLOCK();
  return status;
}

/**
 * @brief Configure the multiread function on all devices
 * 
 * @note currently not supported
 * 
 * @param cfg multiread configuration containing information about what registers shall be reed with a multiread command
 * @return iso_uart_status_t returns status of the bustransaction
 */
iso_uart_status_t TLE9012::configureMultiread(multiread_cfg_t cfg)  //Write a multiread configuration to all devices in the daisy chain
{
  return isoUART_OK; //Multiread is unsuported for the moment
}

/**
 * @brief isoUART function to perform a multiread operation
 * 
 * @note currently not supported
 * 
 * @param databuffer databuffer for the multiread command. The databuffer must have a large enough size to prevent bufferoverflow
 * @return iso_uart_status_t returns status of the bustransaction
 */
iso_uart_status_t TLE9012::multiRead(multread_result_t* databuffer) //Multiread command from all devices in the chain
{
  ISOUART_LOCK();

  ISOUART_UNLOCK();
  return isoUART_OK; //Multiread is unsuported for the moment
}

  //---------------------------------------------------------------------------
  //                       Platform dependent Functions
  //---------------------------------------------------------------------------

/**
 * @brief Check the crc3 of reply frames of the isoUART protocol
 * 
 * @param replyframe 8bit replyframe including the CRC3
 * @return uint8_t returns 1 if CRC is correct and 0 if CRC is incorrect
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

/**
 * @brief Calculate the crc8 for dataframes in the isoUART protocol
 * 
 * @param buffer buffer containing the data for the crc calculation
 * @param len length of the buffer
 * @return uint8_t crc8 value
 */
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

/**
 * @brief flip bits in an byte to allow for msb first uart frames over arduinos lsb first serial driver
 * 
 * @param data databuffer with bytes that shall be converted from lsb first to msb first
 * @param len length of the buffer
 * @return uint8_t always returns 0
 */
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

/**
 * @brief starts a write request on the isoUART bus
 * 
 * While the writeRegisterSingle function seems to do the same, the isoUARTWriteRequest function
 * provides a wrapper to allow for an easy adaption to different Frameworks with different Serial
 * style implementations
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param regaddress address of register for this operation
 * @param data data that shall be written to the register
 */
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

/**
 * @brief request data from a device using the isoUART protocol
 * 
 * Note that this function does not handle the received data but only sends a request for data.
 * Handeling of the received packet is in the responsibility of the caller function
 * 
 * @param nodeID is the address of the node on the daisy chain
 * @param regaddress 
 */
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

/**
 * @brief Clear remaining RX Buffer in case something went wrong during isoUART read or write
 * 
 */
void TLE9012::isoUARTClearRXBUffer()
{
  while(hisoUART->available())
    uint8_t null = hisoUART->read();
}

/**
 * @brief Delay in milliseconds
 * 
 * Wrapper function to enable easier port to different Architectures/Frameworks
 * 
 * @param delay_ms Minimum delay time in milliseconds
 */
void TLE9012::mcuDelay(uint32_t delay_ms)
{
  delay(delay_ms);
}