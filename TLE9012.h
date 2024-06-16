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



#ifndef TLE9012_LIB
#define TLE9012_LIB


#include <Arduino.h>
#include <stdint.h>


#define N_DEVICES 1 //Number of devices in the daisy chain


#define ISOUART_TIMEOUT 100

#define WRITECOMMAND 0x80
#define BROADCAST_ID 0x3F

typedef enum
{
  isoUART_OK,
  isoUART_TIMEOUT,
  isoUART_CRC_ERROR
} iso_uart_status_t;

typedef enum
{
  NO_INIT,
  INIT_COMPLETE,
  RX_BUFFERSIZE_TO_SMALL
} driver_status_t;

typedef struct
{
  uint8_t n_pcvm;
  uint8_t bvm_sel;
  uint8_t ext_temp_sel;
  uint8_t ext_temp_r_sel;
  uint8_t int_temp_sel;
  uint8_t scvm_sel;
  uint8_t stress_pcvm_sel;
} multiread_cfg_t;

typedef struct
{
  uint16_t pcvm[12];
  uint16_t bvm;
  uint16_t ext_temp[5];
  uint16_t r_diag;
  uint16_t int_temp;
  uint16_t scvm[2];
  uint16_t stress_pcvm;
} multread_result_t;

typedef struct
{
  float ntc_resistance;
  float ntc_b_value;
} ntc_config_t;


typedef enum{
	SOURCE_320UA=0,
	SOURCE_80UA,
	SOURCE_20UA,
	SOURCE_5UA
}TempCurrentSource_t;

typedef struct
{
  uint16_t cell_voltages[12];
  uint16_t block_voltage;
  uint16_t ntc_resistances[5];
  uint16_t chiptemperature1;
  uint16_t chiptemperature2;
  uint16_t mailbox_register;
  uint16_t scvm_high;
  uint16_t scvm_low;

  uint16_t ext_temp_diag;

  uint8_t n_cells;

  ntc_config_t temperature_configs[5];

  uint16_t cell_uv_flags;
  uint16_t cell_ov_flags;
  uint16_t balancing_ov_flags;
  uint16_t balancing_uv_flags;
  uint16_t reg_crc_err;

} tle9012_device_t;

typedef enum
{
  OVERVOLTAGE_ERROR=0,
  UNDERVOLTAGE_ERROR,
  ADC_ERROR,
  INTERNAL_IC_ERROR,
  OPEN_LOAD_ERROR,
  REG_CRC_ERROR,
  EXTERNAL_TEMP_ERROR,
  INTERNAL_TEMP_ERROR,
  BALANCING_UNDERCURRENT_ERROR,
  BALANCING_OVERCURRENT_ERROR
} tle9012_error_t;

typedef enum
{
  PWM1000=0,
  PWM875,
  PWM750,
  PWM625,
  PWM500,
  PWM375,
  PWM250,
  PWM125
} tle9012_balancing_pwm_t;

typedef struct
{
  uint8_t adc_error : 1;
  uint8_t open_load_error : 1;
  uint8_t external_termperature_error : 1;
  uint8_t internal_temperature_error : 1;
  uint8_t undervoltage_error : 1;
  uint8_t overvoltage_error : 1;
  uint8_t balancing_undercurrent_error : 1;
  uint8_t balancing_overcurrent_error : 1;
} rr_error_mask_t;

typedef struct
{
  void (*overvoltage_callback)(uint8_t nodeID, uint16_t ov_flags);
  void (*undevoltage_callback)(uint8_t nodeID, uint16_t uv_flags);
  void (*adc_error_callback)(uint8_t nodeID, uint16_t filler);
  void (*internal_IC_error_callback)(uint8_t nodeID, uint16_t filler);
  void (*open_load_error_callback)(uint8_t nodeID, uint16_t diag_ol);
  void (*reg_crc_error_callback)(uint8_t nodeID, uint16_t reg_crc_err);
  void (*external_temp_error_callback)(uint8_t nodeID, uint16_t ext_temp_diag);
  void (*internal_temp_error_callback)(uint8_t nodeID, uint16_t internal_temp);
  void (*balancing_error_undercurrent_callback)(uint8_t nodeID, uint16_t bal_diag_uc);
  void (*balancing_error_overcurrent_callback)(uint8_t nodeID,uint16_t bal_diag_ov);

} tle9012_error_callbacks_t;


  class TLE9012{

    private:

      uint8_t crc8(uint8_t* buffer, uint16_t len);
      uint8_t crc3(uint8_t replyframe);

      uint8_t msb_first_converter(uint8_t* data,  uint16_t len);

      void isoUARTWriteRequest(uint8_t nodeID, uint8_t regaddress, uint16_t data);
      void isoUARTReadRequest(uint8_t nodeID, uint8_t regaddress);     
      void isoUARTClearRXBUffer();

    public:

      uint16_t isoUARTtimeout;
      driver_status_t driverStatus;


      TLE9012();  //Constructor
      ~TLE9012(); //Destructor

      

      
      
      void init(HardwareSerial* serial, uint32_t baudrate,uint8_t rxpin,uint8_t txpin);//Driver init
      void wakeUp();
      

      //High Level Routines

      //Measurement related functions
      void readCellVoltages(uint8_t nodeID);
      void readTemperatures(uint8_t nodeID);
      void setNumberofCells(uint8_t nodeID, uint8_t n_cells);
      void setNumberofTempSensors(uint8_t nodeID, uint8_t n_temp_sensors);

      //Watchdog and Power state handling
      void activateSleep();
      void resetWatchdog();
      void setExtendedWatchdog(uint8_t nodeID);
      void clearExtendedWatchdog(uint8_t nodeID);

      //Miscallanious stuff
      uint16_t readICVersionandManufacturerID(uint8_t nodeID);
      void setNodeID(uint8_t oldID, uint8_t newID, uint8_t finalNode);
      void writeMailboxRegister(uint8_t nodeID, uint16_t value);
      void readMailboxRegister(uint8_t nodeID);


      //Error checking and handling
      void checkDiagnoseResistor(uint8_t nodeID);
      void attachErrorHandler(tle9012_error_t errortype, void (*errorhandler)(uint8_t, uint16_t));
      void checkErrors(uint8_t nodeID);

      //Round Robin Functions

      void setRoundRobinErrorHandling(uint8_t nodeID, uint16_t rr_sleep_interval, uint8_t rr_temp_measurement_interval, uint8_t n_errors);
      void setRoundRobinConfig(uint8_t nodeID, uint8_t rr_counter, rr_error_mask_t errormask);

      //Cell Balancing Functions

      void setBalancingPWM(uint8_t nodeID, tle9012_balancing_pwm_t pwm_duty_cycle);
      void setBalancingCounter(uint8_t nodeID, uint8_t cell, uint8_t value);
      void startBalancing(uint8_t nodeID, uint16_t balancing_mask);

      //Threshold set functions
      void setOvervoltageThreshold(uint8_t nodeID, uint16_t fault_threshold);
      void setUndervoltageThreshold(uint8_t nodeID, uint16_t fault_threshold);
      void setOpenLoadThresholdMax(uint8_t nodeID, uint8_t open_load_threshold);
      void setOpenLoadThresholdMin(uint8_t nodeID, uint8_t open_load_threshold);
      void setExternalTemperatureThreshold(uint8_t nodeID, uint16_t external_overtemperature_threshold);
      void setInternalTemperatureThreshold(uint8_t nodeID, uint16_t internal_overtemperature_threshold);
      void setBalancingCurrentThreshold(uint8_t nodeID, uint8_t overcurrent_threshold, uint8_t undercurrent_threshold);


      //Low Level Routines for direct register Access
      iso_uart_status_t readRegisterSingle(uint8_t nodeID, uint16_t regaddress, uint16_t* result);  //Read data from a single register
      iso_uart_status_t writeRegisterSingle(uint8_t nodeID, uint16_t regaddress, uint16_t databuffer); //Write data to a single register

      iso_uart_status_t readRegisterBroadcast(uint16_t regaddress, uint16_t* result); //Write a broadcast to all devices in the daisy chain
      iso_uart_status_t writeRegisterBroadcast(uint16_t regaddress, uint16_t databuffer); //Read a register as broadcast from all devices in the chain

      iso_uart_status_t configureMultiread(multiread_cfg_t cfg);  //Write a multiread configuration to all devices in the daisy chain
      iso_uart_status_t multiRead(multread_result_t* databuffer); //Multiread command from all devices in the chain
    
  };



#endif