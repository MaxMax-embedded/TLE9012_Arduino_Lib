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


//-----------------------------------------------------------------------------
//                          Defines for Lib Configuration
//-----------------------------------------------------------------------------

#define N_DEVICES 1 //Number of devices in the daisy chain
#define ISOUART_TIMEOUT 100 //IsoUART Timeout in Milliseconds

#define SOFT_MSB_FIRST //undef in case the hardware serial port can be configured to handle MSB First in Hardware
//#define THREAD_SAFE //Uncomment if Thread safety is required -> add code to mutex lock makros for your RTOS/Scheduler

//-----------------------------------------------------------------------------
//                          Useful conversion makros
//-----------------------------------------------------------------------------

const float current_source_exp_lookup[] = {1.0, 4.0, 16.0, 64.0};

#define ADCVALUE_TO_FLOAT_VOLTAGE(value) (5.0/65536 * (float) value)
#define NTC_MEASUREMENT_TO_R(value,r_temp_filter) ((2.0*(value&0x03FF)*current_source_exp_lookup[(value>>11)&0x03])/(1024*0.000320) - r_temp_filter)
#define VOLTAGE_TO_OC_UC_LIMIT(voltage) ((uint16_t) ((voltage/5.0) * 1024))
#define VOLTAGE_TO_OV_UV_LIMIT(voltage) ((uint16_t) ((voltage/5.0) * 1024))
#define VOLTAGE_TO_OL_THRESHOLD(voltage) ((uint8_t) voltage/0.0195) //note max input voltage should not exceed ~1.2285V
#define ADCVALUE_TO_FLOAT_BLOCKVOLTAGE(value) (60.0/65536 * (float) value)
#define INTERNAL_IC_TEMP_TO_DEGREE(value) (-0.6624 * (float) (value & 0x3FF) + 547.3) //Kelvin/degree conversion already included?
#define DEGREE_TO_IC_TEMP_LIMIT(degree) (((uint16_t)((degree - 547.3)/(-0.6624)))&0x3FF) //Kelvin/degree conversion already included?

//-----------------------------------------------------------------------------
//                          Defines for Lib functions
//-----------------------------------------------------------------------------

#define WRITECOMMAND 0x80 
#define BROADCAST_ID 0x3F


//Define calls to Mutex Locks here
#ifdef THREAD_SAFE
#define ISOUART_LOCK()
#define ISOUART_UNLOCK()
#else
#define ISOUART_LOCK()
#define ISOUART_UNLOCK()
#endif

//-----------------------------------------------------------------------------
//                          Typedefs start here
//-----------------------------------------------------------------------------


/** Describes the busstate after an command that accesses the isoUART
 *  mostly used for return types
 */
typedef enum
{
  isoUART_OK, /**< Transaction was completed without issues and response was received */
  isoUART_TIMEOUT, /**< A timeout occured */
  isoUART_CRC_ERROR /**< The CRC Value of the response was corrupted */
} iso_uart_status_t;

/** Describes the state of the driver. This enum will be used to make the library threadsafe in further releases
 * 
 */
typedef enum
{
  NO_INIT,
  INIT_COMPLETE,
  RX_BUFFERSIZE_TO_SMALL
} driver_status_t;

/** Describtor to configure multiread accesses in the future
 * 
 */
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

/** Data structure holding the results of a multiread operation
 * 
 */
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

/** Data structure to describe properties for NTC Resistors. The current model
 *  uses a base resistance and a b value to calculate temperature following the formula
 * T(R) = 1/((ln(R/ntc_resistance)/ntc_b_value) + (1/basetemp))
 */
typedef struct
{
  float ntc_resistance; //Base Value of the NTC
  float ntc_b_value;  //Beta value of the NTC
  float basetemp; //Temperature of the NTC at specified resistance in Kelvin
} ntc_config_t;

/**
 * Different current sources used inside the TLE9012 to measure ntc resistance.
 * Current sources are sorted in a way to simplify resistance calculation
 */
typedef enum{
	SOURCE_320UA=0, /**< Internal 320uA Current Source*/
	SOURCE_80UA, /**< Internal 80uA Current Source */
	SOURCE_20UA, /**< Internal 20uA Current Source */
	SOURCE_5UA /**< Internal 5uA Current Source */
}TempCurrentSource_t;

/**
 * Device struct that holds all relevant data that configure the TLE9012 and measurements from the IC
 * 
 */
typedef struct
{
  uint16_t cell_voltages[12]; /**< Array that hold up to 12 cell voltages that can be measured */
  uint16_t block_voltage; /**< Block Voltage of the complete stack */
  uint16_t ntc_resistances[5]; /**< Measured NTC Resistance */
  uint8_t ntc_results_valid;
  uint16_t chiptemperature1; /**< Temperature of internal Temperature Sensor Nr. 1 */
  uint16_t chiptemperature2; /**< Temperature of internal Temperature Sensor Nr. 2 */
  uint8_t chiptemperatures_valid;
  uint16_t mailbox_register; /**< Content of the Mailbox Register */
  uint16_t scvm_high; /**< Highest value measured by the secondary measurement path */
  uint16_t scvm_low; /**< Lowest value measured by the secondary measurement path */

  uint16_t ext_temp_diag; /**< value of the external temperature diagnostic register */

  uint8_t n_cells; /**< Number of monitored cells */
  uint8_t n_temp_sensors; /**< Number of monitored temperature sensors */

  ntc_config_t sensorconfig; /**< NTC configuration for the IC */

  uint16_t cell_uv_flags; /**< Undervoltage flag status */
  uint16_t cell_ov_flags; /**< Overvoltage flag status */
  uint16_t balancing_ov_flags; /**< Balancing overcurrent flag status */
  uint16_t balancing_uv_flags; /**< Balancing undercurrent flag status */
  uint16_t reg_crc_err; /**< REG CRC Error register state */

} tle9012_device_t;

/**
 * Enum containing all different types of error handlers.
 * Mainly used as argument to attach Error handlers
 */
typedef enum
{
  NO_ERROR, /**< No error was detected */
  OVERVOLTAGE_ERROR=0, /**<  Cell overvoltage was detected by either the ADC or DAC comparators*/
  UNDERVOLTAGE_ERROR, /**< Cell undervoltage was detected by either the ADC or DAC comparators*/
  ADC_ERROR, /**< An internal error with the ADC was detected. e.g. missmatch between PCVM and SCVM */
  INTERNAL_IC_ERROR, /**< An internal IC Error was detected */
  OPEN_LOAD_ERROR, /**< An open load error was detected e.g. open wire to a cell */
  REG_CRC_ERROR, /**< Internal register content was found to be corrupted e.g through writing to reserved bitfields */
  EXTERNAL_TEMP_ERROR, /**< External overtemperature threshold was violated */
  INTERNAL_TEMP_ERROR, /**< Internal Chip temperature limit was violated */
  BALANCING_UNDERCURRENT_ERROR, /**< Balancing current is lower than expected */
  BALANCING_OVERCURRENT_ERROR /**< Balancing current is higher than expected */
} tle9012_error_t;

/**
 * The cell balancing unit allows to use a PWM to allow for some regulation of the balancing current.
 * The PWM duty cycle can vary between 12.5% and 100% in 12.5% steps
 */
typedef enum
{
  PWM1000=0, /**< 100% on time for balancing PWM */
  PWM875,    /**< 87.5% on time for balancing PWM */
  PWM750,    /**< 75% on time for balancing PWM */
  PWM625,    /**< 62.5% on time for balancing PWM */
  PWM500,    /**< 50% on time for balancing PWM */
  PWM375,    /**< 37.5% on time for balancing PWM */
  PWM250,    /**< 25% on time for balancing PWM */
  PWM125     /**< 12.5% on time for balancing PWM */
} tle9012_balancing_pwm_t;


/**
 * This struct holds information about the error counter masks for the round robin cycle.
 * If a flag is set to 0, the errorcounter is activated and a fault will not directly lead to an error and EMM signal.
 * If the flag is set to 1, the fault will be registered at first detection ignoring the error counter.
 * 
 * See RR_Config register at offset 0x0009 in the user manual for further informations
 */

typedef struct
{
  uint8_t adc_error : 1; /**<  Mask for ADC related errors*/
  uint8_t open_load_error : 1; /**< Mask for open load related errors */
  uint8_t external_termperature_error : 1; /**< Mask for external temperature sensor errors */
  uint8_t internal_temperature_error : 1; /**< Mask for internal temperature sensor errors */
  uint8_t undervoltage_error : 1; /**< Mask for cell undervoltage errors */
  uint8_t overvoltage_error : 1; /**< Mask for cell overvoltage errors */
  uint8_t balancing_undercurrent_error : 1; /**< Mask for undercurrent during balancing errors */
  uint8_t balancing_overcurrent_error : 1; /**< Mask for overcurrent during balancing errors */
} rr_error_mask_t;

/**
 * This struct holds error flag masks that decide if the Error Pin is set and a Emergency Message is triggered when 
 * a fault is detected.
 * 
 * If the corresponding flag is set to 0, a detected fault will NOT trigger the error. If the flag is set to 1 an error will
 * be triggered.
 * 
 * Note that the err_pin flag has a special role. If set to 0, an Emergency Message will be sent over the isoUART in case a 
 * fault is detected but the error pin function is deactivated! (If using a TLE9015 transceiver, the error pin on the transceiver
 * will be triggered if an Emergency Message Signal is received)
 * 
 * If the err_pin flag is set to 1, no EMM Message will be send in case of a fault, but the error pin will be set instead
 */
typedef struct
{
  uint8_t err_pin : 1; /**< control flag for the EMM/Error Pin behaviour */
  uint8_t adc_error : 1; /**< error mask for adc errors */
  uint8_t open_load_error : 1; /**< error mask for open load errors*/
  uint8_t int_ic_err : 1; /**< error mask for internal IC errors */
  uint8_t reg_crc_err : 1; /**< error mask for register file CRC errors */
  uint8_t external_termperature_error : 1; /**< error mask for external temperature errors */
  uint8_t internal_temperature_error : 1; /**< error mask for internal temperature errors */
  uint8_t undervoltage_error : 1; /**< error mask for undervoltage errors */
  uint8_t overvoltage_error : 1; /**< error mask for overvoltage errors */
  uint8_t balancing_undercurrent_error : 1; /**< error mask for balancing undercurrent errors */
  uint8_t balancing_overcurrent_error : 1; /**< error mask for balancing overcurrent errors */
} err_emm_error_mask_t;


/**
 * This struct holds function pointers to different error handlers.
 * 
 * Calling the checkErrors() function will check the GEN_DIAG register for faults and if a fault is detected
 * tries to call the appropriate handler. If the handler is a null pointer, which is the default after initilization,
 * no handler will be called. To assign an error handler function to a fault type, the attachErrorHandler function can be used
 * 
 */
typedef struct
{
  void (*overvoltage_callback)(uint8_t nodeID, uint16_t ov_flags); /**< handler for overvoltage errors */
  void (*undervoltage_callback)(uint8_t nodeID, uint16_t uv_flags); /**< handler for undervoltage errors */
  void (*adc_error_callback)(uint8_t nodeID, uint16_t filler); /**< handler for adc errors */
  void (*internal_IC_error_callback)(uint8_t nodeID, uint16_t filler); /**< handler for internal IC errors */
  void (*open_load_error_callback)(uint8_t nodeID, uint16_t diag_ol); /**< handler for open load errors*/
  void (*reg_crc_error_callback)(uint8_t nodeID, uint16_t reg_crc_err); /**< handler for register file crc errors */
  void (*external_temp_error_callback)(uint8_t nodeID, uint16_t ext_temp_diag); /**< handler for external temperature violations */
  void (*internal_temp_error_callback)(uint8_t nodeID, uint16_t internal_temp); /**< handler for internal temperature violations */
  void (*balancing_error_undercurrent_callback)(uint8_t nodeID, uint16_t bal_diag_uc); /**< handler for undercurrent events during balancing */
  void (*balancing_error_overcurrent_callback)(uint8_t nodeID,uint16_t bal_diag_ov); /**< handler for overcurrent events during balancing */
  void (*ps_error_sleep_callback)(uint8_t nodeID, uint16_t filler); /**< handler if loss of supply induced sleep was detected */

} tle9012_error_callbacks_t;


//-----------------------------------------------------------------------------
//                          Class and function definitions
//-----------------------------------------------------------------------------


/*!
 * Class containing all functions related to the TLE9012 and isoUART
 * 
 * Please treat this class as a singleton class meaning only one object instance is allowed.
 * 
 */

  class TLE9012{

    private:

      tle9012_error_callbacks_t errorcallbacks;


      uint8_t crc8(uint8_t* buffer, uint16_t len);
      uint8_t crc3(uint8_t replyframe);

      uint8_t msb_first_converter(uint8_t* data,  uint16_t len);

      void isoUARTWriteRequest(uint8_t nodeID, uint8_t regaddress, uint16_t data);
      void isoUARTReadRequest(uint8_t nodeID, uint8_t regaddress);     
      void isoUARTClearRXBUffer();

      void mcuDelay(uint32_t delay_ms);

    public:

      uint16_t isoUARTtimeout;
      driver_status_t driverStatus; //Status of the driver; Might be used to ensure Thread safety in the future
      tle9012_device_t devices[N_DEVICES]; //device structs holding data like cell voltages etc


      TLE9012();  //Constructor
      ~TLE9012(); //Destructor

      

      
      
      void init(HardwareSerial* serial, uint32_t baudrate,uint8_t rxpin,uint8_t txpin);//Driver init
      void wakeUp();
      

      //High Level Routines

      //Measurement related functions
      void readCellVoltages(uint8_t nodeID);
      void readTemperatures(uint8_t nodeID);
      void setNumberofCells(uint8_t nodeID, uint8_t n_cells);
      void setTempSensorsConfig(uint8_t nodeID, uint8_t n_temp_sensors,ntc_config_t sensorconfig);
      void readChipTemperatures(uint8_t nodeID);

      //Watchdog and Power state handling
      void activateSleep();
      void resetWatchdog();
      void setExtendedWatchdog(uint8_t nodeID);
      void clearExtendedWatchdog(uint8_t nodeID);

      //Miscallanious stuff
      uint16_t readICVersionandManufacturerID(uint8_t nodeID);
      void setNodeID(uint8_t oldID, uint8_t newID, uint8_t finalNode);
      void writeMailboxRegister(uint8_t nodeID, uint16_t value);
      uint16_t readMailboxRegister(uint8_t nodeID);


      //Error checking and handling
      uint8_t checkDiagnoseResistor(uint8_t nodeID);
      void attachErrorHandler(tle9012_error_t errortype, void (*errorhandler)(uint8_t, uint16_t));
      void checkErrors(uint8_t nodeID);
      void resetErrors(uint8_t nodeID);
      void configFaultMasks(uint8_t nodeID, err_emm_error_mask_t err_mask);

      //Round Robin Functions

      void setRoundRobinErrorHandling(uint8_t nodeID, uint16_t rr_sleep_interval, uint8_t rr_temp_measurement_interval, uint8_t n_errors);
      void setRoundRobinConfig(uint8_t nodeID, uint8_t rr_counter, uint8_t rr_sync, rr_error_mask_t errormask);

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