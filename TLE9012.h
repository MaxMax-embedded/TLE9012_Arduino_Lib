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


  class TLE9012{

    private:


      uint8_t crc8(uint8_t* buffer, uint16_t len);
      uint8_t msb_first_converter(uint8_t* data,  uint16_t len);

      void isoUARTWriteRequest(uint8_t nodeID, uint8_t regaddress, uint16_t data);
      void isoUARTReadRequest(uint8_t nodeID, uint8_t regaddress);     
      void isoUARTClearRXBUffer();

    public:

      uint16_t isoUARTtimeout;
      driver_status_t driverStatus;


      TLE9012();  //Constructor
      ~TLE9012(); //Destructor

      

      
      
      void init(HardwareSerial* serial, uint32_t baudrate);//Driver init
      void wakeUp();
      
      //Low Level Routines for direct register Access
      iso_uart_status_t readRegisterSingle(uint8_t nodeID, uint16_t regaddress, uint16_t* result);  //Read data from a single register
      iso_uart_status_t writeRegisterSingle(uint8_t nodeID, uint16_t regaddress, uint16_t databuffer); //Write data to a single register

      iso_uart_status_t readRegisterBroadcast(uint16_t regaddress, uint16_t* result); //Write a broadcast to all devices in the daisy chain
      iso_uart_status_t writeRegisterBroadcast(uint16_t regaddress, uint16_t databuffer); //Read a register as broadcast from all devices in the chain

      iso_uart_status_t configureMultiread(multiread_cfg_t cfg);  //Write a multiread configuration to all devices in the daisy chain
      iso_uart_status_t multiRead(multread_result_t* databuffer); //Multiread command from all devices in the chain
    
  };



#endif