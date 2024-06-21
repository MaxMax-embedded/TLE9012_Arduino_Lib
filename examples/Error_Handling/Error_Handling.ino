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


#define N_CELLS 12
#define TXPIN 17
#define RXPIN 16

#define OVERVOLTAGE_THRESHOLD 4.2
#define UNDERVOLTAGE_THRESHOLD 2.5

TLE9012 tle9012;

/**
 * This example shows how to register error handlers and perform periodic checks
 * 
 */

void setup() {

  tle9012.init(&Serial2, 2000000,RXPIN,TXPIN); //Initialize driver with 2Mbit

  Serial.begin(115200);
  Serial.println("Boot completed");

  tle9012.wakeUp(); //Issue wakeup command
  delay(200); //Wait some time to ensure wakeup was completed

  tle9012.setNodeID(0, 1, 1); //Set device address from 0 to 1 and set device 1 as final Node

  tle9012.writeMailboxRegister(1, 0xAA55);  //Write and Readback Mailbox register to check if communication works
  uint16_t mailbox = tle9012.readMailboxRegister(1);

  if(mailbox == 0xAA55) //Show Successfull communication
    Serial.println("Connection Check completed");

  tle9012.setNumberofCells(1, N_CELLS); //Configure the number of cells

  tle9012.setOvervoltageThreshold(1,VOLTAGE_TO_OV_UV_LIMIT(OVERVOLTAGE_THRESHOLD)); //Set voltage Limits
  tle9012.setUndervoltageThreshold(1,VOLTAGE_TO_OV_UV_LIMIT(UNDERVOLTAGE_THRESHOLD));

  tle9012.attachErrorHandler(UNDERVOLTAGE_ERROR,underVoltageErrorHandler);  //Register Error Handlers
  tle9012.attachErrorHandler(OVERVOLTAGE_ERROR,overVoltageErrorHandler);
  tle9012.resetWatchdog(); //Reset Watchdog Timer

}

void loop() {
  
  tle9012.resetWatchdog();  //Reset Watchdog Timer
  tle9012.readCellVoltages(1); //Optional -> Read out cell voltages. Errors will trigger independent of this measurement
  tle9012.checkErrors(1); //Check for Errors and Call Handlers
  
  delay(1000);
}

/**
 * @brief Error handler function that gets called in case of undervoltage errors
 * 
 * @param nodeID address of the device with a fault
 * @param uv_flags bitmask containing what cells are below the undervoltage threshold
 */
void underVoltageErrorHandler(uint8_t nodeID, uint16_t uv_flags)
{
  Serial.println("Undervoltage Error Detected");
  Serial.print("Undervoltage Flag Mask: ");
  Serial.println(uv_flags,BIN);
  tle9012.resetErrors(1);
}


/**
 * @brief Error handler function that gets called in case of overvoltage errors
 * 
 * @param nodeID address of the device with a fault
 * @param uv_flags bitmask containing what cells are above the overvoltage threshold
 */

void overVoltageErrorHandler(uint8_t nodeID, uint16_t ov_flags)
{
  Serial.println("Overvoltage Error Detected");
  Serial.print("Overvoltage Flag Mask: ");
  Serial.println(ov_flags,BIN);
  tle9012.resetErrors(1);
}