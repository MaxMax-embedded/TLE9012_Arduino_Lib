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

#define TXPIN 17
#define RXPIN 16

TLE9012 tle9012;


/**
 * This Example illustrates how to use low level register access to implement functionallity with no
 * corresponding higher level library function.
 * 
 * This example was written for a ESP32-Node MCU Development Board
 */

void setup() {
  
  //To keep the example short, the initilization is performed using higher level functions
  tle9012.init(&Serial2, 2000000,RXPIN,TXPIN);
  Serial.begin(115200);
  Serial.println("Boot completed");
  tle9012.wakeUp();
  delay(200);

  tle9012.setNodeID(0, 1, 1);
  tle9012.writeMailboxRegister(1, 0xAA55);

  uint16_t mailbox = tle9012.readMailboxRegister(1);
  if(mailbox == 0xAA55)
    Serial.println("Connection Check completed");

  tle9012.writeRegisterSingle(1,PART_CONFIG,0x0FFF); // Set Part Config Register 

}

void loop() {

  // put your main code here, to run repeatedly:
  tle9012.writeRegisterBroadcast(WDOG_CNT,0x007F); // Broadcast Write Command to reset Watchdog Counter

  tle9012.writeRegisterSingle(1, MEAS_CTRL, 0xEE61); //Start cell measurement
  delay(10); //Wait till measurement is completed

  uint16_t cell0_voltage = 0; 
  iso_uart_status_t status = isoUART_OK;

  status = tle9012.readRegisterSingle(1,PCVM_0,&cell0_voltage); //Read cell voltage and communication status

  //Check if the bus action was transfered successfully
  if(status != isoUART_OK)
  {
    Serial.print("Error during bus communication");
  }

  delay(1000);

}
