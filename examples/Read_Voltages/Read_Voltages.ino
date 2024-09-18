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

#define N_CELLS 6
#define TXPIN 10
#define RXPIN 11

TLE9012 tle9012;

/**
 * This Example shows how to do a basic configuration of a single TLE9012 and read cell voltages
 * 
 */

void setup() {
  
  tle9012.init(&Serial1, 2000000,RXPIN,TXPIN); //Initialize driver with 2Mbit


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
  tle9012.setBAVMConfig(1,1);
  tle9012.resetWatchdog(); //Reset Watchdog Timer
  tle9012.clearExtendedWatchdog(1);
}

void loop() {
  
  tle9012.resetWatchdog(); //Reset Watchdog Timer
  tle9012.readCellVoltagesWithBAVM(1);  //Read all cell voltages from device 1

  for(uint8_t n = 0; n < N_CELLS; n++)  //Print cell voltages to Serial Monitor
  {
    Serial.print("Cell Voltage ");
    Serial.print(n+1);
    Serial.print(": ");
    Serial.println(ADCVALUE_TO_FLOAT_VOLTAGE(tle9012.devices[0].cell_voltages[n]),4);
  }

  Serial.print("BAVM: ");
  Serial.println(convertBAVMtoCurrent(tle9012.devices[0].bipolar_auxilary_voltage),5);

  Serial.println("-------------------------------------------------");

  delay(1000);

}

float convertBAVMtoCurrent(int16_t shuntval)
{
  return ((2*(float)shuntval)/32768)/0.2;
}
