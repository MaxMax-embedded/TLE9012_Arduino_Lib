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
#include <math.h>

#define TXPIN 17
#define RXPIN 16
#define N_TEMPS 5

TLE9012 tle9012;

ntc_config_t ntc_cfg = {.ntc_resistance=1000,.ntc_b_value=3348 ,.basetemp = 298.15};

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

  tle9012.setTempSensorsConfig(1,N_TEMPS,ntc_cfg); //Set temperature sensor config
  tle9012.resetWatchdog(); //Reset Watchdog Timer

}

void loop() {
  
  tle9012.resetWatchdog(); //Reset Watchdog Timer
  tle9012.readTemperatures(1); //Measure Temperatures

  for(uint8_t n = 0; n < N_TEMPS; n++) //Convert and Print Temperature over Serial
  {
    float ntc_resistance = NTC_MEASUREMENT_TO_R(tle9012.devices[0].ntc_resistances[n],100); //get ntc resistance assuming 100 Ohm Rf for NTC
    Serial.print("NTC Resistance: ");
    Serial.print(ntc_resistance);
    Serial.print("Ohm  |  Temperature: ");
    float temperature = r_to_temp(ntc_cfg,ntc_resistance); //Calculate temperature from ntc resistance
    Serial.print(temperature);
    Serial.println("°C");
  }
  Serial.println("-----------------------------------------------------------------------");

  delay(1000);
}

/**
 * This function is deliberatly not included in the library to prevent a forced inclusion of math.h
 */

float r_to_temp(ntc_config_t cfg, float resistance)
{
  return 1/((log(resistance/cfg.ntc_resistance)/cfg.ntc_b_value) + (1/cfg.basetemp))-273.15;
}