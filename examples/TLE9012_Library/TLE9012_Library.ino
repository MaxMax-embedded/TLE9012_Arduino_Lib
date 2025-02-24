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
//#include <Serial.h>


TLE9012 tle9012;

ntc_config_t ntc_config = {.ntc_resistance=100000,.ntc_b_value=3000};


void setup() {
  // put your setup code here, to run once:
  tle9012.init(&Serial2, 2000000,16,17);
  Serial.begin(115200);
  Serial.println("Boot completed");
  tle9012.wakeUp();
  delay(200);
  tle9012.setExtendedWatchdog(0);
  tle9012.setNodeID(0, 1, 1);
  tle9012.writeMailboxRegister(1, 0xAA55);

  uint16_t mailbox = tle9012.readMailboxRegister(1);
  if(mailbox == 0xAA55)
    Serial.println("Connection Check completed");

  tle9012.setNumberofCells(1, 12);
  tle9012.setTempSensorsConfig(1, 5, ntc_config);
  uint16_t ptconf = 0;
  tle9012.readRegisterSingle(1, 0x01, &ptconf);
  Serial.println(ptconf);
}

void loop() {

  // put your main code here, to run repeatedly:
  //tle9012.readTemperatures(1);
  /*
  for(uint8_t n = 0; n < 5; n++)
  {
    Serial.println("-------------------------------------------");
    Serial.print("Resistance ");
    Serial.print(n+1);
    Serial.print(" : ");
    Serial.print(tle9012.devices[0].ntc_resistances[n]);
    Serial.print("\n");
    Serial.println("-------------------------------------------");
  }*/
  tle9012.readChipTemperatures(1);

  Serial.print(INTERNAL_IC_TEMP_TO_DEGREE(tle9012.devices[0].chiptemperature1));
  Serial.print("    ");
  Serial.println(INTERNAL_IC_TEMP_TO_DEGREE(tle9012.devices[0].chiptemperature2));

  delay(1000);

}
