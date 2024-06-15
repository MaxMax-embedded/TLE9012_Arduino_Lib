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

void setup() {
  // put your setup code here, to run once:
  tle9012.init(&Serial2, 2000000);
  Serial.begin(115200);
  Serial.println("Boot completed");
  tle9012.wakeUp();
  delay(200);
  tle9012.writeRegisterSingle(0, 0x01, 0x0FFF);
  tle9012.writeRegisterSingle(0, 0x36, 0x0800);
}

void loop() {
    iso_uart_status_t status = isoUART_OK;
    uint16_t cellVoltages[12];
  // put your main code here, to run repeatedly:
  tle9012.wakeUp();
  delay(5);
  status = tle9012.writeRegisterBroadcast(0x3D, 0x007F);//Trigger WDT
  tle9012.writeRegisterBroadcast(0x18, 0xEE61);

  delay(5);
  uint8_t errors = 0;
  for(uint8_t n = 0; n < 12; n++)
  {
   errors += (uint8_t) tle9012.readRegisterSingle(0, 0x19+n, &cellVoltages[n]);
    
    Serial.print("Cell Voltage: ");
    Serial.print(n+1);
    Serial.print(" is ");
    Serial.print((float) 5*cellVoltages[n]/65536);
    Serial.print(" V\n");
  }
 
  Serial.print("Status: ");
  Serial.println(status+errors);
  delay(100);

}
