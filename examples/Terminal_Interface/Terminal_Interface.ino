#include "TLE9012.h"
#include <stdio.h>

#define TXPIN 17
#define RXPIN 16

TLE9012 tle9012;

//Watchdog Kicker Status Variables
uint8_t watchdog_active = 0;
uint16_t watchdog_time = 0;
uint32_t last_wd_trigger = 0;

//Command Line Buffer
uint8_t terminalbufferrecvlen = 0;
char terminalbuffer[64];


void setup() { 
  Serial.begin(115200); //Start Console interface
  tle9012.init(&Serial2, 2000000,RXPIN,TXPIN); //Initialize driver with 2Mbit
}

void loop() {
  
  handleTerminal(); //Handle the command line interface

  //Check if Watchdog timer is activated and trigger a reset if watchdog_time parameter expires
  if(watchdog_active)
  {
    if((millis()-last_wd_trigger) > watchdog_time)
    {
      tle9012.resetWatchdog();
      last_wd_trigger = millis();
    }
  }

}

/**
 * @brief Handel the command line buffer and look for EOL characters
 * 
 * This function checks if new data is available over the Serial Stream
 * If new data was found, it is copied into a separate commandbuffer
 * 
 * If an overflow of the commandbuffer was detected, the buffer is cleared and resetted before new data is copied into the buffer.
 * On detecting an \n Character, the buffer is passed to the interpretCommand function
 */

void handleTerminal()
{

  //Check if new data was received
  uint8_t rx_len = Serial.available();
  
  //Handle overflow situations by resetting the buffer
  if((rx_len+terminalbufferrecvlen) > 64)
  {
    for(uint8_t n = 0; n < 64; n++)
      terminalbuffer[n] = 0;

      terminalbufferrecvlen = 0;
  }

  //Read Bytes from Serial RX Buffer to terminalbuffer
  Serial.readBytes(&terminalbuffer[terminalbufferrecvlen], rx_len);

  //Check newly received data for \n end of line character
  uint8_t eol_char_found = 0;
  for(uint8_t n = terminalbufferrecvlen; n < (terminalbufferrecvlen+rx_len); n++)
  {
    if(terminalbuffer[n] == '\n') eol_char_found = 1;
  }

  terminalbufferrecvlen += rx_len; //Increase received length by number of received characters

  //Call interpret command function and clean up terminalbuffer
  if(eol_char_found)
  {
    interpretCommand(terminalbuffer, terminalbufferrecvlen);

    for(uint8_t n = 0; n < 64; n++)
      terminalbuffer[n] = 0;

    terminalbufferrecvlen = 0;
  }

}

/**
 * @brief This function interprets a command line and executes a command if a valid input was detected
 * 
 * @param commandbuffer buffer containing a complete command line string
 * @param bufferlength length of the buffer
 */
void interpretCommand(const char commandbuffer[], uint8_t bufferlength)
{
    char cmd[64]; //Size might be reduced if no robustness against bufferoverflows due to excessive input string length can be accepted

    if(sscanf(commandbuffer,"%s",cmd)  == 1) //Read command
    {
      if(!strcmp(cmd,"IL")) //Emulate IL Command (no function because no ring bus mode is supported by this library)
      {
        Serial.println("IL OK");
      }

      else if(!strcmp(cmd,"IH")) //Emulate IH Command (no function because no ring bus mode is supported by this library)
      {
        Serial.println("IH OK");
      }

      else if((strcmp(cmd,"WH") == 0) | (strcmp(cmd,"WL") == 0)) //Write data to a single register of a single device on the bus
      {
        uint8_t dev_address;
        uint8_t reg_address;
        uint16_t data;
        if(sscanf(commandbuffer, "%s %d %x %x", cmd, &dev_address, &reg_address, &data) == 4)
        {
          iso_uart_status_t status = tle9012.writeRegisterSingle(dev_address, reg_address, data);
          if(status == isoUART_OK)
          {
            Serial.println("OK 8000");
          }
          if(status == isoUART_TIMEOUT)
          {
            Serial.println("TIMEOUT");
          }
          if(status == isoUART_CRC_ERROR)
          {
            Serial.println("CRC ERROR");
          }
        }
        else
        {
          Serial.println("INVALID COMMAND FORMAT");
        }
      }

      else if((strcmp(cmd,"RH") == 0) | (strcmp(cmd,"RL") == 0)) //Read data from a single register of a single device on the bus
      {

        uint8_t dev_address;
        uint8_t reg_address;
        uint16_t data;

        if(sscanf(commandbuffer, "%s %d %x", cmd, &dev_address, &reg_address) == 3)
        {
          iso_uart_status_t status = tle9012.readRegisterSingle(dev_address, reg_address, &data);
          if(status == isoUART_OK)
          {
            Serial.print(data,HEX);
            Serial.print(" OK ");
            Serial.println("8000");
          }
          if(status == isoUART_TIMEOUT)
          {
            Serial.println("TIMEOUT");
          }
          if(status == isoUART_CRC_ERROR)
          {
            Serial.println("CRC ERROR");
          }
        }
        else
        {
          Serial.println("INVALID COMMAND FORMAT");
        }        
      }

      else if(!strcmp(cmd,"K")) //Activate watchdog kicker (deactivated for time = 0)
      {
        uint16_t wd_time = 0;
        if(sscanf(commandbuffer, "%s %d", cmd, &wd_time) == 2)
        {
          if(wd_time != 0)
          {
            watchdog_time = wd_time;
            watchdog_active = 1;
            Serial.print("Watchdog kicking time change to ");
            Serial.print(watchdog_time);
            Serial.println(" ms");
          }
          else
          {
             watchdog_active = 0;
             Serial.println("Watchdog kicker deactivated");
          }
        }
      }
      else //No valid command was detected
      {
        Serial.println("INVALID COMMAND");
      }

    }
}