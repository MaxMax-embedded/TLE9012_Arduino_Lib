#include "../include/TLE9012_Matlab.h"
#include "../../TLE9012.h"


#define N_CELLS 6
#define TXPIN 17
#define RXPIN 16

TLE9012 tle9012;
uint8_t tle_init_cplt = 0;

extern "C" void init_tle9012(void)
{
  tle9012.init(&Serial2, 2000000,RXPIN,TXPIN); //Initialize driver with 2Mbit

  tle9012.wakeUp(); //Issue wakeup command
  tle9012.setExtendedWatchdog(0);
  delay(200); //Wait some time to ensure wakeup was completed

  tle9012.setNodeID(0, 1, 1); //Set device address from 0 to 1 and set device 1 as final Node

  tle9012.writeMailboxRegister(1, 0xAA55);  //Write and Readback Mailbox register to check if communication works
  uint16_t mailbox = tle9012.readMailboxRegister(1);

  if(mailbox == 0xAA55) //Show Successfull communication
    tle_init_cplt = 1;

  tle9012.setNumberofCells(1, N_CELLS); //Configure the number of cells
  tle9012.setBAVMConfig(1,1);
  tle9012.resetWatchdog(); //Reset Watchdog Timer
  tle_init_cplt = 1;
}

extern "C" void get_cell_voltages(float* voltages)
{
    if(tle_init_cplt == 0)
        return;

    tle9012.resetWatchdog(); //Reset Watchdog Timer
    tle9012.readCellVoltages(1);  //Read all cell voltages from device 1
    for(uint8_t n = 0; n < N_CELLS; n++)
        voltages[n] = ADCVALUE_TO_FLOAT_VOLTAGE(tle9012.devices[0].cell_voltages[n]);
}

extern "C" void get_cell_voltages_with_current(float* voltages, float* current)
{
  if(tle_init_cplt == 0)
        return;

    tle9012.resetWatchdog(); //Reset Watchdog Timer
    tle9012.readCellVoltagesWithBAVM(1);  //Read all cell voltages from device 1
    for(uint8_t n = 0; n < N_CELLS; n++)
        voltages[n] = ADCVALUE_TO_FLOAT_VOLTAGE(tle9012.devices[0].cell_voltages[n]);

    current[0] = (float)tle9012.devices[0].bipolar_auxilary_voltage*2.0/32768 * 5;
}

extern "C" void tle9012_Terminate(void)
{

}