#ifndef PCF8591_H
#define PCF8591_H


int setupPCF8591(uint8_t addr);
float getSupplyVoltage(void);
float getSupplyBattery1(void);
float getSupplyCurrent(void);
void readPowerSupply(void);


#endif // PCF8591_H

