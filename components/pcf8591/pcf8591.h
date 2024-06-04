#ifndef PCF8591_H
#define PCF8591_H


int setupPCF8591(uint8_t addr);
void readPowerSupply(uint32_t *voltage, uint32_t *battery1, uint32_t *current);
uint32_t getSupplyVoltage(void);


#endif // PCF8591_H

