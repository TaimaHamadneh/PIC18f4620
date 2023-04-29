#include "my_adc.h"

int read_adc_raw_no_lib(unsigned char channel) {
    int raw_value;
    ADCON0bits.CHS = channel;
    //start conversion
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO) {
    }; // wait until conversion is done
    raw_value = ADRESH << 8 | ADRESL; // 10 bit, need to shift the bits right    
    return raw_value; // 16 bit , it is actually 10 bit 0 ---1023
}

float read_adc_voltage(unsigned char channel) {
    int raw_value;
    float voltage;
    raw_value = read_adc_raw_no_lib(channel);
    voltage = (raw_value * 5) / 1023.0;
    return voltage;
}

