#ifndef readVCC_h
#define readVCC_h

#include "Arduino.h"
#include <avr/power.h>
#define adjTimer 8
//Read VCC voltage

long readVcc(){

  power_adc_enable();
  ADCSRA = _BV(ADEN) | _BV(ADIF); //enables de ADC and clear the INTERRUPT FLAG
  //ADMUX: REFS1, REFS0, ADLAR, REFS2, MUX3, MUX2, MUX1, MUX0
  //REFS[2:0]: Seleccionan el voltaje de referencia (Vref) para el ADC: defecto "000" (Usa vcc como voltaje de referencia, desconectando AREF)
  //ADLAR: Ajusta la conversión del ADC a la Derecha si es "1", por defecto la conversión es Left Adjusted.
  //MUX[3:0]: Analog Channel and Gain selection.
  //"1100": Channel selected Vbg - After switching to internal voltage reference the ADC requires a settling time of 1ms before measurements are stable. Conversions starting before this may not be reliable. The ADC must be enabled during the settling time.
  //Vbg is and internal Voltage Bandgap reference at 1.1V
  ADCSRA |= _BV(ADPS2); //Preescales of 16, for keep ADCclock between 50 and 200 KHz better Resolution
  ADMUX = _BV(MUX3)|_BV(MUX2);
  delay(8); //1ms wait time for ADC settling time.

  ADCSRA |= _BV(ADSC); //start conversion
  while(bit_is_set(ADCSRA, ADSC)); //wait until conversion is finished.

  ADCSRA |= _BV(ADSC); //start conversion
  while(bit_is_set(ADCSRA, ADSC)); //wait until conversion is finished.

  uint8_t low = ADCL;
  uint8_t high = ADCH;

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; //vcc en mV
}

#endif
