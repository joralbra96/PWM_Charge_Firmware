#include "readVcc.h"
#include <SoftwareSerial.h>

#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/sleep.h>



#define PUMP_1 0
#define PUMP_2 1
#define DRV_PIN 4
#define LED_Pin  3 //LED
#define VBAT_PIN 2
#define AR_VOLTAGE_CONST 163209

#define wdt_int() WDTCR |= _BV(WDIE) // | _BV(WDCE) | _BV(WDE) // WDT goes to interrupt, not reset

unsigned int floatV = 1350;
unsigned int vcc = 500;


unsigned int voltage = 1350;
int stepSize = 0;
int pulseWidth = 0;

//SoftwareSerial mySerial (DRV_PIN, DRV_PIN);//Se activa la comunicación serial como tx en el puerto 1 del attiny

//when ADC completed, take an interrupt
EMPTY_INTERRUPT (ADC_vect);


void setup() {
  pinMode(PUMP_1, OUTPUT); // Pump 2 clock
  pinMode(PUMP_2, OUTPUT); //Pump 1 clock
  pinMode(DRV_PIN, OUTPUT); //Driver PWM
  pinMode(LED_Pin, OUTPUT);  // LED

  power_usi_disable(); // Power Register - Shuts down the USI (used in i2c and SPI interfaces)
  PWR_stop_ADC(); // Stop ADC to save power

  set_sleep_mode(SLEEP_MODE_IDLE); // Configure attiny85 sleep mode

  PWM_setup_Pump();  // Configure registers for most efficient charge pump
  PWM_setup_Driver();  // Configure registers for mosfet driver pin
  OCR1B = 0; //Write mosfet pin Low

//  mySerial.begin(9600);
 // mySerial.println("Empieza");

  WDT_Sleep(WDTO_2S);
  
}

void loop() {
  //Getting ADC reading
  vcc = readVcc();
  unsigned long newReading = 0;
  for (uint8_t i = 0; i < 16; i++){  //toma 16 muestras para hacer una división para un numero 2^n
    newReading = newReading + getReading(); //estas son las lecturas en binario (0 - 1024)
  }
  newReading = newReading/16; //This is a bitwise operation to make a division by 16 (newReading/16)
  //covnersion a centivoltios.
  newReading = (newReading*vcc*100)/AR_VOLTAGE_CONST;
  voltage = (voltage*8 + newReading*2)/10; //Here is the voltage in centivoltios

  stepSize = floatV - voltage; //can be negative
  pulseWidth += stepSize; //can decrease too!
  pulseWidth = constrain(pulseWidth, 0, 255);
  OCR1B = pulseWidth;

//  mySerial.print("voltage: ");
//  mySerial.print(voltage);
//  mySerial.print(" - pulse:");
//  mySerial.println(pulseWidth);
//  mySerial.flush();


  set_sleep_mode(SLEEP_MODE_IDLE); // Configure attiny85 sleep mode
  sleep_enable();
  WDT_Sleep(WDTO_60MS);
}

void PWR_stop_ADC()
{
  ADCSRA &= ~_BV(ADEN); // ADC off
  power_adc_disable(); // Disable ADC in power register
}

////////////////////
void PWM_setup_Pump()
{
  TCCR0A = 2 << COM0A0 | 3 << COM0B0 | 3 << WGM00; // Timer 0 Control Register A -  Enable Fast PWM, OC0A Non Inverting Mode, OC0B Inverting Mode (OC0A)
  TCCR0B = 0 << WGM02 | 1 << CS00; // Timer 0 Control Register B - Enable Fast PWM, Clock Select Bit no prescaling
  OCR0A = 117; // set PWM duty // analogWrite(0, 117); // OC0A // PMP1_Pin
  OCR0B = 137; // set PWM duty // analogWrite(1, 137); // OC0B // PMP2_Pin
}

////////////////////
void PWM_setup_Driver()
{
  TCCR1 = 6 << CS10; // Timer 1 Control Register -  Set Prescaler (1<<CS10 ~ 4kHz, 2 ~ 2kHz, 3 ~ 1kHz, 4 ~ 500Hz, 5 ~ 250Hz, 6 ~ 125Hz, 7 ~ 63Hz)
  GTCCR = 1 << PWM1B | 1 << COM1B0; // General Control Register for Timer 1 - *PWM*, OC1B and ~OC1B connected. // DRV_Pin ~LED_Pin
}

////////////////////
void WDT_Sleep(byte WDTO_time)
{
  wdt_enable(WDTO_time); // Enable watchdog timer ~= 2sec
  wdt_int();
  sleep_mode(); // Make CPU sleep until next WDT interrupt
  wdt_disable();
}

////////////////////
// Watchdog Timer Interrupt Service is executed when watchdog timed out
ISR(WDT_vect) {
  wdt_int(); // Needed each time
}

//////////////////////
uint16_t getReading() {
  power_adc_enable();
  ADCSRA = _BV(ADEN) | _BV(ADIF); //enables de ADC and clear the INTERRUPT FLAG
  ADCSRA |= _BV(ADPS2); //Preescales of 16, for keep ADCclock between 50 and 200 KHz better Resolution
  ADMUX = _BV(MUX0); //"0001" //Port Analog1 PB2
  delay(8);
  noInterrupts();
  set_sleep_mode(SLEEP_MODE_ADC); //Sleep for taking and ADC measurement
  sleep_enable();

  ADCSRA |= _BV(ADIE); //Habilita las interrupciones por ADC
  interrupts(); //Habilita las interrupciones generales
  sleep_cpu(); //send to sleep and take ADC measurements

  //WakeUP
  sleep_disable();
  // Once Awake, reading should be done, but better make sure
  while (bit_is_set(ADCSRA, ADSC)); //wait until conversion is finished.

  PWR_stop_ADC();

  return ADC;
}
