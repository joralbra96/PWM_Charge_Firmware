#include "readVcc.h"
#include <SoftwareSerial.h>

#include <avr/power.h>
#include <avr/wdt.h> //Library for WDT
#include <avr/sleep.h>



#define PUMP_1 0
#define PUMP_2 1
#define DRV_PIN 4
#define LED_Pin  3 //LED
#define VBAT_PIN 2
#define AR_VOLTAGE_CONST 170500



#define wdt_int() WDTCR |= _BV(WDIE) // | _BV(WDCE) | _BV(WDE) // WDT goes to interrupt, not reset

//Battery and Panel Data.
const unsigned int CAPACITY = 40; //26AH
const unsigned int CYCLE_VOLTAGE = 1420; //14.4 - 30mV/°C adicional de 25°C (Se asume temperatura en EC de 30-35°C dentro de la caja)
const unsigned int FLOAT_VOLTAGE = 1330; //13.5 - 20mV/°C adicional de 25°C ( Se asume temperatura en EC de 30-35°C dentro de la caja)
const unsigned int PANEL_POWER = 11000; //110W * 100

//unsigned int MAX_CURRENT = min((CAPACITY*400/16), (PANEL_POWER/2020)); //2020 = 20.2 V max voltage from Panel 
//Calculating Manually to save memory on Attiny
//unsigned long T_ABS_MAX = 0.3*(CAPACITY/MAX_CURRENT)
unsigned long T_ABS_MAX = 63412844; //2.20 H * 3600 seg * 1000 * adjTimer(8) = 89856000 ms
unsigned long T_ABS = 0;

unsigned long T_BULK = 7200000; //15 min * 60 seg * 1000 * adjTimer(8)=7200000 ms
//default para T_BULK: tiempo que demora en alcanzar el estado de ABSORTION.

unsigned long T_MIN_VOLT = millis();
unsigned int minVoltage = 1350;

unsigned long T_REFERENCE = 0;
unsigned long T_ACTUAL = 0;



unsigned int vcc = 500;

unsigned int voltage = 1350;
unsigned int setPoint = FLOAT_VOLTAGE;
int stepSize = 0;
int pulseWidth = 0;

//SoftwareSerial mySerial (DRV_PIN, DRV_PIN);//Se activa la comunicación serial como tx en el puerto 1 del attiny

//when ADC completed, take an interrupt
EMPTY_INTERRUPT (ADC_vect);

//estados del algoritmo de carga
enum charge_state {
  BULK,
  BULK_WAITING_TO_ABSORTION, //ESTADO BULK en el que confirma por 1 minuto que haya llegado al Cycle Voltage
  ABSORTION,
  FLOAT
};

charge_state estado = BULK;


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
  vcc = 5000;
  unsigned long newReading = 0;
  for (uint8_t i = 0; i < 16; i++){  //toma 16 muestras para hacer una división para un numero 2^n
    newReading = newReading + getReading(); //estas son las lecturas en binario (0 - 1024)
  }
  newReading = newReading/16; //This is a bitwise operation to make a division by 16 (newReading/16)
  //covnersion a centivoltios.
  newReading = (newReading*vcc*100)/AR_VOLTAGE_CONST;
  voltage = (voltage*5 + newReading*5)/10; //Here is the voltage in centivoltios

  stepSize = setPoint - voltage; //can be negative
  if(stepSize > 0){
    stepSize = 2;
  }else if(stepSize < 0){
    stepSize = -2;
  }else{
    stepSize = 0;
  }
  pulseWidth += stepSize; //can decrease too!
  pulseWidth = constrain(pulseWidth, 0, 255);
  OCR1B = pulseWidth;

//  mySerial.print("voltage: ");
//  mySerial.print(voltage);
//  mySerial.print(" - pulse:");
//  mySerial.println(pulseWidth);
//  mySerial.flush();

  T_ACTUAL = millis(); //tiempo actual de ejecucion de programa

  switch(estado){
    case BULK:
      if(voltage < minVoltage){ //encuentra el minimo voltage (esto mientras se este descargando en la Noche
        minVoltage = voltage;
        T_MIN_VOLT = millis();
      }

      setPoint = CYCLE_VOLTAGE; //Setpoint to Cycle Voltage

      if(voltage > (CYCLE_VOLTAGE-15)){ //cuando empieze a cargar (en el día), debe de llegar al Cycle Voltage
        estado = BULK_WAITING_TO_ABSORTION;
        T_REFERENCE = millis();
      }
      break;

    case BULK_WAITING_TO_ABSORTION:
      setPoint = CYCLE_VOLTAGE;

      if(voltage > (CYCLE_VOLTAGE - 40)){
        if((T_ACTUAL - T_REFERENCE) >= 240000) { //1min*60seg*1000ms*adjTimer(8) = 480000
          estado = ABSORTION; //Ha estado en CYCLE_VOLTAGE por al menos 1 min, por lo que cambia a estado ABSORTION
          T_BULK = T_ACTUAL - T_MIN_VOLT; //This is the real Time (No need to adjTimer)
          T_REFERENCE = T_ACTUAL;
        }
      }else{
        estado = BULK; //Si en algun momento el voltaje baja, entonces aun no esta cargado hasta el estado de ABSORTION.
      }
      break;

    case ABSORTION:
      setPoint = CYCLE_VOLTAGE;
      T_ABS = 10*T_BULK; //Time in ABsortion Mode is 10 times the Time in T_BULK, it goes from 15min to a max T_ABS_MAX
      T_ABS = constrain(T_ABS, (unsigned long) 7200000, T_ABS_MAX);
      if((T_ACTUAL - T_REFERENCE) >= T_ABS){ //espera el tiempo necesario.
        estado = FLOAT;
      }
      break;

    case FLOAT:
      setPoint = FLOAT_VOLTAGE;

      if(voltage < 1250){ //llega la noche y lo envia a BULK para que este preparado para la carga completa en el día.
        estado = BULK;
      }
      break;

    default:
      estado = BULK;
      break;
  }


  set_sleep_mode(SLEEP_MODE_IDLE); // Configure attiny85 sleep mode
  sleep_enable();
  WDT_Sleep(WDTO_15MS);
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
  TCCR1A = 2 << COM1A0 | 3<< COM1B0 | 3 << WGM10; 
  TCCR1B = 0 << WGM12 | 1 << CS10; 
  //TCCR1 = 6 << CS10; // Timer 1 Control Register -  Set Prescaler (1<<CS10 ~ 4kHz, 2 ~ 2kHz, 3 ~ 1kHz, 4 ~ 500Hz, 5 ~ 250Hz, 6 ~ 125Hz, 7 ~ 63Hz)
  //GTCCR = 1 << PWM1B | 1 << COM1B0; // General Control Register for Timer 1 - *PWM*, OC1B and ~OC1B connected. // DRV_Pin ~LED_Pin
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
