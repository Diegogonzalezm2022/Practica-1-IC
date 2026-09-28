/* ----------------------------------------------------------------------------
 *  Ejemplo 10: Este ejemplo muestra cómo es posible combinar dos fuentes de
 *              interrupción para despertar al microcontrolador y sacarlo de
 *              un estado deep sleep.
 * 
 *  - Importante: El uso del SerialUSB es cuasi-incompatible con el modo sleep        
 *  - Requiere la librería ArduinoLowPower
 *      https://www.arduino.cc/reference/en/libraries/arduino-low-power/
 *
 *  NOTA: Mientras el microcontrolador esté en el modo sleep no será posible 
 *        cargar un nuevo firmware. En ese caso, se puede pulsar dos veces el 
 *        botón de reset.
 *
 *  Asignatura (GII-IoT)
 * -----------------------------------------------------------------------------
 */
#include <ArduinoLowPower.h>

// ArduinoLowPower.h incluye internamente RTCZero.h
RTCZero rtc;

const int externalPin = 5;
const uint32_t alarm_halfPeriod_ms = 100;
const uint32_t external_halfPeriod_ms = 500;

volatile uint16_t alarm_iterations = 0;
volatile uint16_t external_iterations = 0;
volatile uint32_t _period_sec = 0;

void flash_n_times(uint16_t repetitions, uint32_t halfPeriod_ms)
{
  if (!repetitions) repetitions = 1;
  for (uint16_t k = 0; k < repetitions; k++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(halfPeriod_ms);
    digitalWrite(LED_BUILTIN, LOW);
    delay(halfPeriod_ms);
  }
}

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);

  // Ajustamos el modo del externalPin
  // Lo configuramos en modo pullup para que se produzca una
  // interrupción por flanco de bajada (FALLING) al llevarl a tierra
  pinMode(externalPin, INPUT_PULLUP); 
  LowPower.attachInterruptWakeup(externalPin, externalCallback, FALLING);

  // Ponemos en hora el RTC
  rtc.begin();
  rtc.setTime(10, 0, 0);
  rtc.setDate(21, 9, 23);

  // IMPORTANTE: Mantener el orden de estas dos sentencias
  // LowPower.attachInterruptWakeup() reinicializa la configuración del
  // RTC 
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, alarmCallback, CHANGE);
  setPeriodicAlarm(10,5);

 
  // Pone a dormir el microcontrolador INDEFINIDAMENTE hasta que se genere 
  // alguna de las interrupciones
  LowPower.sleep();
}

void loop() 
{
  if (external_iterations) {
    // Si se recibe una interrupción externa, el LED parpadeará dos veces
    // de forma "lenta" (semiperiodo de 500 ms)
    external_iterations = 0;
    flash_n_times( 2, external_halfPeriod_ms );
  }
  
  if (alarm_iterations) {
    // Si expira el periodo programado en el RTC, el LED parpadeará dos veces
    // de forma "rápida" (semiperiodo de 100 ms)
    alarm_iterations = 0;
    flash_n_times( 2, alarm_halfPeriod_ms );
  }

  // Pone a dormir el microcontrolador hasta que se genere una interrupción
  LowPower.sleep();
}

// --------------------------------------------------------------------------------
// Callback cuando se genera una interrupción externa
// --------------------------------------------------------------------------------
void externalCallback()
{
  external_iterations++;
}

// --------------------------------------------------------------------------------
// Callback cuando se activa la alarma del RTC
// --------------------------------------------------------------------------------
void alarmCallback() 
{
  // Increment the counter
  alarm_iterations++;

  // Reprogramamos la alarma usando el mismo periodo 
  rtc.setAlarmEpoch(rtc.getEpoch() + _period_sec);
}

// --------------------------------------------------------------------------------
// Programa la alarma del RTC para que se active cada  period_sec segundos a 
// partir de "offsetFromNow_sec" en segundos desde el instante actual
// --------------------------------------------------------------------------------
void setPeriodicAlarm(uint32_t period_sec, uint32_t offsetFromNow_sec)
{
  _period_sec = period_sec;
  rtc.setAlarmEpoch(rtc.getEpoch() + offsetFromNow_sec);

  // Ver enum Alarm_Match en RTCZero.h
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
}
