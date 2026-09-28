/* ----------------------------------------------------------------------------
 *  Ejemplo 9: Ilustra cómo es posible usar interrupciones internas para
 *             activar periódicamente el modo sleep con el consiguiente
 *             ahorro de energía. El RTC del microcontrolador lo despertará
 *             cada 2s.
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
#include "ArduinoLowPower.h"

const uint32_t halfPeriod_ms = 250;
const uint32_t sleepingTime_ms = 2000;
volatile uint16_t iterations = 0;

void flash_n_times(uint16_t repetitions)
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
 
  // Comentar esta función evitará que se invoque la función dummy() tras despertar
  // al micro y la variable repetitions no cambiará
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, dummy, CHANGE);
}

void loop() 
{
  // Hacemos parpadear el LED del MKR1310
  flash_n_times(iterations%10);
  
  // Pone a dormir el microcontrolador durante el tiempo consignado en la variable
  // sleepingTime_ms con el consiguiente ahorro de energía. Al expirar este tiempo,
  // el RTC emitirá una alarma que despertará al micro. La librería oculta muchos
  // detalles, como el registro de los tipos de eventos que pueden sacar 
  // al microcontrolador del modo sleep
  LowPower.sleep(sleepingTime_ms);
}

void dummy() 
{
  // Esta función se ejecuta una vez cuando el dispositivo se despierte
  // Como se ejecuta en un contexto de interrupción, el procesamiento debe 
  // ser muy rápido. En consecuencia, no deben invocarse funciones que 
  // consuman mucho tiempo (e.g delay())
  iterations++;
}
