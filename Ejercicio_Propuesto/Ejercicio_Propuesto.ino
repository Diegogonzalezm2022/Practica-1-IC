/* ---------------------------------------------------------------------------------
 * Ejercicio Propuesto - Práctica 1A
 * 
 * Firmware para Arduino MKR 1310 que implementa los siguientes objetivos:
 *
 *   1. Ajustar el RTC a partir de la hora y fecha de generación del firmware
 *      (usando __DATE__ y __TIME__).
 *   2. Programar la alarma del RTC para simular la lectura de un sensor de forma
 *      periódica (cada 10 segundos). Cada vez que la alarma se active, se genera
 *      una cadena de texto con la fecha y la hora.
 *   3. Esta cadena de texto se guarda en un fichero creado en el chip de memoria
 *      externa FLASH de la tarjeta (W25Q16, sistema de archivos SPIFFS).
 *   4. Poner el microcontrolador en modo sleep indefinido. Se despertará cuando
 *      se active la alarma del RTC.
 *   5. (Extra) Permitir que el microcontrolador registre otra interrupción
 *      (flanco de bajada) a través de un pin digital (pin 5) configurado como
 *      entrada en modo pull-up. Si ocurre esta interrupción, se procede como
 *      en el objetivo 2 pero indicando que la línea se añade por interrupción
 *      externa.
 *
 * Librerías necesarias:
 *   - RTCZero          (https://www.arduino.cc/reference/en/libraries/rtczero/)
 *   - ArduinoLowPower  (https://www.arduino.cc/reference/en/libraries/arduino-low-power/)
 *   - Arduino_MKRMEM   (https://github.com/arduino-libraries/Arduino_MKRMEM)
 *     NOTA: Comentar la declaración de la variable flash en
 *           src/Arduino_WQ16DV.cpp (líneas 193-199) de la librería MKRMEM.
 *
 * Asignatura: Internet de las Cosas (GII-IoT)
 * ---------------------------------------------------------------------------------
 */

#include <time.h>
#include <ArduinoLowPower.h>   // Incluye internamente RTCZero.h
#include <Arduino_MKRMEM.h>

// ---------------------------------------------------------------------------------
// Declaración de la memoria FLASH usando SPI1 y FLASH_CS
// (porque hemos comentado la declaración original en la librería MKRMEM)
// ---------------------------------------------------------------------------------
Arduino_W25Q16DV flash(SPI1, FLASH_CS);

// ---------------------------------------------------------------------------------
// Objeto RTC (RTCZero está incluido vía ArduinoLowPower)
// ---------------------------------------------------------------------------------
RTCZero rtc;

// ---------------------------------------------------------------------------------
// Configuración de pines e interrupciones
// ---------------------------------------------------------------------------------
const int EXTERNAL_PIN = 5;              // Pin para interrupción externa
const int FINISH_PIN = 4;

// ---------------------------------------------------------------------------------
// Variables volátiles (accedidas desde ISR)
// ---------------------------------------------------------------------------------
volatile uint16_t alarmFlag    = 0;      // Flag: alarma RTC activada
volatile uint16_t externalFlag = 0;      // Flag: interrupción externa activada
volatile uint16_t finishFlag = 0;
volatile uint32_t _period_sec  = 0;      // Periodo de la alarma en segundos

// ---------------------------------------------------------------------------------
// Nombre del fichero en SPIFFS
// ---------------------------------------------------------------------------------
char filename[] = "sensor_log.txt";

// =================================================================================
//  SETUP
// =================================================================================
void setup()
{
  // --- Desactivar el módulo LoRA para poder usar la memoria FLASH ----------------
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, LOW);

  // --- Configurar el LED integrado -----------------------------------------------
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // --- Configurar el pin de interrupción externa (pull-up + flanco de bajada) ----
  pinMode(EXTERNAL_PIN, INPUT_PULLUP);
  pinMode(FINISH_PIN, INPUT_PULLUP);  

  SerialUSB.begin(9600);
  while(!SerialUSB) {;}

  // --- Inicializar memoria FLASH y montar SPIFFS ---------------------------------
  flash.begin();

  int res = filesystem.mount();
  if (res != SPIFFS_OK && res != SPIFFS_ERR_NOT_A_FS) {
    // Si falla el montaje, entramos en bucle infinito con el LED parpadeando
    errorBlink();
  }

  // Crear el fichero (si ya existe, lo truncamos para empezar limpio)
  File file = filesystem.open(filename, CREATE | TRUNCATE);
  if (!file) {
    errorBlink();
  }
  file.close();

  // --- Objetivo 1: Ajustar el RTC con __DATE__ y __TIME__ ------------------------
  rtc.begin();
  if (!setDateTime(__DATE__, __TIME__)) {
    errorBlink();
  }

  // --- Objetivo 5 (Extra): Registrar interrupción externa ------------------------
  // IMPORTANTE: Esta llamada debe hacerse ANTES de attachInterruptWakeup del RTC
  // porque LowPower.attachInterruptWakeup() reinicializa la configuración del RTC
  LowPower.attachInterruptWakeup(EXTERNAL_PIN, externalCallback, FALLING);

  LowPower.attachInterruptWakeup(FINISH_PIN, finishCallback, FALLING);

  // --- Objetivo 2: Programar alarma periódica del RTC cada 10 segundos -----------
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, alarmCallback, CHANGE);
  setPeriodicAlarm(10, 5);  // Periodo 10s, primera alarma a los 5s desde ahora

  // --- Escribir línea inicial en el fichero --------------------------------------
  writeToFlash("--- Inicio del registro de datos ---", false);
  char dateTimeStr[64];
  getDateTimeString(dateTimeStr, sizeof(dateTimeStr), "START");
  writeToFlash(dateTimeStr, true);


  // --- Objetivo 4: Poner el micro en modo sleep indefinido -----------------------
  LowPower.sleep();
}

// =================================================================================
//  LOOP
// =================================================================================
void loop()
{
  // --- Comprobar si se ha activado la alarma del RTC (Objetivo 2) ----------------
  if (alarmFlag) {
    alarmFlag = 0;

    // Generar cadena con fecha y hora
    char dateTimeStr[64];
    getDateTimeString(dateTimeStr, sizeof(dateTimeStr), "RTC_ALARM");

    // Objetivo 3: Guardar en fichero FLASH
    writeToFlash(dateTimeStr, true);

    // Parpadeo breve del LED como indicación visual
    blinkLED(2, 100);
  }

  // --- Comprobar si se ha producido una interrupción externa (Objetivo 5) --------
  if (externalFlag) {
    externalFlag = 0;

    // Generar cadena con fecha y hora, indicando interrupción externa
    char dateTimeStr[64];
    getDateTimeString(dateTimeStr, sizeof(dateTimeStr), "EXT_INT");

    // Guardar en fichero FLASH
    writeToFlash(dateTimeStr, true);

    // Parpadeo lento del LED como indicación visual diferenciada
    blinkLED(3, 300);
  }

  if (finishFlag) {
    finishFlag=0;
    USBDevice.detach();
    delay(100);
    USBDevice.attach();
    delay(1000);
    while(!SerialUSB) {;}

    File file = filesystem.open(filename,  READ_ONLY);
    if (!file) {
      SerialUSB.print("Opening file ");
      SerialUSB.print(filename);
      SerialUSB.print(" failed for reading. Aborting ...");
      on_exit_with_error_do();
    }    
    SerialUSB.print("Reading file contents:\n\t ");
    
    // Leemos el contenido del fichero hasta alcanzar la marca EOF
    while(!file.eof()) {
      char c;
      int const bytes_read = file.read(&c, sizeof(c));
      if (bytes_read) {
        SerialUSB.print(c);
        if (c == '\n') SerialUSB.print("\t ");
      }
    }

    // Cerramos el fichero
    file.close();
    SerialUSB.println("\nFile closed");

    // Desmontamos el sistema de archivos
    SerialUSB.println("Unmounting filesystem ... (program finished)");
    filesystem.unmount();
    exit(0);
  }

  // --- Volver a dormir al microcontrolador hasta la próxima interrupción ---------
  LowPower.sleep();

}

// =================================================================================
//  FUNCIONES AUXILIARES
// =================================================================================

// ---------------------------------------------------------------------------------
// Ajusta el RTC a partir de las cadenas __DATE__ y __TIME__
// Formato de __DATE__: "Sep 28 2026"
// Formato de __TIME__: "10:30:00"
// ---------------------------------------------------------------------------------
bool setDateTime(const char *date_str, const char *time_str)
{
  char month_str[4];
  char months[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  uint16_t i, mday, month, hour, min, sec, year;

  if (sscanf(date_str, "%3s %hu %hu", month_str, &mday, &year) != 3) return false;
  if (sscanf(time_str, "%hu:%hu:%hu", &hour, &min, &sec) != 3) return false;

  for (i = 0; i < 12; i++) {
    if (!strncmp(month_str, months[i], 3)) {
      month = i + 1;
      break;
    }
  }
  if (i == 12) return false;

  rtc.setTime((uint8_t)hour, (uint8_t)min, (uint8_t)sec);
  rtc.setDate((uint8_t)mday, (uint8_t)month, (uint8_t)(year - 2000));
  return true;
}

// ---------------------------------------------------------------------------------
// Programa la alarma periódica del RTC
// ---------------------------------------------------------------------------------
void setPeriodicAlarm(uint32_t period_sec, uint32_t offsetFromNow_sec)
{
  _period_sec = period_sec;
  rtc.setAlarmEpoch(rtc.getEpoch() + offsetFromNow_sec);
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
}

// ---------------------------------------------------------------------------------
// Genera una cadena con la fecha/hora actual y la fuente de la interrupción
// ---------------------------------------------------------------------------------
void getDateTimeString(char *buffer, size_t bufSize, const char *source)
{
  const char *weekDay[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

  // Lectura atómica del RTC mediante getEpoch()
  time_t epoch = rtc.getEpoch();
  struct tm stm;
  gmtime_r(&epoch, &stm);

  snprintf(buffer, bufSize, "[%s] %s %04u/%02u/%02u %02u:%02u:%02u",
           source,
           weekDay[stm.tm_wday],
           stm.tm_year + 1900, stm.tm_mon + 1, stm.tm_mday,
           stm.tm_hour, stm.tm_min, stm.tm_sec);
}

// ---------------------------------------------------------------------------------
// Escribe una línea de texto en el fichero SPIFFS de la memoria FLASH
// ---------------------------------------------------------------------------------
void writeToFlash(const char *text, bool appendNewline)
{
  File file = filesystem.open(filename, WRITE_ONLY | APPEND);
  if (!file) {
    return;  // Si falla, simplemente retornamos (no podemos usar Serial en sleep)
  }

  // Preparamos la línea a escribir
  char line[80];
  if (appendNewline) {
    snprintf(line, sizeof(line), "%s\n", text);
  } else {
    snprintf(line, sizeof(line), "%s\n", text);
  }

  int bytes_to_write = strlen(line);
  file.write((void *)line, bytes_to_write);

  file.close();
}

// ---------------------------------------------------------------------------------
// Hace parpadear el LED un número de veces con un semiperiodo dado
// ---------------------------------------------------------------------------------
void blinkLED(uint16_t times, uint32_t halfPeriod_ms)
{
  for (uint16_t k = 0; k < times; k++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(halfPeriod_ms);
    digitalWrite(LED_BUILTIN, LOW);
    delay(halfPeriod_ms);
  }
}

// ---------------------------------------------------------------------------------
// Parpadeo continuo de error (bucle infinito)
// ---------------------------------------------------------------------------------
void errorBlink()
{
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void on_exit_with_error_do()
{
  filesystem.unmount();
  exit(EXIT_FAILURE);
}

// =================================================================================
//  CALLBACKS (ISR) - Ejecución rápida, sin delay ni funciones lentas
// =================================================================================

// ---------------------------------------------------------------------------------
// Callback de la alarma del RTC
// ---------------------------------------------------------------------------------
void alarmCallback()
{
  alarmFlag++;
  // Reprogramar la alarma para el próximo periodo
  rtc.setAlarmEpoch(rtc.getEpoch() + _period_sec);
}

// ---------------------------------------------------------------------------------
// Callback de la interrupción externa
// ---------------------------------------------------------------------------------
void externalCallback()
{
  externalFlag++;
}

void finishCallback()
{
  finishFlag=1;
}