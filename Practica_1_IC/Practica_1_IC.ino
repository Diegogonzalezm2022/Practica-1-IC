#include <time.h>
#include <RTCZero.h>

RTCZero rtc;


volatile uint32_t _period_sec = 0;
volatile uint16_t _rtcFlag = 0;

#define elapsedMilliseconds(since_ms)  (uint32_t)(millis() - since_ms)

void setup() {
  uint32_t t_start_ms = millis();

  SerialUSB.begin(9600);
  while(!SerialUSB) {;}

  SerialUSB.print("Starting at: ");
  SerialUSB.print(__DATE__);
  SerialUSB.print(" ");
  SerialUSB.println(__TIME__);

  rtc.begin();

  if (!setDateTime(__DATE__, __TIME__))
  {
    SerialUSB.println("setDateTime() failed!\nExiting ...");
    while (1) { ; }
  }

  _rtcFlag = 0;

  setPeriodicAlarm(10, 5);

  rtc.attachInterrupt(alarmCallback);
}

void loop() {
  // put your main code here, to run repeatedly:
  if ( _rtcFlag ) {

    char dateTime[32];

    getDateTime(dateTime);

    SerialUSB.println(dateTime);

    _rtcFlag--;
    if ( _rtcFlag) SerialUSB.println("WARNING: Unattended RTC alarm events!");
    
    digitalWrite(LED_BUILTIN, LOW);
  }
}

bool setDateTime(const char * date_str, const char * time_str)
{
  char month_str[4];
  char months[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug",
                        "Sep", "Oct", "Nov", "Dec"};
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

void getDateTime(char* charArray)
{
  const char *weekDay[7] = { "Sun", "Mon", "Tue", "Wed", "Thr", "Fri", "Sat" };
  
  // Obtenemos el tiempo Epoch, segundos desde el 1 de enero de 1970
  time_t epoch = rtc.getEpoch();

  // Convertimos a la forma habitual de fecha y hora
  struct tm stm;
  gmtime_r(&epoch, &stm);
  
  // Generamos e imprimimos la fecha y la hora
  snprintf(charArray, 32,"%s %4u/%02u/%02u %02u:%02u:%02u",
           weekDay[stm.tm_wday], 
           stm.tm_year + 1900, stm.tm_mon + 1, stm.tm_mday, 
           stm.tm_hour, stm.tm_min, stm.tm_sec);
}

void setPeriodicAlarm(uint32_t period_sec, uint32_t offsetFromNow_sec)
{
  _period_sec = period_sec;
  rtc.setAlarmEpoch(rtc.getEpoch() + offsetFromNow_sec);

  // Ver enum Alarm_Match en RTCZero.h
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
}

void alarmCallback()
{
  _rtcFlag++;
  
  digitalWrite(LED_BUILTIN, HIGH);
  
  rtc.setAlarmEpoch(rtc.getEpoch() + _period_sec);
}
