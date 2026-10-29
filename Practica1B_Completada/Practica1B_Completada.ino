//------------------------------------------------------------------------------
// Práctica 1.B - balanceo de carga (GII-IoT, Internet de las Cosas)
//
// Punto de partida: Ejemplo 3 del guion, "Estimación de la carga computacional".
// Se conservan su estructura (top + 3 workers + loop() como hebra idle), sus
// vectores de medida y chBSemWaitTimeout(); se añade el control de carga del
// enunciado y el extra de carga aleatoria periódica con histéresis.  Requiere ChRt.
//------------------------------------------------------------------------------
#include <ChRt.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

//------------------------------------------------------------------------------
// fmt1() - porcentaje con 1 decimal.
// No usa "%.1f": el newlib-nano de Arduino llega sin coma flotante en printf.
//------------------------------------------------------------------------------
static void fmt1(float v, char *out, size_t n)
{
  if (!(v >= 0.0f)) v = 0.0f;          // también captura NaN
  long ip = (long)v;
  long fp = (long)((v - (float)ip) * 10.0f + 0.5f);
  if (fp >= 10) { fp = 0; ip += 1; }   // redondeo que desborda el decimal
  snprintf(out, n, "%ld.%ld", ip, fp);
}

//------------------------------------------------------------------------------
// Parámetros de configuración
//------------------------------------------------------------------------------
#define USE_DOUBLE          TRUE
#define CYCLE_MS            1000
#define NUM_THREADS         5
#define NUM_WORKERS         3

// Carga por cada 100 ms de periodo
#define LOAD_MIN_PER_100MS  5
#define LOAD_MAX_PER_100MS  20000

// Control de utilización
#define UTIL_MAX_PCT       85.0f
// Reserva = distancia entre el límite duro y el umbral de actuación.
// Medido en placa: con 6,0 hubo 1/149 ciclos a 85,6 %; con 7,0, 0/299 (máx. 82,9 %).
#define UTIL_RESERVE_PCT   7.0f                        // 85,0 % -> 78,0 %
#define UTIL_BAJAR_TH      (UTIL_MAX_PCT - UTIL_RESERVE_PCT)   // 78,0 %
#define UTIL_HYST_PCT      2.0f
#define UTIL_TARGET_PCT    (UTIL_BAJAR_TH - UTIL_HYST_PCT)          // 76,0 %
#define CTRL_KP             0.88f
#define CTRL_KI             0.5f
#define CTRL_KD             0.39f
#define LOAD_UP_MAX_FACTOR  1.15f   // máx. 15 % de subida por ciclo
#define LOAD_DOWN_GLOBAL    0.92f   // cota inferior del factor proporcional (máx. 8 % bajada/ciclo)
#define LOAD_DOWN_THREAD    0.85f   // máx. 15 % bajada en la hebra incumplidora de periodo
#define WARMUP_CYCLES       3
#define PERIOD_TOL_PCT      5
#define OVERLOAD_CYCLES     2
#define MISS_CYCLES         2

// Extra: variación aleatoria de la intensidad por activación (0 = desactivada).
// Con RANDOM_LOAD_PCT > 0, la carga aleatoria se activa y desactiva cada
// RANDOM_LOAD_INTERVAL_MS milisegundos (alterna entre activa e inactiva).
#define RANDOM_LOAD_PCT          10   // ±10 % de variación aleatoria
#define RANDOM_LOAD_INTERVAL_MS  30000  // alterna cada 30 s

#define DETAIL_EVERY        5

//------------------------------------------------------------------------------
// Estado de las hebras
//------------------------------------------------------------------------------
char thread_name[NUM_THREADS][15] = {  "top",
                                       "worker_1", "worker_2", "worker_3",
                                       "idle" };

volatile uint32_t threadPeriod_ms[NUM_THREADS] = { CYCLE_MS, 200, 100, 200, 0 };

// Intensidad actual, mínima y máxima (iteraciones por activación)
volatile int threadLoad[NUM_THREADS]    = { 0, 0, 0, 0, 0 };
int          threadLoadMin[NUM_THREADS] = { 0, 0, 0, 0, 0 };
int          threadLoadMax[NUM_THREADS] = { 0, 0, 0, 0, 0 };

// Medidas publicadas por las hebras worker y consumidas por top
volatile uint32_t threadEffectivePeriod_ms[NUM_THREADS] = { 0, 0, 0, 0, 0 };
volatile uint32_t threadCycle_ms[NUM_THREADS]           = { 0, 0, 0, 0, 0 };
volatile uint32_t threadPeriodMisses[NUM_THREADS]       = { 0, 0, 0, 0, 0 };

// Evita que el compilador elimine el bucle de carga computacional.
volatile float workloadSink = 0;

// Valores de control PID

volatile double previousTime = 0.0f;
volatile double lastError = 0.0f;
volatile double cumError = 0.0f;

//------------------------------------------------------------------------------
// Carga aleatoria periódica (extra)
// randomLoadActive se alterna cada RANDOM_LOAD_INTERVAL_MS ms mediante un
// virtual timer de ChibiOS disparado desde la ISR del sistema.
//------------------------------------------------------------------------------
#if RANDOM_LOAD_PCT > 0
static virtual_timer_t randomLoadTimer;
volatile bool randomLoadActive = false;   // accedida desde worker (normal) y timer (ISR)

static void randomLoadTimerCb(void *p)
{
  (void)p;
  // Alterna el estado de la carga aleatoria
  randomLoadActive = !randomLoadActive;
  // Re-arma el timer para el siguiente intervalo
  chVTSetI(&randomLoadTimer, TIME_MS2I(RANDOM_LOAD_INTERVAL_MS), randomLoadTimerCb, NULL);
}
#endif

//------------------------------------------------------------------------------
// Estructuras de medida y de informe
//------------------------------------------------------------------------------
typedef struct {
  thread_t * thd;           // Hebra monitorizada
  systime_t lastSampleTime_i;
  sysinterval_t lastPeriod_i;
  systime_t ticksTotal;     // Último valor leído de chThdGetTicksX()
  sysinterval_t ticksPerCycle;
  float loadPerCycle_per;
} threadLoad_t;

typedef struct {
  threadLoad_t threadLoad[NUM_THREADS];
  uint32_t idling_per;
} systemLoad_t;

systemLoad_t sysLoad;

typedef struct {
  uint32_t cycle;
  float idlePct;
  float utilPct;
  float workersPct;
  float overheadPct;
  int load[NUM_THREADS];
  int loadMin[NUM_THREADS];
  int loadMax[NUM_THREADS];
  uint32_t periodMs[NUM_THREADS];
  uint32_t effPeriodMs[NUM_THREADS];
  uint32_t cycleMs[NUM_THREADS];
  uint32_t misses[NUM_THREADS];
  uint32_t missDelta[NUM_THREADS];
  float cpuPct[NUM_THREADS];
  const char *action;
  bool changed;
  bool detail;
  bool randomActive;  // true cuando la carga aleatoria está activa
} report_t;

report_t reportData;

//------------------------------------------------------------------------------
// Semáforos
//------------------------------------------------------------------------------
BSEMAPHORE_DECL(top_sem, true);
BSEMAPHORE_DECL(print_sem, true);

//------------------------------------------------------------------------------
// Utilidades del controlador
//------------------------------------------------------------------------------
static inline int clampLoad(int value, int lo, int hi)
{
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

// Aplica un factor dentro de los límites; devuelve true si cambia.
static bool scaleWorkerLoad(int worker, float factor)
{
  int current = threadLoad[worker];
  int updated = (int)lroundf((float)current * factor);

  // Evita que el redondeo bloquee el control.
  if (factor > 1.0f && updated <= current) updated = current + 1;
  if (factor < 1.0f && updated >= current) updated = current - 1;

  updated = clampLoad(updated, threadLoadMin[worker], threadLoadMax[worker]);
  if (updated == current) return false;

  threadLoad[worker] = updated;
  return true;
}

static bool scaleAllWorkers(float factor)
{
  bool changed = false;

  for (int w = 1; w <= NUM_WORKERS; w++) {
    if (scaleWorkerLoad(w, factor)) changed = true;
  }
  return changed;
}

//------------------------------------------------------------------------------
// Hebra top: monitorización y control de carga
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waTop, 4096);

static THD_FUNCTION(top, arg)
{
  (void)arg;

  pinMode(LED_BUILTIN, OUTPUT);
  bool ledState = false;
  digitalWrite(LED_BUILTIN, LOW);

  systime_t period_i = TIME_MS2I(CYCLE_MS);
  systime_t lastTime_i = chVTGetSystemTimeX();
  systime_t lastSampleTime_i = lastTime_i;

  // Último valor conocido de los contadores de incumplimiento de periodo
  uint32_t lastMisses[NUM_THREADS];
  memset(lastMisses, 0, sizeof(lastMisses));

  // Incumplimientos consecutivos por hebra: si alcanza MISS_CYCLES activa reducción de carga.
  uint32_t missStreak[NUM_THREADS];
  memset(missStreak, 0, sizeof(missStreak));

  // Toma los ticks actuales como referencia para la primera ventana.
  for (int tid = 1; tid < NUM_THREADS; tid++) {
    sysLoad.threadLoad[tid].ticksTotal = chThdGetTicksX(sysLoad.threadLoad[tid].thd);
    sysLoad.threadLoad[tid].ticksPerCycle = 0;
    sysLoad.threadLoad[tid].loadPerCycle_per = 0.0f;
  }
  lastTime_i = chVTGetSystemTimeX();
  lastSampleTime_i = lastTime_i;

  // Ciclos consecutivos por encima del umbral de actuación (78 %)
  uint32_t overloadCount = 0;

  uint32_t cycle = 0;

  while (!chThdShouldTerminateX()) {

    //------------------------------------------------------------------
    // Espera hasta el siguiente instante de muestreo
    //------------------------------------------------------------------
    systime_t deadline_i = lastTime_i + period_i;
    systime_t now_i = chVTGetSystemTimeX();

    if (deadline_i > now_i) {
      chBSemWaitTimeout(&top_sem, (sysinterval_t)(deadline_i - now_i));
    }

    now_i = chVTGetSystemTimeX();

    // Ventana real transcurrida (en ticks). En SAMD, 1 tick = 1 ms.
    sysinterval_t elapsed_i = now_i - lastSampleTime_i;
    if (elapsed_i == 0) elapsed_i = 1;
    lastSampleTime_i = now_i;

    uint32_t idleTicks = 0;
    float workersPct = 0.0f;

    // Workers e idle; top ya está incluido en la utilización total.
    for (int tid = 1; tid < NUM_THREADS; tid++) {
      threadLoad_t * tl = &sysLoad.threadLoad[tid];

      systime_t ticks = chThdGetTicksX(tl->thd);
      uint32_t delta = (uint32_t)(ticks - tl->ticksTotal);

      tl->ticksTotal = ticks;
      tl->ticksPerCycle = delta;
      tl->loadPerCycle_per = 100.0f * (float)delta / (float)elapsed_i;

      if (tid == NUM_THREADS - 1) {
        idleTicks = delta;              // La última entrada es la hebra idle
      } else {
        workersPct += tl->loadPerCycle_per;
      }
    }

    // Utilización = 100% - tiempo ocioso respecto al tiempo real transcurrido.
    float idlePct = 100.0f * (float)idleTicks / (float)elapsed_i;
    if (idlePct < 0.0f)   idlePct = 0.0f;
    if (idlePct > 100.0f) idlePct = 100.0f;

    float utilPct = 100.0f - idlePct;
    float overheadPct = utilPct - workersPct;   // top + printer + interrupciones
    if (overheadPct < 0.0f) overheadPct = 0.0f;

    sysLoad.idling_per = (uint32_t)(idlePct + 0.5f);

    // Parpadeo de actividad de la hebra top
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);

    cycle++;

    uint32_t missDelta[NUM_THREADS];
    memset(missDelta, 0, sizeof(missDelta));

    for (int w = 1; w <= NUM_WORKERS; w++) {
      uint32_t total = threadPeriodMisses[w];
      missDelta[w] = total - lastMisses[w];
      lastMisses[w] = total;

      if (missDelta[w] > 0) {
        if (missStreak[w] < 1000000u) missStreak[w]++;
      } else {
        missStreak[w] = 0;
      }
    }

    if (utilPct > UTIL_BAJAR_TH) {
      if (overloadCount < 1000000u) overloadCount++;
    } else {
      overloadCount = 0;
    }

    //------------------------------------------------------------------
    // Control de carga (warm-up, histéresis y ajuste PID)
    //------------------------------------------------------------------
    const char *action = "MANTENER (banda de histeresis)";
    bool changed = false;
    unsigned long currentTime = millis();
    double elapsedTime = (double)(currentTime - previousTime);
    previousTime = currentTime;

    // Un miss sólo se atribuye a la carga si C supera la mitad del periodo.
    bool periodMissCausedByLoad = false;
    for (int w = 1; w <= NUM_WORKERS; w++) {
      if (missStreak[w] >= MISS_CYCLES &&
          threadCycle_ms[w] > threadPeriod_ms[w] / 2) {
        periodMissCausedByLoad = true;
        break;
      }
    }

    // Las reducciones de seguridad también actúan en warm-up; solo se evita subir.
    if (overloadCount >= OVERLOAD_CYCLES) {
      float errDown = utilPct - UTIL_TARGET_PCT;    // positivo cuando util > target
      cumError += errDown * elapsedTime;
      float rateError = (errDown - lastError) / elapsedTime;
      float totalError = CTRL_KP * errDown + CTRL_KI * cumError + CTRL_KD * rateError;

      lastError = totalError;

      float factorDown = 1.0f - (totalError) / 100.0f;
      if (factorDown < LOAD_DOWN_GLOBAL) factorDown = LOAD_DOWN_GLOBAL;  // cota mínima
      if (factorDown > 1.0f) factorDown = 1.0f;
      changed = scaleAllWorkers(factorDown);
      action = "BAJAR: utilizacion > umbral";

    } else if (periodMissCausedByLoad) {
      for (int w = 1; w <= NUM_WORKERS; w++) {
        if (missStreak[w] >= MISS_CYCLES &&
            threadCycle_ms[w] > threadPeriod_ms[w] / 2) {
          if (scaleWorkerLoad(w, LOAD_DOWN_THREAD)) changed = true;
        }
      }
      action = "BAJAR: periodo incumplido";

    } else if (cycle <= WARMUP_CYCLES) {
      action = "WARM-UP (carga minima)";

    } else if (utilPct < UTIL_TARGET_PCT) {
      // Control PID acotado: sube la carga cuando hay margen disponible.
      float error = UTIL_TARGET_PCT - utilPct;
      cumError += error * elapsedTime;
      float rateError = (error - lastError) / elapsedTime;

      float totalError = CTRL_KP * error + CTRL_KI * cumError + CTRL_KD * rateError;

      lastError = totalError;
            
      float factor = 1.0f + (totalError) / 100.0f;

      if (factor > LOAD_UP_MAX_FACTOR) factor = LOAD_UP_MAX_FACTOR;
      if (factor < 1.0f) factor = 1.0f;

      changed = scaleAllWorkers(factor);
      action = "SUBIR";
    }

    reportData.cycle = cycle;
    reportData.idlePct = idlePct;
    reportData.utilPct = utilPct;
    reportData.workersPct = workersPct;
    reportData.overheadPct = overheadPct;

    for (int w = 0; w < NUM_THREADS; w++) {
      reportData.load[w] = threadLoad[w];
      reportData.loadMin[w] = threadLoadMin[w];
      reportData.loadMax[w] = threadLoadMax[w];
      reportData.periodMs[w] = threadPeriod_ms[w];
      reportData.effPeriodMs[w] = threadEffectivePeriod_ms[w];
      reportData.cycleMs[w] = threadCycle_ms[w];
      reportData.misses[w] = threadPeriodMisses[w];
      reportData.missDelta[w] = missDelta[w];
      reportData.cpuPct[w] = sysLoad.threadLoad[w].loadPerCycle_per;
    }

    reportData.action = action;
    reportData.changed = changed;
    reportData.detail = ((cycle % DETAIL_EVERY) == 0);
#if RANDOM_LOAD_PCT > 0
    reportData.randomActive = randomLoadActive;
#else
    reportData.randomActive = false;
#endif

    chBSemSignal(&print_sem);

    lastTime_i = now_i;
  }
}

//------------------------------------------------------------------------------
// Hebra worker: ejecuta la carga computacional
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waWorker1, 2048);
static THD_WORKING_AREA(waWorker2, 2048);
static THD_WORKING_AREA(waWorker3, 2048);

static THD_FUNCTION(worker, arg)
{
  int worker_ID = (int)arg;

  sysinterval_t period_i = TIME_MS2I(threadPeriod_ms[worker_ID]);

  // Tolerancia para considerar respetado el periodo
  sysinterval_t periodTol_i = (period_i * PERIOD_TOL_PCT) / 100;
  if (periodTol_i < TIME_MS2I(3)) periodTol_i = TIME_MS2I(3);

  // La primera medida debe dar ~periodo nominal, no el tiempo desde el arranque.
  systime_t lastBeginTime_i = chVTGetSystemTimeX() - period_i;
  systime_t deadline_i = chVTGetSystemTimeX();

#if RANDOM_LOAD_PCT > 0
  uint32_t rndState = 0x9E3779B9u ^
                      ((uint32_t)worker_ID * 2654435761u) ^
                      (uint32_t)chVTGetSystemTimeX();
#endif

  while (!chThdShouldTerminateX()) {

    systime_t beginTime_i = chVTGetSystemTimeX();

    // Comprobación del periodo (inicio a inicio)
    sysinterval_t effectivePeriod_i = beginTime_i - lastBeginTime_i;
    threadEffectivePeriod_ms[worker_ID] = TIME_I2MS(effectivePeriod_i);

    if (effectivePeriod_i > (period_i + periodTol_i)) {
      threadPeriodMisses[worker_ID]++;
    }
    lastBeginTime_i = beginTime_i;

    // Carga computacional
    int niter = threadLoad[worker_ID];

#if RANDOM_LOAD_PCT > 0
    // Variación aleatoria de la intensidad (extra):
    // solo se aplica cuando randomLoadActive == true (alternado cada RANDOM_LOAD_INTERVAL_MS).
    if (randomLoadActive) {
      rndState = rndState * 1664525u + 1013904223u;
      int spread = (niter * RANDOM_LOAD_PCT) / 100;
      if (spread > 0) {
        niter += (int)(rndState % (uint32_t)(2 * spread + 1)) - spread;
      }
    }
#endif

    if (niter < 0) niter = 0;
    if (niter > threadLoadMax[worker_ID]) niter = threadLoadMax[worker_ID];

#if USE_DOUBLE
    double num = 10;
#else
    float num = 10;
#endif

    for (int iter = 0; iter < niter; iter++) {
#if USE_DOUBLE
      num = exp(num) / (1 + exp(num));
#else
      num = expf(num) / (1 + expf(num));
#endif
    }

    // Impide que el compilador elimine el bucle por no usar su resultado.
    workloadSink = (float)num;

    systime_t endTime_i = chVTGetSystemTimeX();
    threadCycle_ms[worker_ID] = TIME_I2MS(endTime_i - beginTime_i);

    deadline_i += period_i;

    if (deadline_i > endTime_i) {
      chThdSleepUntil(deadline_i);
    } else {
      // Plazo incumplido: salta al siguiente múltiplo del periodo.
      while (deadline_i <= endTime_i) {
        deadline_i += period_i;
        threadPeriodMisses[worker_ID]++;
      }
      chThdSleepUntil(deadline_i);
    }
  }
}

//------------------------------------------------------------------------------
// Hebra printer: imprime las medidas por el puerto serie
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waPrinter, 1024);

static THD_FUNCTION(printer, arg)
{
  (void)arg;

  while (!chThdShouldTerminateX()) {

    chBSemWait(&print_sem);

    // Copia coherente aunque top actualice los datos mientras se imprime.
    report_t r;
    chSysLock();
    r = reportData;
    chSysUnlock();

    // Toda la salida se formatea en un único búfer y se emite en una sola escritura:
    // cada SerialUSB.print() suelto puede esperar al frame USB de 1 ms (costaba 7,4 % de CPU).
    static char buf[640];
    static char fs[5][24];
    static char fd[24];

    fmt1(r.idlePct,       fs[0], sizeof(fs[0]));
    fmt1(r.utilPct,       fs[1], sizeof(fs[1]));
    fmt1(r.workersPct,    fs[2], sizeof(fs[2]));
    fmt1(r.overheadPct,   fs[3], sizeof(fs[3]));
    fmt1(UTIL_TARGET_PCT, fs[4], sizeof(fs[4]));

    size_t len = 0;
    int n = snprintf(buf, sizeof(buf),
                     "#%lu  idle=%s%%  util=%s%%  workers=%s%%"
                     "  top/ISR/print=%s%%  target=%s%%"
#if RANDOM_LOAD_PCT > 0
                     "  rand=%s"
#endif
                     "\r\n",
                     (unsigned long)r.cycle,
                     fs[0], fs[1], fs[2], fs[3], fs[4]
#if RANDOM_LOAD_PCT > 0
                     , r.randomActive ? "ON" : "off"
#endif
                     );
    if (n > 0) len = (n < (int)sizeof(buf)) ? (size_t)n : sizeof(buf) - 1;

    if (r.detail) {
      for (int w = 1; w <= NUM_WORKERS; w++) {
        if (len + 1 >= sizeof(buf)) break;
        fmt1(r.cpuPct[w], fd, sizeof(fd));

        size_t rem = sizeof(buf) - len;
        n = snprintf(buf + len, rem,
                     "   %s  L=%d (%d-%d)  CPU=%s%%  C=%lums  T=%lu/%lums"
                     "  M=%lu (+%lu)\r\n",
                     thread_name[w], r.load[w], r.loadMin[w], r.loadMax[w], fd,
                     (unsigned long)r.cycleMs[w],
                     (unsigned long)r.effPeriodMs[w],
                     (unsigned long)r.periodMs[w],
                     (unsigned long)r.misses[w],
                     (unsigned long)r.missDelta[w]);
        if (n < 0) break;
        if ((size_t)n >= rem) { len = sizeof(buf) - 1; break; }  // truncado
        len += (size_t)n;
      }
    }

    if (len + 1 < sizeof(buf)) {
      // "sin cambio" solo si el ajuste lo impidieron los límites o la histéresis.
      bool attemptedChange = (strncmp(r.action, "SUBIR", 5) == 0) ||
                             (strncmp(r.action, "BAJAR", 5) == 0);
      size_t rem = sizeof(buf) - len;
      n = snprintf(buf + len, rem, "   [CTRL] %s%s\r\n", r.action,
                   (attemptedChange && !r.changed) ? " (sin cambio)" : "");
      if (n > 0) len += ((size_t)n < rem) ? (size_t)n : rem - 1;
    }

    buf[len] = '\0';
    SerialUSB.print(buf);
  }
}

//------------------------------------------------------------------------------
// chSetup(): inicialización dentro del contexto del RTOS
//------------------------------------------------------------------------------
void chSetup()
{
  // Verificación de la configuración de ChibiOS
  if (CH_CFG_TIME_QUANTUM == 0) {
    SerialUSB.println("ERROR: CH_CFG_TIME_QUANTUM debe ser distinto de cero");
    SerialUSB.println("       para habilitar round-robin con desplazamiento.");
    while (true) {}
  }

  if (CH_CFG_NO_IDLE_THREAD == FALSE) {
    SerialUSB.println("ERROR: CH_CFG_NO_IDLE_THREAD debe estar a TRUE");
    SerialUSB.println("       para que loop() actue como hebra idle.");
    while (true) {}
  }

  if (CH_DBG_THREADS_PROFILING == FALSE) {
    SerialUSB.println("ERROR: CH_DBG_THREADS_PROFILING debe estar a TRUE");
    SerialUSB.println("       para poder usar chThdGetTicksX().");
    while (true) {}
  }

  // Inicialización del estado compartido
  memset(&sysLoad, 0, sizeof(sysLoad));
  memset(&reportData, 0, sizeof(reportData));

  // Rango de intensidad de cada worker, proporcional a su periodo
  for (int w = 1; w <= NUM_WORKERS; w++) {
    threadLoadMin[w] = (LOAD_MIN_PER_100MS * (int)threadPeriod_ms[w]) / 100;
    threadLoadMax[w] = (LOAD_MAX_PER_100MS * (int)threadPeriod_ms[w]) / 100;
    threadLoad[w]    = threadLoadMin[w];
  }

  // La hebra actual es la idle (loop), pues CH_CFG_NO_IDLE_THREAD = TRUE
  sysLoad.threadLoad[NUM_THREADS - 1].thd = chThdGetSelfX();

  chBSemReset(&top_sem, true);
  chBSemReset(&print_sem, true);

  // Informe de configuración
  SerialUSB.println();
  SerialUSB.print("CH_CFG_TIME_QUANTUM:      ");
  SerialUSB.println(CH_CFG_TIME_QUANTUM);
  SerialUSB.print("CH_CFG_NO_IDLE_THREAD:    ");
  SerialUSB.println(CH_CFG_NO_IDLE_THREAD ? "TRUE" : "FALSE");
  SerialUSB.print("CH_DBG_THREADS_PROFILING: ");
  SerialUSB.println(CH_DBG_THREADS_PROFILING ? "TRUE" : "FALSE");
  SerialUSB.print("CYCLE_MS=");
  SerialUSB.print(CYCLE_MS);
  SerialUSB.print("  UTIL_MAX=");
  SerialUSB.print(UTIL_MAX_PCT, 1);
  SerialUSB.print("%  UTIL_TARGET=");
  SerialUSB.print(UTIL_TARGET_PCT, 1);
  SerialUSB.print("%  BAJAR_TH=");
  SerialUSB.print(UTIL_BAJAR_TH, 1);
  SerialUSB.print("%  HIST=");
  SerialUSB.print(UTIL_HYST_PCT, 1);
  SerialUSB.println("%");
  SerialUSB.print("RANDOM_LOAD=");
  SerialUSB.print(RANDOM_LOAD_PCT);
#if RANDOM_LOAD_PCT > 0
  SerialUSB.print("%  RANDOM_INTERVAL=");
  SerialUSB.print(RANDOM_LOAD_INTERVAL_MS / 1000);
  SerialUSB.print("s  OVERLOAD_CYCLES=");
#else
  SerialUSB.print("%  OVERLOAD_CYCLES=");
#endif
  SerialUSB.print(OVERLOAD_CYCLES);
  SerialUSB.print("  MISS_CYCLES=");
  SerialUSB.println(MISS_CYCLES);
  SerialUSB.println("Hebra       periodo   L_min   L_max");
  for (int w = 1; w <= NUM_WORKERS; w++) {
    SerialUSB.print(thread_name[w]);
    SerialUSB.print("   ");
    SerialUSB.print(threadPeriod_ms[w]);
    SerialUSB.print(" ms   ");
    SerialUSB.print(threadLoadMin[w]);
    SerialUSB.print("   ");
    SerialUSB.println(threadLoadMax[w]);
  }
  SerialUSB.println("Empezando a carga minima (warm-up)...");
  SerialUSB.println();

#if RANDOM_LOAD_PCT > 0
  // Arranca el virtual timer que alterna la carga aleatoria cada RANDOM_LOAD_INTERVAL_MS.
  // La primera activación ocurrirá al cabo de ese intervalo; empieza inactiva.
  chSysLock();
  chVTSetI(&randomLoadTimer, TIME_MS2I(RANDOM_LOAD_INTERVAL_MS), randomLoadTimerCb, NULL);
  chSysUnlock();
#endif

  // Creación de las hebras (workers y printer antes que top), prioridades según Rate Monotonic.
  sysLoad.threadLoad[1].thd = chThdCreateStatic(waWorker1, sizeof(waWorker1),
                                                NORMALPRIO + 1, worker, (void *)1);

  sysLoad.threadLoad[2].thd = chThdCreateStatic(waWorker2, sizeof(waWorker2),
                                                NORMALPRIO + 2, worker, (void *)2);

  sysLoad.threadLoad[3].thd = chThdCreateStatic(waWorker3, sizeof(waWorker3),
                                                NORMALPRIO + 1, worker, (void *)3);

  chThdCreateStatic(waPrinter, sizeof(waPrinter), NORMALPRIO, printer, NULL);

  // top por encima de todo para que su muestreo no sea interrumpido.
  sysLoad.threadLoad[0].thd = chThdCreateStatic(waTop, sizeof(waTop),
                                                NORMALPRIO + 3, top, NULL);
}

//------------------------------------------------------------------------------
// setup() de Arduino
//------------------------------------------------------------------------------
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  SerialUSB.begin(115200);
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }

  int status = LOW;
  while (!SerialUSB) {
    status = status == LOW ? HIGH : LOW;
    digitalWrite(LED_BUILTIN, status);
    delay(250);
  }

  SerialUSB.println("Hit any key + ENTER to start ...");
  while (!SerialUSB.available()) { delay(10); }

  // chBegin() no retorna: loop() pasa a ejecutarse como hebra idle.
  chBegin(chSetup);

  while (true) {;}
}

//------------------------------------------------------------------------------
// loop(): hebra idle (CH_CFG_NO_IDLE_THREAD = TRUE)
//------------------------------------------------------------------------------
void loop()
{
}
