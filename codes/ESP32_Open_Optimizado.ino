#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>
#include <math.h>
#include <stdlib.h>

// ====================================================================
//                  VARIABLES PARA CALIBRAR EL ROBOT
// ====================================================================

// ----------------------- VELOCIDAD DEL MOTOR ------------------------
// Estos son los valores principales para ajustar el movimiento.
// Rango permitido: 0 (detenido) a 100 (velocidad maxima).
constexpr uint8_t VELOCIDAD_MOTOR_PORCENTAJE = 100;
constexpr uint8_t VELOCIDAD_MOTOR_EN_ESQUINA_PORCENTAJE = 70;
constexpr uint8_t VELOCIDAD_MINIMA_PARED_LATERAL_PORCENTAJE = 70;
constexpr uint8_t VELOCIDAD_RETROCESO_ESCAPE_PORCENTAJE = 45;

// Mientras el interruptor de potencia del motor esta abierto, el ESP32 sigue
// encendido. Se mantiene este PWM bajo preparado y la rampa no comienza hasta
// que el encoder confirma que el motor realmente se movio.
constexpr uint8_t VELOCIDAD_DETECCION_ARRANQUE_PORCENTAJE = 45;
constexpr uint8_t PULSOS_ENCODER_PARA_ARRANCAR = 4;
constexpr unsigned long VENTANA_DETECCION_MOVIMIENTO_MS = 600;
constexpr unsigned long DURACION_ARRANQUE_SUAVE_MS = 5000;

// Cambia entre true y false para invertir el sentido del motor.
constexpr bool INVERTIR_DIRECCION_MOTOR = true;

// Pines del canal A del TB6612FNG: PWMA, AIN2 y AIN1.
constexpr uint8_t MOTOR_PWM_PIN = D6;
constexpr uint8_t MOTOR_AIN2_PIN = D7;
constexpr uint8_t MOTOR_AIN1_PIN = D8;

// El esquema de la PCB conecta ENC_A3.3 a D5 y ENC_B3.3 a D9.
// Para detectar movimiento basta contar los flancos del canal A; el canal B
// queda configurado para poder agregar direccion/odometria mas adelante.
constexpr uint8_t ENCODER_A_PIN = D5;
constexpr uint8_t ENCODER_B_PIN = D9;

// ----------------------- CONTEO DE VUELTAS --------------------------
// El modulo GY-BNO085 usa 0x4B por defecto; ADDR en LOW selecciona 0x4A.
// El programa intenta ambas direcciones automaticamente sobre SDA=21/SCL=22.
constexpr uint8_t BNO085_I2C_ADDRESS_PRIMARY = 0x4B;
constexpr uint8_t BNO085_I2C_ADDRESS_SECONDARY = 0x4A;
constexpr uint32_t BNO085_REPORT_INTERVAL_US = 10000;
constexpr uint8_t VUELTAS_OBJETIVO = 3;
constexpr float GRADOS_POR_VUELTA = 360.0f;

// Seguro suave cada 90 grados. Poco antes del angulo reduce velocidad y
// limita el volante; al alcanzarlo centra brevemente el servo para evitar
// que el robot siga cerrando la curva por inercia.
constexpr bool SEGURO_CADA_90_HABILITADO = true;
constexpr float GRADOS_POR_ESQUINA = 90.0f;
constexpr float ANTICIPACION_SEGURO_90_GRADOS = 10.0f;
constexpr int CORRECCION_MAXIMA_PREVIA_90_GRADOS = 15;
constexpr uint8_t VELOCIDAD_SEGURO_90_PORCENTAJE = 60;
constexpr unsigned long DURACION_SERVO_CENTRADO_90_MS = 350;

// Al salir de cada esquina, devuelve gradualmente el mando al PID para evitar
// que una lectura lateral extrema produzca un volantazo inmediato.
constexpr unsigned long DURACION_REENTRADA_PID_90_MS = 300;

// Maximo cambio solicitado al servo entre dos paquetes de sensores. Conserva
// el angulo objetivo, pero evita saltos por ruido de los ultrasonicos.
constexpr int PASO_MAXIMO_SERVO_POR_LECTURA_GRADOS = 10;

// El ancho inicial se obtiene promediando S1 + S5 mientras el robot permanece
// quieto. Esa clasificacion selecciona el avance posterior a la tercera vuelta.
constexpr uint8_t MUESTRAS_PARA_CLASIFICAR_PASILLO = 10;
constexpr uint16_t UMBRAL_PASILLO_ANCHO_MM = 1000;
constexpr unsigned long DURACION_AVANCE_FINAL_PASILLO_ANCHO_MS = 250;
constexpr unsigned long DURACION_AVANCE_FINAL_PASILLO_ANGOSTO_MS = 2500;

// Al terminar, si S3 ve la pared a menos de 100 cm, retrocede hasta que la
// distancia supere 110 cm. El BNO085 mantiene el giro total en +/-1080 grados.
constexpr uint16_t DISTANCIA_INICIO_AJUSTE_FINAL_MM = 1000;
constexpr uint16_t DISTANCIA_SALIDA_AJUSTE_FINAL_MM = 1100;
constexpr uint8_t VELOCIDAD_AJUSTE_FINAL_PORCENTAJE = 35;
constexpr unsigned long DURACION_FRENO_AJUSTE_FINAL_MS = 150;
constexpr unsigned long DURACION_MAXIMA_AJUSTE_FINAL_MS = 5000;
constexpr float KP_RUMBO_AJUSTE_FINAL = 2.0f;
constexpr int DIRECCION_SERVO_AJUSTE_FINAL = -1;
constexpr int CORRECCION_MAXIMA_AJUSTE_FINAL_GRADOS = 20;

// Descarta saltos imposibles de rumbo producidos por un paquete corrupto o
// por reiniciar/desconectar el sensor.
constexpr float CAMBIO_RUMBO_MAXIMO_GRADOS = 90.0f;

// Si deja de llegar el rumbo durante la carrera, se detiene el robot.
constexpr unsigned long BNO085_FAILSAFE_TIMEOUT_MS = 500;

// Durante la puesta a punto, una falla del BNO085 solo desactiva el conteo de
// vueltas; no bloquea el control existente del motor ni del servo. Cuando el
// sensor ya este validado, puede cambiarse a true para exigirlo en competencia.
constexpr bool EXIGIR_BNO085_PARA_MOVERSE = false;

// -------------------------- DASHBOARD WIFI -------------------------
// true para pruebas y practicas. Cambia a false para una ronda: el radio WiFi
// queda apagado y el servidor no consume tiempo de procesamiento.
constexpr bool DASHBOARD_HABILITADO = true;
constexpr char DASHBOARD_WIFI_NOMBRE[] = "WRO-Open";
constexpr char DASHBOARD_WIFI_CLAVE[] = "wroopen26";
constexpr unsigned long DASHBOARD_UPDATE_INTERVAL_MS = 250;

// --------------------- DIRECCION Y SENSORES -------------------------

// S1 y S2 se consideran sensores del lado izquierdo.
// S3 se considera el sensor frontal.
// S4 y S5 se consideran sensores del lado derecho.
//
// Error usado:
//   (S1 * PESO_S1 + S2 * PESO_S2)
//   -
//   (S4 * PESO_S4 + S5 * PESO_S5)
//
// Ajusta los pesos segun la posicion de los sensores y el ancho del pasillo.
constexpr float PESO_S1 = 1.0f;
constexpr float PESO_S2 = 1.0f;
constexpr float PESO_S4 = 1.0f;
constexpr float PESO_S5 = 1.0f;

// ----------------------------- PID ----------------------------------
// KP: aumenta la reaccion inmediata para alejarse de la pared.
// KI: corrige una desviacion pequena que se mantiene durante varios segundos.
// KD: anticipa cambios rapidos; ayuda a reaccionar antes, pero demasiado
//     valor puede causar vibracion por el ruido de los ultrasonicos.
constexpr float KP = 0.04f;    ////////////////////////////////////////////////////////////////////////////////// KP
constexpr float KI = 0.001f;
constexpr float KD = 0.003f;

// Durante la espera de movimiento, Kp comienza en esta fraccion de su valor
// normal. Al iniciar la rampa del motor, sube linealmente hasta 100% durante
// DURACION_ARRANQUE_SUAVE_MS. Por ejemplo, 0.50 deja Kp a la mitad.
constexpr float MULTIPLICADOR_KP_INICIAL = 0.50f;

// Limita la contribucion integral para evitar que el control se acumule.
constexpr float LIMITE_INTEGRAL_GRADOS = 8.0f;

// 0 filtra completamente la derivada; 1 no aplica filtro.
constexpr float FILTRO_DERIVADA = 0.25f;

// Si el robot gira hacia el lado equivocado, cambia 1 por -1.
constexpr int8_t DIRECCION_SERVO = -1;

// Durante la REVERSA de escape, el efecto del volante sobre el rumbo se
// invierte respecto al avance. Por eso se usa una direccion independiente.
// 1 = servo hacia +angulo produce giro del robot en sentido positivo al
// retroceder; -1 = produce giro en sentido negativo.
constexpr int8_t DIRECCION_SERVO_REVERSA = 1;
constexpr int ANGULO_GUIA_RETROCESO_GRADOS = 18;
constexpr float UMBRAL_SENTIDO_RETROCESO_GRADOS = 12.0f;

// Configuracion mecanica del SG90.
constexpr uint8_t SERVO_PIN = 27;  // D4 / IO27 en la DFR0478
constexpr int SERVO_CENTRO_GRADOS = 90;
constexpr int SERVO_RECORRIDO_MAXIMO_GRADOS = 40;

// Limita las lecturas laterales muy grandes o sin eco. Esto evita que un
// valor 0xFFFF produzca un giro descontrolado.
constexpr uint16_t DISTANCIA_LATERAL_MAXIMA_MM = 1500;

// Reduccion preventiva por pared lateral. Solo se activa si un lado esta
// cerca Y existe suficiente diferencia con el lado opuesto. Por eso no
// reduce la velocidad cuando ambos lados estan cerca en un pasillo angosto.
// La transicion es gradual entre los umbrales de inicio y maximo.
constexpr uint16_t PARED_LATERAL_CERCA_INICIO_MM = 250;
constexpr uint16_t PARED_LATERAL_CERCA_MAXIMA_MM = 100;
constexpr uint16_t DIFERENCIA_LATERAL_INICIO_MM = 150;
constexpr uint16_t DIFERENCIA_LATERAL_MAXIMA_MM = 350;

// Comportamiento al acercarse a una pared o esquina de frente.
// Entre INICIO y CRITICA se aumenta Kp y se reduce la velocidad gradualmente.
// La cercania a una pared lateral usa el mismo multiplicador de Kp.
constexpr uint16_t DISTANCIA_FRENTE_INICIO_MM = 1000;
constexpr uint16_t DISTANCIA_FRENTE_CRITICA_MM = 300;

// Escape frontal: frena, centra el servo y retrocede hasta recuperar espacio.
constexpr uint16_t DISTANCIA_FRENTE_RETROCESO_MM = 300;
constexpr uint16_t DISTANCIA_FRENTE_SALIDA_RETROCESO_MM = 450;
constexpr unsigned long DURACION_FRENO_ANTES_RETROCESO_MS = 150;
constexpr unsigned long DURACION_MAXIMA_RETROCESO_MS = 2500;
constexpr float MULTIPLICADOR_KP_EN_ESQUINA = 1.5f;   ////////////////////////////////////////////////////////////////
constexpr uint8_t BRILLO_LED_EN_ESQUINA_PORCENTAJE = 20;

// Pulsos habituales para un SG90. Como el movimiento queda limitado a
// 55..135 grados, el servo trabaja lejos de sus topes electricos.
constexpr uint16_t SERVO_PULSO_MINIMO_US = 500;
constexpr uint16_t SERVO_PULSO_MAXIMO_US = 2500;

// ====================================================================
//                    FIN DE VARIABLES DE CALIBRACION
// ====================================================================

// ------------------------ Configuracion general ----------------------

constexpr uint8_t NANO_I2C_ADDRESS = 0x08;
constexpr uint8_t SENSOR_COUNT = 5;
constexpr uint8_t DISTANCE_PACKET_BYTES = SENSOR_COUNT * 2;
constexpr uint16_t INVALID_DISTANCE_MM = 0xFFFF;
constexpr uint16_t DISTANCIA_SIN_ECO_MM = 2000;

// Pines I2C de la DFRobot FireBeetle ESP32 DFR0478.
// En la placa estan rotulados como SDA/IO21 y SCL/IO22.
constexpr uint8_t ESP32_SDA_PIN = 21;
constexpr uint8_t ESP32_SCL_PIN = 22;
constexpr uint32_t I2C_FREQUENCY_HZ = 100000;

// El ESP32 consulta a 50 Hz para usar cada lectura nueva del Nano con poca
// latencia. La impresion se limita para que Serial no frene el control.
constexpr unsigned long SENSOR_POLL_INTERVAL_MS = 20;
constexpr unsigned long SERIAL_PRINT_INTERVAL_MS = 500;

// Limite de trabajo auxiliar por ciclo. No cambia el control ni sus
// calibraciones; evita que una rafaga serial monopolice loop().
constexpr uint8_t MAX_SERIAL_BYTES_PER_LOOP = 16;
constexpr uint16_t I2C_TRANSACTION_TIMEOUT_MS = 50;

constexpr uint32_t SERVO_PWM_FREQUENCY_HZ = 50;
constexpr uint8_t SERVO_PWM_RESOLUTION_BITS = 16;

// PWM inaudible para el motor DC. Diez bits dan un rango de 0 a 1023.
constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t MOTOR_PWM_RESOLUTION_BITS = 10;

// Si se pierde la comunicacion con los sensores, el motor se detiene.
constexpr unsigned long SENSOR_FAILSAFE_TIMEOUT_MS = 250;

#if ESP_ARDUINO_VERSION_MAJOR < 3
constexpr uint8_t SERVO_PWM_CHANNEL = 0;
constexpr uint8_t MOTOR_PWM_CHANNEL = 1;
#endif

static_assert(DIRECCION_SERVO == 1 || DIRECCION_SERVO == -1,
              "DIRECCION_SERVO debe ser 1 o -1");
static_assert(DISTANCIA_FRENTE_INICIO_MM > DISTANCIA_FRENTE_CRITICA_MM,
              "La distancia de inicio debe ser mayor que la critica");
static_assert(DISTANCIA_FRENTE_SALIDA_RETROCESO_MM >
                  DISTANCIA_FRENTE_RETROCESO_MM,
              "La salida del retroceso debe superar su distancia de entrada");
static_assert(DISTANCIA_FRENTE_RETROCESO_MM >=
                  DISTANCIA_FRENTE_CRITICA_MM,
              "El escape debe activarse antes o al llegar a distancia critica");
static_assert(DURACION_FRENO_ANTES_RETROCESO_MS > 0,
              "La duracion del freno debe ser mayor que cero");
static_assert(DURACION_MAXIMA_RETROCESO_MS > 0,
              "La duracion maxima de retroceso debe ser mayor que cero");
static_assert(VELOCIDAD_MOTOR_PORCENTAJE >=
                  VELOCIDAD_MOTOR_EN_ESQUINA_PORCENTAJE,
              "La velocidad normal debe ser mayor o igual que la minima");
static_assert(VELOCIDAD_MOTOR_PORCENTAJE >=
                  VELOCIDAD_MINIMA_PARED_LATERAL_PORCENTAJE,
              "La velocidad lateral minima no puede superar la normal");
static_assert(PARED_LATERAL_CERCA_INICIO_MM >
                  PARED_LATERAL_CERCA_MAXIMA_MM,
              "El rango de cercania lateral debe ser descendente");
static_assert(DIFERENCIA_LATERAL_MAXIMA_MM >
                  DIFERENCIA_LATERAL_INICIO_MM,
              "El rango de diferencia lateral debe ser ascendente");
static_assert(VELOCIDAD_DETECCION_ARRANQUE_PORCENTAJE > 0,
              "La velocidad de deteccion debe ser mayor que cero");
static_assert(VELOCIDAD_DETECCION_ARRANQUE_PORCENTAJE <=
                  VELOCIDAD_MOTOR_EN_ESQUINA_PORCENTAJE,
              "La velocidad de deteccion no debe superar la minima");
static_assert(VELOCIDAD_RETROCESO_ESCAPE_PORCENTAJE > 0 &&
                  VELOCIDAD_RETROCESO_ESCAPE_PORCENTAJE <= 100,
              "La velocidad de retroceso debe estar entre 1 y 100");
static_assert(PULSOS_ENCODER_PARA_ARRANCAR > 0,
              "Se necesita al menos un pulso para detectar movimiento");
static_assert(MUESTRAS_PARA_CLASIFICAR_PASILLO > 0,
              "Se necesita al menos una muestra para clasificar el pasillo");
static_assert(DURACION_AVANCE_FINAL_PASILLO_ANCHO_MS > 0 &&
                  DURACION_AVANCE_FINAL_PASILLO_ANGOSTO_MS > 0,
              "Los avances finales deben durar mas de cero");
static_assert(GRADOS_POR_ESQUINA > 0.0f &&
                  ANTICIPACION_SEGURO_90_GRADOS > 0.0f &&
                  ANTICIPACION_SEGURO_90_GRADOS < GRADOS_POR_ESQUINA,
              "La anticipacion del seguro de 90 grados no es valida");
static_assert(VELOCIDAD_SEGURO_90_PORCENTAJE > 0 &&
                  VELOCIDAD_SEGURO_90_PORCENTAJE <= 100,
              "La velocidad del seguro debe estar entre 1 y 100");
static_assert(DURACION_SERVO_CENTRADO_90_MS > 0 &&
                  DURACION_REENTRADA_PID_90_MS > 0,
              "Los tiempos de centrado de esquina deben ser mayores que cero");
static_assert(PASO_MAXIMO_SERVO_POR_LECTURA_GRADOS > 0 &&
                  PASO_MAXIMO_SERVO_POR_LECTURA_GRADOS <=
                      SERVO_RECORRIDO_MAXIMO_GRADOS,
              "El paso maximo del servo debe ser valido");
static_assert(VELOCIDAD_AJUSTE_FINAL_PORCENTAJE > 0 &&
                  VELOCIDAD_AJUSTE_FINAL_PORCENTAJE <= 100,
              "La velocidad de ajuste final debe estar entre 1 y 100");
static_assert(DISTANCIA_SALIDA_AJUSTE_FINAL_MM >
                  DISTANCIA_INICIO_AJUSTE_FINAL_MM,
              "La salida del ajuste final debe superar su inicio");
static_assert(DURACION_FRENO_AJUSTE_FINAL_MS > 0 &&
                  DURACION_MAXIMA_AJUSTE_FINAL_MS > 0,
              "Los tiempos del ajuste final deben ser mayores que cero");
static_assert(DIRECCION_SERVO_AJUSTE_FINAL == 1 ||
                  DIRECCION_SERVO_AJUSTE_FINAL == -1,
              "La direccion del servo de ajuste debe ser 1 o -1");
static_assert(KP >= 0.0f && KI >= 0.0f && KD >= 0.0f,
              "KP, KI y KD no pueden ser negativos");
static_assert(MULTIPLICADOR_KP_INICIAL > 0.0f &&
                  MULTIPLICADOR_KP_INICIAL <= 1.0f,
              "El multiplicador de Kp inicial debe estar entre 0 y 1");
static_assert(BRILLO_LED_EN_ESQUINA_PORCENTAJE <= 100,
              "El brillo LED de esquina no puede superar 100");
static_assert(FILTRO_DERIVADA >= 0.0f && FILTRO_DERIVADA <= 1.0f,
              "FILTRO_DERIVADA debe estar entre 0 y 1");

bool servoPwmReady = false;
bool motorPwmReady = false;
bool servoPwmValueWritten = false;
uint8_t velocidadObjetivoPorcentaje = 0;
uint8_t velocidadAplicadaPorcentaje = 0;
uint8_t brilloLedBasePorcentaje = 0;
bool ledsIndicandoEsquina = false;
bool retrocesoEscapeActivo = false;
bool retrocesoEscapeFrenando = false;
bool retrocesoEscapeLimiteAlcanzado = false;
unsigned long retrocesoEscapeStartMs = 0;
bool distanceSensorFailsafeActive = false;
int currentServoAngle = SERVO_CENTRO_GRADOS;
float latestCornerFactor = 0.0f;
float latestLateralWallFactor = 0.0f;
uint16_t latestDistancesMm[SENSOR_COUNT] = {
    INVALID_DISTANCE_MM,
    INVALID_DISTANCE_MM,
    INVALID_DISTANCE_MM,
    INVALID_DISTANCE_MM,
    INVALID_DISTANCE_MM
};
volatile uint32_t encoderPulseCount = 0;

Adafruit_BNO08x bno085(-1);
sh2_SensorValue_t bno085SensorValue;
bool bno085Initialized = false;
bool bno085HasHeading = false;
bool bno085FailsafeActive = false;
uint8_t bno085I2cAddressDetected = 0;
bool lapTrackingActive = false;
bool missionComplete = false;
bool startupCorridorClassified = false;
bool startupCorridorIsWide = false;
uint8_t startupCorridorSampleCount = 0;
uint32_t startupCorridorWidthAccumulatorMm = 0;
uint16_t startupCorridorWidthMm = 0;
unsigned long selectedFinalAdvanceDurationMs =
    DURACION_AVANCE_FINAL_PASILLO_ANGOSTO_MS;
bool quarterTurnGuardHolding = false;
uint8_t nextQuarterTurnGuard = 1;
unsigned long quarterTurnGuardStartMs = 0;
unsigned long quarterTurnGuardReleaseStartMs = 0;
bool finalAdvanceActive = false;
unsigned long finalAdvanceAccumulatedMs = 0;
unsigned long finalAdvanceLastUpdateMs = 0;
bool finalReverseActive = false;
bool finalReverseBraking = false;
unsigned long finalReverseStateStartMs = 0;
unsigned long finalReverseStartMs = 0;
float finalReverseTargetTurnDegrees = 0.0f;
bool roundTimerStarted = false;
unsigned long roundStartMs = 0;
unsigned long roundFinishMs = 0;
float currentYawDegrees = 0.0f;
float previousYawDegrees = 0.0f;
float accumulatedTurnDegrees = 0.0f;
uint8_t completedLaps = 0;
unsigned long lastBno085FrameMs = 0;

WebServer dashboardServer(80);

enum class MotorStartupState : uint8_t {
  WAITING_FOR_MOVEMENT,
  RAMPING,
  RUNNING
};

MotorStartupState motorStartupState =
    MotorStartupState::WAITING_FOR_MOVEMENT;
uint32_t encoderDetectionBaseline = 0;
unsigned long encoderDetectionWindowStartMs = 0;
unsigned long motorRampStartMs = 0;
bool encoderDetectionWindowActive = false;

float pidIntegralError = 0.0f;
float pidPreviousError = 0.0f;
float pidFilteredDerivative = 0.0f;
unsigned long pidPreviousUpdateUs = 0;
bool pidHasPreviousSample = false;

struct ControlState {
  float leftValue;
  float rightValue;
  float error;
  float startupKpFactor;
  float effectiveKp;
  float pTerm;
  float iTerm;
  float dTerm;
  float cornerFactor;
  float lateralWallFactor;
  int servoAngle;
  uint8_t speedPercent;
};

void resetPidControl() {
  pidIntegralError = 0.0f;
  pidPreviousError = 0.0f;
  pidFilteredDerivative = 0.0f;
  pidPreviousUpdateUs = 0;
  pidHasPreviousSample = false;
}

// --------------------------- Dashboard WiFi ------------------------

const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>WRO Open - Telemetria</title>
  <style>
    :root{color-scheme:dark;--bg:#071018;--card:#101d28;--line:#263846;
      --text:#eef6fa;--muted:#91a6b4;--cyan:#35d6ff;--green:#45e68a;
      --orange:#ffb454;--red:#ff6474}
    *{box-sizing:border-box} body{margin:0;background:radial-gradient(circle at
      top,#123046 0,var(--bg) 45%);color:var(--text);font:15px system-ui,
      sans-serif;min-height:100vh}.wrap{width:min(1040px,94vw);margin:auto;
      padding:24px 0 40px}header{display:flex;justify-content:space-between;
      gap:16px;align-items:center;margin-bottom:18px}h1{font-size:clamp(22px,5vw,
      36px);margin:0;letter-spacing:-.04em}.tag{color:var(--cyan);font-weight:800;
      letter-spacing:.12em;font-size:12px}.status{padding:9px 13px;border:1px solid
      var(--line);border-radius:999px;background:#0c1720;color:var(--muted);
      white-space:nowrap}.status.ok{color:var(--green);border-color:#246b49}
      .status.warn{color:var(--orange);border-color:#745124}.status.stop{
      color:var(--red);border-color:#7a303a}.grid{display:grid;
      grid-template-columns:repeat(4,1fr);gap:12px}.card{background:linear-gradient(
      145deg,#132430,#0d1821);border:1px solid var(--line);border-radius:16px;
      padding:16px;box-shadow:0 12px 35px #0004}.wide{grid-column:span 2}
      .label{color:var(--muted);font-size:12px;text-transform:uppercase;
      letter-spacing:.09em}.value{font-size:clamp(26px,5vw,42px);font-weight:800;
      margin-top:4px}.unit{font-size:14px;color:var(--muted);font-weight:500}
      .laps{color:var(--cyan)}.sensors{display:grid;grid-template-columns:
      repeat(5,1fr);gap:9px;margin-top:12px}.sensor{text-align:center;background:#09131b;
      border-radius:12px;padding:12px 5px}.sensor b{display:block;font-size:20px;
      margin-top:4px}.bar{height:5px;background:#24333d;border-radius:9px;
      overflow:hidden;margin-top:9px}.bar i{display:block;height:100%;width:0;
      background:var(--cyan);transition:width .2s}.details{display:grid;
      grid-template-columns:repeat(4,1fr);gap:8px;margin-top:12px}.detail{
      background:#09131b;border-radius:10px;padding:10px}.detail b{display:block;
      margin-top:4px;font-size:18px}footer{color:var(--muted);text-align:center;
      margin-top:18px;font-size:12px}@media(max-width:720px){.grid{
      grid-template-columns:repeat(2,1fr)}.wide{grid-column:span 2}.details{
      grid-template-columns:repeat(2,1fr)}header{align-items:flex-start;
      flex-direction:column}}@media(max-width:420px){.sensors{gap:4px}.sensor{
      padding:10px 2px}.sensor b{font-size:16px}}
  </style>
</head>
<body><main class="wrap">
  <header><div><div class="tag">WRO INDIA 2026</div><h1>Robot Open</h1></div>
    <div id="status" class="status">Conectando...</div></header>
  <section class="grid">
    <article class="card"><div class="label">Vueltas</div>
      <div class="value laps"><span id="laps">--</span><span class="unit"> / <span id="goal">3</span></span></div></article>
    <article class="card"><div class="label">Rumbo</div>
      <div class="value"><span id="yaw">--</span><span class="unit">&deg;</span></div></article>
    <article class="card"><div class="label">Giro acumulado</div>
      <div class="value"><span id="turn">--</span><span class="unit">&deg;</span></div></article>
    <article class="card"><div class="label">Tiempo de ronda</div>
      <div class="value"><span id="roundTime">0:00.0</span></div></article>
    <article class="card wide"><div class="label">Distancias</div>
      <div class="sensors" id="sensors"></div></article>
    <article class="card wide"><div class="label">Control</div>
      <div class="details">
        <div class="detail">Motor<b><span id="speed">--</span>/<span id="target">--</span>%</b></div>
        <div class="detail">Servo<b><span id="servo">--</span>&deg;</b></div>
        <div class="detail">Error PID<b id="error">--</b></div>
        <div class="detail">Proteccion lateral<b><span id="lateralRisk">--</span>%</b></div>
        <div class="detail">Encoder<b id="encoder">--</b></div>
      </div></article>
  </section>
  <footer>Actualizacion local cada 250 ms &middot; 192.168.4.1</footer>
</main>
<script>
  const sensors=document.getElementById('sensors');
  sensors.innerHTML=[1,2,3,4,5].map(n=>`<div class="sensor"><span class="label">S${n}</span><b id="s${n}">--</b><div class="bar"><i id="b${n}"></i></div></div>`).join('');
  const put=(id,v)=>document.getElementById(id).textContent=v;
  async function refresh(){
    try{
      const r=await fetch('/api/status',{cache:'no-store'}),d=await r.json();
      put('laps',d.laps);put('goal',d.goal);put('yaw',d.yaw.toFixed(1));
      put('turn',d.turn.toFixed(1));put('speed',d.speed);put('target',d.target);
      const totalSeconds=d.elapsedMs/1000,minutes=Math.floor(totalSeconds/60);
      put('roundTime',minutes+':'+String(Math.floor(totalSeconds%60)).padStart(2,'0')+'.'+Math.floor((d.elapsedMs%1000)/100));
      put('servo',d.servo);put('error',d.error.toFixed(1));put('encoder',d.encoder);
      put('lateralRisk',Math.round(d.lateralRisk*100));
      d.distances.forEach((v,i)=>{put(`s${i+1}`,v<0?'--':(v/10).toFixed(1)+' cm');
        document.getElementById(`b${i+1}`).style.width=(v<0?0:Math.min(100,v/15))+'%'});
      const st=document.getElementById('status');st.textContent=d.state;
      st.className='status '+d.level;
    }catch(e){const st=document.getElementById('status');st.textContent='Sin conexion';
      st.className='status stop'}
  }
  refresh();setInterval(refresh,250);
</script></body></html>
)rawliteral";

const char *dashboardStateText() {
  if (missionComplete) return "3 vueltas completadas";
  if (finalReverseActive) return "3 vueltas: ajuste final";
  if (finalAdvanceActive) return "3 vueltas: avance final";
  if (!startupCorridorClassified) return "Midiendo ancho inicial";
  if (quarterTurnGuardHolding) return "Seguro de 90 grados";
  if (!bno085Initialized) return "BNO085 I2C no detectado";
  if (!bno085HasHeading) return "BNO085 esperando orientacion";
  if (bno085FailsafeActive) return "Sin datos BNO085";
  if (distanceSensorFailsafeActive) return "Sin sensores de distancia";
  if (retrocesoEscapeActivo) return "Escape en retroceso";
  if (latestLateralWallFactor > 0.0f) return "Reduciendo por pared lateral";
  if (motorStartupState == MotorStartupState::WAITING_FOR_MOVEMENT) {
    return "Esperando movimiento";
  }
  if (motorStartupState == MotorStartupState::RAMPING) return "Rampa suave";
  return "En marcha";
}

const char *dashboardStateLevel() {
  if (missionComplete || distanceSensorFailsafeActive ||
      (EXIGIR_BNO085_PARA_MOVERSE && bno085FailsafeActive)) return "stop";
  if (finalAdvanceActive || finalReverseActive || quarterTurnGuardHolding ||
      !bno085HasHeading ||
      retrocesoEscapeActivo ||
      latestLateralWallFactor > 0.0f ||
      bno085FailsafeActive ||
      motorStartupState == MotorStartupState::WAITING_FOR_MOVEMENT) {
    return "warn";
  }
  return "ok";
}

int dashboardDistance(uint8_t index) {
  return latestDistancesMm[index] == INVALID_DISTANCE_MM
             ? -1
             : static_cast<int>(latestDistancesMm[index]);
}

void sendDashboardStatus() {
  char json[700];
  snprintf(
      json,
      sizeof(json),
      "{\"yaw\":%.2f,\"turn\":%.2f,\"laps\":%u,\"goal\":%u,"
      "\"elapsedMs\":%lu,\"speed\":%u,\"target\":%u,"
      "\"servo\":%d,\"error\":%.2f,\"lateralRisk\":%.3f,"
      "\"encoder\":%lu,\"distances\":[%d,%d,%d,%d,%d],"
      "\"bnoI2c\":%u,"
      "\"state\":\"%s\",\"level\":\"%s\"}",
      currentYawDegrees,
      accumulatedTurnDegrees,
      completedLaps,
      VUELTAS_OBJETIVO,
      roundElapsedMs(),
      velocidadAplicadaPorcentaje,
      velocidadObjetivoPorcentaje,
      currentServoAngle,
      pidPreviousError,
      latestLateralWallFactor,
      static_cast<unsigned long>(readEncoderPulseCount()),
      dashboardDistance(0),
      dashboardDistance(1),
      dashboardDistance(2),
      dashboardDistance(3),
      dashboardDistance(4),
      bno085I2cAddressDetected,
      dashboardStateText(),
      dashboardStateLevel()
  );

  dashboardServer.send(200, "application/json", json);
}

void beginDashboard() {
  if (!DASHBOARD_HABILITADO) {
    WiFi.mode(WIFI_OFF);
    Serial.println("Dashboard WiFi deshabilitado para ronda.");
    return;
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(DASHBOARD_WIFI_NOMBRE, DASHBOARD_WIFI_CLAVE);
  dashboardServer.on("/", []() {
    dashboardServer.send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML);
  });
  dashboardServer.on("/api/status", sendDashboardStatus);
  dashboardServer.onNotFound([]() {
    dashboardServer.send(404, "text/plain", "No encontrado");
  });
  dashboardServer.begin();

  Serial.print("Dashboard: conecta a ");
  Serial.print(DASHBOARD_WIFI_NOMBRE);
  Serial.print(" y abre http://");
  Serial.println(WiFi.softAPIP());
}

void updateDashboard() {
  if (DASHBOARD_HABILITADO) {
    dashboardServer.handleClient();
  }
}

// ----------------------- Sensor BNO085 / vueltas -------------------

float normalizeHeadingDelta(float deltaDegrees) {
  while (deltaDegrees > 180.0f) {
    deltaDegrees -= 360.0f;
  }

  while (deltaDegrees < -180.0f) {
    deltaDegrees += 360.0f;
  }

  return deltaDegrees;
}

void finishThreeLapMission() {
  if (missionComplete) {
    return;
  }

  finalAdvanceActive = false;
  finalReverseActive = false;
  finalReverseBraking = false;
  finalReverseStateStartMs = 0;
  finalReverseStartMs = 0;
  finalReverseTargetTurnDegrees = 0.0f;
  missionComplete = true;
  completedLaps = VUELTAS_OBJETIVO;
  if (roundTimerStarted) {
    roundFinishMs = millis();
  }
  stopMotor();
  writeServoAngle(SERVO_CENTRO_GRADOS);

  Serial.println();
  Serial.println(
      "*** 3 VUELTAS Y AJUSTE FINAL COMPLETADOS: ROBOT DETENIDO ***"
  );
}

void startFinalAdvance(unsigned long nowMs) {
  if (missionComplete || finalAdvanceActive || finalReverseActive) {
    return;
  }

  completedLaps = VUELTAS_OBJETIVO;
  finalAdvanceActive = true;
  finalAdvanceAccumulatedMs = 0;
  finalAdvanceLastUpdateMs = nowMs;

  Serial.println();
  Serial.print("*** 3 VUELTAS: AVANCE FINAL DURANTE ");
  Serial.print(selectedFinalAdvanceDurationMs);
  Serial.print(" ms (pasillo ");
  Serial.print(startupCorridorIsWide ? "ancho" : "angosto");
  Serial.println(", tiempo efectivo) ***");
}

void startFinalReverseIfNeeded(unsigned long nowMs) {
  const uint16_t frontDistanceMm = latestDistancesMm[2];
  const bool needsReverse =
      frontDistanceMm != INVALID_DISTANCE_MM &&
      frontDistanceMm < DISTANCIA_INICIO_AJUSTE_FINAL_MM;

  finalAdvanceActive = false;

  if (!needsReverse) {
    finishThreeLapMission();
    return;
  }

  stopMotor();
  writeServoAngle(SERVO_CENTRO_GRADOS);
  finalReverseActive = true;
  finalReverseBraking = true;
  finalReverseStateStartMs = nowMs;
  finalReverseStartMs = nowMs;
  finalReverseTargetTurnDegrees =
      accumulatedTurnDegrees < 0.0f
          ? -(GRADOS_POR_VUELTA * VUELTAS_OBJETIVO)
          : GRADOS_POR_VUELTA * VUELTAS_OBJETIVO;

  Serial.print("Ajuste final: pared frontal a ");
  Serial.print(frontDistanceMm / 10);
  Serial.print(" cm; retrocedera hasta ");
  Serial.print(DISTANCIA_SALIDA_AJUSTE_FINAL_MM / 10);
  Serial.println(" cm.");
}

void updateFinalReverse(unsigned long nowMs) {
  if (!finalReverseActive || missionComplete) {
    return;
  }

  const uint16_t frontDistanceMm = latestDistancesMm[2];
  const bool reachedTargetDistance =
      frontDistanceMm != INVALID_DISTANCE_MM &&
      frontDistanceMm >= DISTANCIA_SALIDA_AJUSTE_FINAL_MM;

  if (reachedTargetDistance) {
    Serial.print("Ajuste final terminado: S3 alcanzo ");
    Serial.print(frontDistanceMm / 10);
    Serial.println(" cm.");
    finishThreeLapMission();
    return;
  }

  if (nowMs - finalReverseStartMs >= DURACION_MAXIMA_AJUSTE_FINAL_MS) {
    Serial.println(
        "Ajuste final detenido por alcanzar el limite de seguridad."
    );
    finishThreeLapMission();
    return;
  }

  const float headingErrorDegrees =
      accumulatedTurnDegrees - finalReverseTargetTurnDegrees;
  const float steeringCorrectionDegrees = constrain(
      static_cast<float>(DIRECCION_SERVO_AJUSTE_FINAL) *
          KP_RUMBO_AJUSTE_FINAL * headingErrorDegrees,
      -static_cast<float>(CORRECCION_MAXIMA_AJUSTE_FINAL_GRADOS),
      static_cast<float>(CORRECCION_MAXIMA_AJUSTE_FINAL_GRADOS)
  );
  const int finalReverseServoAngle = static_cast<int>(roundf(
      SERVO_CENTRO_GRADOS + steeringCorrectionDegrees
  ));

  // A +/-1080 grados el error es cero y el servo queda exactamente en 90.
  writeServoAngle(finalReverseServoAngle);

  if (finalReverseBraking) {
    if (nowMs - finalReverseStateStartMs >=
        DURACION_FRENO_AJUSTE_FINAL_MS) {
      finalReverseBraking = false;
      finalReverseStateStartMs = nowMs;
      velocidadObjetivoPorcentaje =
          VELOCIDAD_AJUSTE_FINAL_PORCENTAJE;
      setMotorDirectionReverse();
      applyMotorPwmPercent(VELOCIDAD_AJUSTE_FINAL_PORCENTAJE);

      Serial.print("Ajuste final: reversa al ");
      Serial.print(VELOCIDAD_AJUSTE_FINAL_PORCENTAJE);
      Serial.print("%, rumbo objetivo ");
      Serial.print(finalReverseTargetTurnDegrees, 1);
      Serial.println(" grados.");
    }
    return;
  }

  velocidadObjetivoPorcentaje = VELOCIDAD_AJUSTE_FINAL_PORCENTAJE;
  setMotorDirectionReverse();
  applyMotorPwmPercent(VELOCIDAD_AJUSTE_FINAL_PORCENTAJE);

}

void updateFinalAdvance(unsigned long nowMs) {
  if (!finalAdvanceActive || missionComplete) {
    return;
  }

  const unsigned long elapsedMs = nowMs - finalAdvanceLastUpdateMs;
  finalAdvanceLastUpdateMs = nowMs;

  // Conserva PID y protecciones. Solo cuenta tiempo con orden de avance y
  // PWM aplicado; una frenada, un retroceso o un failsafe pausa el avance.
  const bool movingForward =
      !retrocesoEscapeActivo &&
      velocidadObjetivoPorcentaje > 0 &&
      velocidadAplicadaPorcentaje > 0;

  if (movingForward) {
    finalAdvanceAccumulatedMs += elapsedMs;
  }

  if (finalAdvanceAccumulatedMs >= selectedFinalAdvanceDurationMs) {
    startFinalReverseIfNeeded(nowMs);
  }
}

unsigned long roundElapsedMs() {
  if (!roundTimerStarted) {
    return 0;
  }

  const unsigned long endMs =
      missionComplete ? roundFinishMs : millis();
  return endMs - roundStartMs;
}

void processBno085Heading(float yawDegrees, unsigned long nowMs) {
  currentYawDegrees = yawDegrees;
  lastBno085FrameMs = nowMs;
  bno085FailsafeActive = false;

  if (!bno085HasHeading) {
    bno085HasHeading = true;
    previousYawDegrees = yawDegrees;
    Serial.println("BNO085 listo: rumbo recibido por I2C.");
    return;
  }

  const float headingDelta =
      normalizeHeadingDelta(yawDegrees - previousYawDegrees);
  previousYawDegrees = yawDegrees;

  if (missionComplete || finalAdvanceActive) {
    return;
  }

  if (finalReverseActive) {
    if (fabsf(headingDelta) <= CAMBIO_RUMBO_MAXIMO_GRADOS) {
      accumulatedTurnDegrees += headingDelta;
    }
    return;
  }

  // La referencia de la primera vuelta se fija cuando el encoder confirma
  // movimiento. Asi, girar el robot a mano antes de arrancar no suma vueltas.
  if (!lapTrackingActive &&
      motorStartupState != MotorStartupState::WAITING_FOR_MOVEMENT) {
    lapTrackingActive = true;
    accumulatedTurnDegrees = 0.0f;
    completedLaps = 0;
    quarterTurnGuardHolding = false;
    nextQuarterTurnGuard = 1;
    quarterTurnGuardStartMs = 0;
    quarterTurnGuardReleaseStartMs = 0;
    Serial.println("Conteo de 3 vueltas iniciado.");
    return;
  }

  if (!lapTrackingActive) {
    return;
  }

  if (fabsf(headingDelta) > CAMBIO_RUMBO_MAXIMO_GRADOS) {
    Serial.println("BNO085: salto de rumbo descartado.");
    return;
  }

  accumulatedTurnDegrees += headingDelta;

  const uint8_t newCompletedLaps = static_cast<uint8_t>(
      floorf(fabsf(accumulatedTurnDegrees) / GRADOS_POR_VUELTA)
  );

  if (newCompletedLaps > completedLaps) {
    completedLaps = min(newCompletedLaps, VUELTAS_OBJETIVO);
    Serial.print("Vuelta completada: ");
    Serial.print(completedLaps);
    Serial.print("/");
    Serial.println(VUELTAS_OBJETIVO);
  }

  if (fabsf(accumulatedTurnDegrees) >=
      GRADOS_POR_VUELTA * VUELTAS_OBJETIVO) {
    startFinalAdvance(nowMs);
  }
}

void updateBno085(unsigned long nowMs) {
  if (!bno085Initialized) {
    return;
  }

  if (bno085.wasReset()) {
    bno085HasHeading = false;
    if (!bno085.enableReport(
            SH2_GAME_ROTATION_VECTOR,
            BNO085_REPORT_INTERVAL_US
        )) {
      bno085Initialized = false;
      Serial.println("ERROR: no se pudo reactivar el reporte del BNO085.");
      return;
    }

    Serial.println("BNO085 reiniciado: reporte de orientacion restaurado.");
  }

  while (bno085.getSensorEvent(&bno085SensorValue)) {
    if (bno085SensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
      continue;
    }

    const float qr = bno085SensorValue.un.gameRotationVector.real;
    const float qi = bno085SensorValue.un.gameRotationVector.i;
    const float qj = bno085SensorValue.un.gameRotationVector.j;
    const float qk = bno085SensorValue.un.gameRotationVector.k;
    const float sqr = qr * qr;
    const float sqi = qi * qi;
    const float sqj = qj * qj;
    const float sqk = qk * qk;
    const float yawDegrees = atan2f(
        2.0f * (qi * qj + qk * qr),
        sqi - sqj - sqk + sqr
    ) * RAD_TO_DEG;

    processBno085Heading(yawDegrees, nowMs);
  }
}

bool tryBeginBno085AtAddress(uint8_t address) {
  if (!bno085.begin_I2C(address, &Wire)) {
    return false;
  }

  if (!bno085.enableReport(
          SH2_GAME_ROTATION_VECTOR,
          BNO085_REPORT_INTERVAL_US
      )) {
    Serial.print("BNO085 encontrado en 0x");
    Serial.print(address, HEX);
    Serial.println(", pero no acepto el reporte de orientacion.");
    return false;
  }

  bno085I2cAddressDetected = address;
  bno085Initialized = true;
  lastBno085FrameMs = millis();

  Serial.print("BNO085 I2C listo en direccion 0x");
  Serial.println(address, HEX);
  return true;
}

void beginBno085I2c() {
  if (tryBeginBno085AtAddress(BNO085_I2C_ADDRESS_PRIMARY)) {
    return;
  }

  if (tryBeginBno085AtAddress(BNO085_I2C_ADDRESS_SECONDARY)) {
    return;
  }

  Serial.println(
      "AVISO: BNO085 no detectado en I2C 0x4B ni 0x4A."
  );
}

// ----------------------- Deteccion del encoder ----------------------

void IRAM_ATTR handleEncoderAPulse() {
  // Un uint32_t alineado se lee/escribe atomicamente en el ESP32. Para el
  // arranque solo interesa la cantidad de movimiento, no la direccion.
  ++encoderPulseCount;
}

uint32_t readEncoderPulseCount() {
  return encoderPulseCount;
}

void beginEncoder() {
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  encoderDetectionBaseline = readEncoderPulseCount();
  attachInterrupt(
      digitalPinToInterrupt(ENCODER_A_PIN),
      handleEncoderAPulse,
      RISING
  );
}

void resetMotorStartupDetection() {
  motorStartupState = MotorStartupState::WAITING_FOR_MOVEMENT;
  encoderDetectionBaseline = readEncoderPulseCount();
  encoderDetectionWindowStartMs = 0;
  motorRampStartMs = 0;
  encoderDetectionWindowActive = false;
}

// ------------------------ Control del servo SG90 ---------------------

bool beginServoPwm() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(
      SERVO_PIN,
      SERVO_PWM_FREQUENCY_HZ,
      SERVO_PWM_RESOLUTION_BITS
  );
#else
  ledcSetup(
      SERVO_PWM_CHANNEL,
      SERVO_PWM_FREQUENCY_HZ,
      SERVO_PWM_RESOLUTION_BITS
  );
  ledcAttachPin(SERVO_PIN, SERVO_PWM_CHANNEL);
  return true;
#endif
}

void writeServoAngle(int angleDegrees) {
  if (!servoPwmReady) {
    return;
  }

  angleDegrees = constrain(angleDegrees, 0, 180);

  // El control puede solicitar el mismo angulo miles de veces por segundo
  // durante un retroceso. La senal PWM ya permanece activa por hardware.
  if (servoPwmValueWritten && angleDegrees == currentServoAngle) {
    return;
  }
  currentServoAngle = angleDegrees;

  const uint32_t pulseUs =
      SERVO_PULSO_MINIMO_US +
      (
          static_cast<uint32_t>(angleDegrees) *
          (SERVO_PULSO_MAXIMO_US - SERVO_PULSO_MINIMO_US)
      ) /
          180UL;

  constexpr uint32_t PWM_PERIOD_US =
      1000000UL / SERVO_PWM_FREQUENCY_HZ;
  constexpr uint32_t PWM_MAX_DUTY =
      (1UL << SERVO_PWM_RESOLUTION_BITS) - 1UL;

  const uint32_t duty =
      (pulseUs * PWM_MAX_DUTY + PWM_PERIOD_US / 2UL) /
      PWM_PERIOD_US;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(SERVO_PWM_CHANNEL, duty);
#endif
  servoPwmValueWritten = true;
}

// ---------------------- Control del motor TB6612 --------------------

void setMotorDirectionForward() {
  digitalWrite(
      MOTOR_AIN1_PIN,
      INVERTIR_DIRECCION_MOTOR ? LOW : HIGH
  );
  digitalWrite(
      MOTOR_AIN2_PIN,
      INVERTIR_DIRECCION_MOTOR ? HIGH : LOW
  );
}

void setMotorDirectionReverse() {
  digitalWrite(
      MOTOR_AIN1_PIN,
      INVERTIR_DIRECCION_MOTOR ? HIGH : LOW
  );
  digitalWrite(
      MOTOR_AIN2_PIN,
      INVERTIR_DIRECCION_MOTOR ? LOW : HIGH
  );
}

void writeMotorPwmPercent(uint8_t percent) {
  if (!motorPwmReady) {
    return;
  }

  percent = constrain(percent, 0, 100);

  constexpr uint32_t MOTOR_PWM_MAX_DUTY =
      (1UL << MOTOR_PWM_RESOLUTION_BITS) - 1UL;
  const uint32_t duty =
      (static_cast<uint32_t>(percent) * MOTOR_PWM_MAX_DUTY + 50UL) /
      100UL;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PWM_PIN, duty);
#else
  ledcWrite(MOTOR_PWM_CHANNEL, duty);
#endif
}

void applyMotorPwmPercent(uint8_t percent) {
  percent = constrain(percent, 0, 100);

  if (!motorPwmReady || percent == velocidadAplicadaPorcentaje) {
    return;
  }

  writeMotorPwmPercent(percent);
  velocidadAplicadaPorcentaje = percent;
}

void stopMotor() {
  writeMotorPwmPercent(0);
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
  velocidadObjetivoPorcentaje = 0;
  velocidadAplicadaPorcentaje = 0;
  retrocesoEscapeActivo = false;
  retrocesoEscapeFrenando = false;
  retrocesoEscapeLimiteAlcanzado = false;
  retrocesoEscapeStartMs = 0;
  resetMotorStartupDetection();
  resetPidControl();
}

bool beginMotorPwm() {
  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(
      MOTOR_PWM_PIN,
      MOTOR_PWM_FREQUENCY_HZ,
      MOTOR_PWM_RESOLUTION_BITS
  );
#else
  ledcSetup(
      MOTOR_PWM_CHANNEL,
      MOTOR_PWM_FREQUENCY_HZ,
      MOTOR_PWM_RESOLUTION_BITS
  );
  ledcAttachPin(MOTOR_PWM_PIN, MOTOR_PWM_CHANNEL);
  return true;
#endif
}

// --------------------------- Control PID ----------------------------

float lateralDistanceForControl(uint16_t rawDistanceMm) {
  if (rawDistanceMm == INVALID_DISTANCE_MM ||
      rawDistanceMm > DISTANCIA_LATERAL_MAXIMA_MM) {
    return static_cast<float>(DISTANCIA_LATERAL_MAXIMA_MM);
  }

  return static_cast<float>(rawDistanceMm);
}

float calculateCornerFactor(uint16_t frontDistanceMm) {
  if (frontDistanceMm == INVALID_DISTANCE_MM ||
      frontDistanceMm >= DISTANCIA_FRENTE_INICIO_MM) {
    return 0.0f;
  }

  if (frontDistanceMm <= DISTANCIA_FRENTE_CRITICA_MM) {
    return 1.0f;
  }

  return static_cast<float>(
             DISTANCIA_FRENTE_INICIO_MM - frontDistanceMm
         ) /
         static_cast<float>(
             DISTANCIA_FRENTE_INICIO_MM -
             DISTANCIA_FRENTE_CRITICA_MM
         );
}

float calculateLateralWallFactor(
    float s1,
    float s2,
    float s4,
    float s5
) {
  // La lectura mas cercana de cada lado protege tambien cuando el robot
  // entra inclinado y solo uno de los dos sensores se aproxima a la pared.
  const float closestLeftMm = fminf(s1, s2);
  const float closestRightMm = fminf(s4, s5);
  const float closestSideMm = fminf(closestLeftMm, closestRightMm);
  const float oppositeSideMm = fmaxf(closestLeftMm, closestRightMm);
  const float sideDifferenceMm = oppositeSideMm - closestSideMm;

  if (closestSideMm >= PARED_LATERAL_CERCA_INICIO_MM ||
      sideDifferenceMm <= DIFERENCIA_LATERAL_INICIO_MM) {
    return 0.0f;
  }

  const float proximityFactor = constrain(
      (PARED_LATERAL_CERCA_INICIO_MM - closestSideMm) /
          static_cast<float>(
              PARED_LATERAL_CERCA_INICIO_MM -
              PARED_LATERAL_CERCA_MAXIMA_MM
          ),
      0.0f,
      1.0f
  );
  const float differenceFactor = constrain(
      (sideDifferenceMm - DIFERENCIA_LATERAL_INICIO_MM) /
          static_cast<float>(
              DIFERENCIA_LATERAL_MAXIMA_MM -
              DIFERENCIA_LATERAL_INICIO_MM
          ),
      0.0f,
      1.0f
  );

  // Ambos requisitos deben ser fuertes para aplicar la reduccion completa.
  return fminf(proximityFactor, differenceFactor);
}

float calculateStartupKpFactor(unsigned long nowMs) {
  if (motorStartupState == MotorStartupState::WAITING_FOR_MOVEMENT) {
    return MULTIPLICADOR_KP_INICIAL;
  }

  if (motorStartupState == MotorStartupState::RAMPING) {
    const unsigned long elapsedMs = nowMs - motorRampStartMs;

    if (elapsedMs >= DURACION_ARRANQUE_SUAVE_MS) {
      return 1.0f;
    }

    const float rampProgress =
        static_cast<float>(elapsedMs) /
        static_cast<float>(DURACION_ARRANQUE_SUAVE_MS);

    return MULTIPLICADOR_KP_INICIAL +
           (1.0f - MULTIPLICADOR_KP_INICIAL) * rampProgress;
  }

  return 1.0f;
}

void setMotorSpeedPercent(uint8_t percent) {
  velocidadObjetivoPorcentaje = constrain(percent, 0, 100);

  if (velocidadObjetivoPorcentaje == 0) {
    stopMotor();
    return;
  }

  setMotorDirectionForward();
}

void updateMotorSoftStart(unsigned long nowMs) {
  if (retrocesoEscapeActivo || !motorPwmReady ||
      velocidadObjetivoPorcentaje == 0) {
    return;
  }

  setMotorDirectionForward();

  if (motorStartupState == MotorStartupState::WAITING_FOR_MOVEMENT) {
    applyMotorPwmPercent(VELOCIDAD_DETECCION_ARRANQUE_PORCENTAJE);

    const uint32_t currentPulseCount = readEncoderPulseCount();
    const uint32_t detectedPulses =
        currentPulseCount - encoderDetectionBaseline;

    if (detectedPulses > 0 && !encoderDetectionWindowActive) {
      encoderDetectionWindowActive = true;
      encoderDetectionWindowStartMs = nowMs;
    }

    if (encoderDetectionWindowActive &&
        detectedPulses >= PULSOS_ENCODER_PARA_ARRANCAR) {
      motorStartupState = MotorStartupState::RAMPING;
      motorRampStartMs = nowMs;

      if (!roundTimerStarted) {
        roundTimerStarted = true;
        roundStartMs = nowMs;
        roundFinishMs = 0;
        Serial.println("Cronometro de ronda iniciado.");
      }

      Serial.println(
          "Movimiento de motor detectado: inicia rampa suave."
      );
      return;
    }

    if (encoderDetectionWindowActive &&
        nowMs - encoderDetectionWindowStartMs >=
            VENTANA_DETECCION_MOVIMIENTO_MS) {
      // Pulsos aislados no deben armar el arranque. Se abre una ventana nueva.
      encoderDetectionBaseline = currentPulseCount;
      encoderDetectionWindowActive = false;
    }

    return;
  }

  if (motorStartupState == MotorStartupState::RAMPING) {
    const unsigned long elapsedMs = nowMs - motorRampStartMs;

    if (elapsedMs >= DURACION_ARRANQUE_SUAVE_MS) {
      motorStartupState = MotorStartupState::RUNNING;
      applyMotorPwmPercent(velocidadObjetivoPorcentaje);
      Serial.println("Rampa suave terminada.");
      return;
    }

    const int16_t speedRange =
        static_cast<int16_t>(velocidadObjetivoPorcentaje) -
        static_cast<int16_t>(
            VELOCIDAD_DETECCION_ARRANQUE_PORCENTAJE
        );
    const int16_t rampSpeed =
        static_cast<int16_t>(
            VELOCIDAD_DETECCION_ARRANQUE_PORCENTAJE
        ) +
        static_cast<int32_t>(speedRange) * elapsedMs /
            DURACION_ARRANQUE_SUAVE_MS;

    applyMotorPwmPercent(static_cast<uint8_t>(rampSpeed));
    return;
  }

  applyMotorPwmPercent(velocidadObjetivoPorcentaje);
}

int getReverseAlignmentServoAngle() {
  // Si todavia no hay suficiente giro acumulado, no inventamos una
  // direccion: mantenemos el servo centrado.
  if (!lapTrackingActive ||
      fabsf(accumulatedTurnDegrees) < UMBRAL_SENTIDO_RETROCESO_GRADOS) {
    return SERVO_CENTRO_GRADOS;
  }

  // El signo del giro acumulado define el sentido de la ronda.
  const int8_t roundYawSign =
      accumulatedTurnDegrees > 0.0f ? 1 : -1;

  const int correction =
      roundYawSign *
      DIRECCION_SERVO_REVERSA *
      ANGULO_GUIA_RETROCESO_GRADOS;

  return constrain(
      SERVO_CENTRO_GRADOS + correction,
      SERVO_CENTRO_GRADOS - SERVO_RECORRIDO_MAXIMO_GRADOS,
      SERVO_CENTRO_GRADOS + SERVO_RECORRIDO_MAXIMO_GRADOS
  );
}

bool updateReverseEscape(
    uint16_t frontDistanceMm,
    unsigned long nowMs
) {
  const bool validFrontDistance =
      frontDistanceMm != INVALID_DISTANCE_MM;

  if (!retrocesoEscapeActivo && validFrontDistance &&
      frontDistanceMm <= DISTANCIA_FRENTE_RETROCESO_MM) {
    // Corta el PWM antes de invertir el puente H.
    writeMotorPwmPercent(0);
    velocidadAplicadaPorcentaje = 0;
    digitalWrite(MOTOR_AIN1_PIN, LOW);
    digitalWrite(MOTOR_AIN2_PIN, LOW);

    retrocesoEscapeActivo = true;
    retrocesoEscapeFrenando = true;
    retrocesoEscapeLimiteAlcanzado = false;
    retrocesoEscapeStartMs = nowMs;
    velocidadObjetivoPorcentaje = 0;
    resetPidControl();

    writeServoAngle(getReverseAlignmentServoAngle());

    Serial.print("Obstaculo a menos de ");
    Serial.print(DISTANCIA_FRENTE_RETROCESO_MM / 10);
    Serial.println(" cm: inicia frenado antes del retroceso.");
  }

  const bool frontClear =
      validFrontDistance &&
      frontDistanceMm >= DISTANCIA_FRENTE_SALIDA_RETROCESO_MM;
  const bool reverseTimeout =
      retrocesoEscapeActivo && !frontClear &&
      !retrocesoEscapeFrenando &&
      !retrocesoEscapeLimiteAlcanzado &&
      nowMs - retrocesoEscapeStartMs >= DURACION_MAXIMA_RETROCESO_MS;

  if (retrocesoEscapeActivo && frontClear) {
    // Detiene el motor antes de recuperar el sentido hacia adelante.
    writeMotorPwmPercent(0);
    velocidadAplicadaPorcentaje = 0;
    velocidadObjetivoPorcentaje = 0;
    retrocesoEscapeActivo = false;
    retrocesoEscapeFrenando = false;
    retrocesoEscapeLimiteAlcanzado = false;
    retrocesoEscapeStartMs = 0;

    setMotorDirectionForward();
    resetMotorStartupDetection();
    resetPidControl();

    Serial.println(
        "Espacio frontal liberado: termina el retroceso de escape."
    );
  }

  if (retrocesoEscapeActivo && retrocesoEscapeFrenando &&
      nowMs - retrocesoEscapeStartMs >=
          DURACION_FRENO_ANTES_RETROCESO_MS) {
    retrocesoEscapeFrenando = false;
    retrocesoEscapeStartMs = nowMs;
    velocidadObjetivoPorcentaje =
        VELOCIDAD_RETROCESO_ESCAPE_PORCENTAJE;

    setMotorDirectionReverse();
    applyMotorPwmPercent(VELOCIDAD_RETROCESO_ESCAPE_PORCENTAJE);

    Serial.print("Inicia retroceso al ");
    Serial.print(VELOCIDAD_RETROCESO_ESCAPE_PORCENTAJE);
    Serial.print("%, guiando al sentido de la ronda. Servo=");
    Serial.println(getReverseAlignmentServoAngle());
  }

  if (reverseTimeout) {
    writeMotorPwmPercent(0);
    velocidadAplicadaPorcentaje = 0;
    velocidadObjetivoPorcentaje = 0;
    retrocesoEscapeLimiteAlcanzado = true;

    Serial.println(
        "Tiempo maximo de retroceso alcanzado: motor detenido."
    );
  }

  if (retrocesoEscapeActivo) {
    writeServoAngle(getReverseAlignmentServoAngle());

    if (!retrocesoEscapeFrenando &&
        !retrocesoEscapeLimiteAlcanzado) {
      setMotorDirectionReverse();
      applyMotorPwmPercent(VELOCIDAD_RETROCESO_ESCAPE_PORCENTAJE);
    }
  }

  return retrocesoEscapeActivo;
}

void applyQuarterTurnGuard(
    unsigned long nowMs,
    int &servoAngle,
    uint8_t &speedPercent
) {
  if (!SEGURO_CADA_90_HABILITADO || !lapTrackingActive ||
      !bno085HasHeading || bno085FailsafeActive || missionComplete ||
      retrocesoEscapeActivo || finalReverseActive) {
    return;
  }

  const float absoluteTurnDegrees = fabsf(accumulatedTurnDegrees);
  const float targetDegrees =
      static_cast<float>(nextQuarterTurnGuard) * GRADOS_POR_ESQUINA;

  if (quarterTurnGuardHolding) {
    servoAngle = SERVO_CENTRO_GRADOS;
    speedPercent = min(
        speedPercent,
        VELOCIDAD_SEGURO_90_PORCENTAJE
    );

    if (nowMs - quarterTurnGuardStartMs >=
        DURACION_SERVO_CENTRADO_90_MS) {
      quarterTurnGuardHolding = false;
      quarterTurnGuardStartMs = 0;
      quarterTurnGuardReleaseStartMs = nowMs;
      resetPidControl();
      if (nextQuarterTurnGuard < VUELTAS_OBJETIVO * 4) {
        ++nextQuarterTurnGuard;
      }
      Serial.println("Seguro de 90 terminado: regresa el control PID.");
    }
    return;
  }

  if (absoluteTurnDegrees >= targetDegrees) {
    quarterTurnGuardHolding = true;
    quarterTurnGuardStartMs = nowMs;
    quarterTurnGuardReleaseStartMs = 0;
    resetPidControl();
    servoAngle = SERVO_CENTRO_GRADOS;
    speedPercent = min(
        speedPercent,
        VELOCIDAD_SEGURO_90_PORCENTAJE
    );

    Serial.print("Seguro de 90 activado en ");
    Serial.print(accumulatedTurnDegrees, 1);
    Serial.println(" grados: servo centrado.");
    return;
  }

  if (quarterTurnGuardReleaseStartMs != 0) {
    const unsigned long releaseElapsedMs =
        nowMs - quarterTurnGuardReleaseStartMs;

    if (releaseElapsedMs >= DURACION_REENTRADA_PID_90_MS) {
      quarterTurnGuardReleaseStartMs = 0;
    } else {
      const float releaseProgress =
          static_cast<float>(releaseElapsedMs) /
          static_cast<float>(DURACION_REENTRADA_PID_90_MS);
      const int requestedCorrection = servoAngle - SERVO_CENTRO_GRADOS;
      servoAngle = SERVO_CENTRO_GRADOS + static_cast<int>(roundf(
          requestedCorrection * releaseProgress
      ));
      speedPercent = min(
          speedPercent,
          VELOCIDAD_MOTOR_EN_ESQUINA_PORCENTAJE
      );
    }
  }

  const float remainingDegrees = targetDegrees - absoluteTurnDegrees;
  if (remainingDegrees <= ANTICIPACION_SEGURO_90_GRADOS) {
    servoAngle = constrain(
        servoAngle,
        SERVO_CENTRO_GRADOS - CORRECCION_MAXIMA_PREVIA_90_GRADOS,
        SERVO_CENTRO_GRADOS + CORRECCION_MAXIMA_PREVIA_90_GRADOS
    );
    speedPercent = min(
        speedPercent,
        VELOCIDAD_SEGURO_90_PORCENTAJE
    );
  }
}

ControlState updatePidControl(
    const uint16_t distancesMm[SENSOR_COUNT],
    unsigned long nowMs
) {
  const float s1 = lateralDistanceForControl(distancesMm[0]);
  const float s2 = lateralDistanceForControl(distancesMm[1]);
  const float s4 = lateralDistanceForControl(distancesMm[3]);
  const float s5 = lateralDistanceForControl(distancesMm[4]);

  ControlState state;
  state.leftValue = s1 * PESO_S1 + s2 * PESO_S2;
  state.rightValue = s4 * PESO_S4 + s5 * PESO_S5;
  state.error = state.leftValue - state.rightValue;

  state.cornerFactor = calculateCornerFactor(distancesMm[2]);
  latestCornerFactor = state.cornerFactor;
  state.lateralWallFactor = calculateLateralWallFactor(s1, s2, s4, s5);
  latestLateralWallFactor = state.lateralWallFactor;
  state.startupKpFactor = calculateStartupKpFactor(nowMs);
  const float wallProximityFactor =
      fmaxf(state.cornerFactor, state.lateralWallFactor);
  state.effectiveKp =
      KP *
      (
          1.0f +
          wallProximityFactor *
              (MULTIPLICADOR_KP_EN_ESQUINA - 1.0f)
      ) *
      state.startupKpFactor;

  const unsigned long nowUs = micros();
  float derivative = 0.0f;

  if (pidHasPreviousSample) {
    const float dtSeconds =
        static_cast<float>(nowUs - pidPreviousUpdateUs) / 1000000.0f;

    if (dtSeconds > 0.0f && dtSeconds <= 0.25f) {
      pidIntegralError += state.error * dtSeconds;

      if (KI > 0.0f) {
        const float integralLimitError = LIMITE_INTEGRAL_GRADOS / KI;
        pidIntegralError = constrain(
            pidIntegralError,
            -integralLimitError,
            integralLimitError
        );
      } else {
        pidIntegralError = 0.0f;
      }

      const float rawDerivative =
          (state.error - pidPreviousError) / dtSeconds;
      pidFilteredDerivative +=
          FILTRO_DERIVADA *
          (rawDerivative - pidFilteredDerivative);
      derivative = pidFilteredDerivative;
    }
  } else {
    pidHasPreviousSample = true;
  }

  pidPreviousError = state.error;
  pidPreviousUpdateUs = nowUs;

  state.pTerm = state.effectiveKp * state.error;
  state.iTerm = constrain(
      KI * pidIntegralError,
      -LIMITE_INTEGRAL_GRADOS,
      LIMITE_INTEGRAL_GRADOS
  );
  state.dTerm = KD * derivative;

  float correctionDegrees =
      static_cast<float>(DIRECCION_SERVO) *
      (state.pTerm + state.iTerm + state.dTerm);

  correctionDegrees = constrain(
      correctionDegrees,
      -static_cast<float>(SERVO_RECORRIDO_MAXIMO_GRADOS),
      static_cast<float>(SERVO_RECORRIDO_MAXIMO_GRADOS)
  );

  state.servoAngle = static_cast<int>(
      roundf(SERVO_CENTRO_GRADOS + correctionDegrees)
  );

  // El SG90 no necesita recibir saltos de decenas de grados por una sola
  // lectura ruidosa. El objetivo final se conserva y se alcanza por pasos.
  state.servoAngle = constrain(
      state.servoAngle,
      currentServoAngle - PASO_MAXIMO_SERVO_POR_LECTURA_GRADOS,
      currentServoAngle + PASO_MAXIMO_SERVO_POR_LECTURA_GRADOS
  );

  const float cornerSpeed =
      VELOCIDAD_MOTOR_PORCENTAJE -
      state.cornerFactor *
          (VELOCIDAD_MOTOR_PORCENTAJE -
           VELOCIDAD_MOTOR_EN_ESQUINA_PORCENTAJE);
  const float lateralWallSpeed =
      VELOCIDAD_MOTOR_PORCENTAJE -
      state.lateralWallFactor *
          (VELOCIDAD_MOTOR_PORCENTAJE -
           VELOCIDAD_MINIMA_PARED_LATERAL_PORCENTAJE);

  // No suma reducciones: toma la velocidad mas segura de las dos.
  state.speedPercent = static_cast<uint8_t>(
      roundf(fminf(cornerSpeed, lateralWallSpeed))
  );

  applyQuarterTurnGuard(
      nowMs,
      state.servoAngle,
      state.speedPercent
  );

  writeServoAngle(state.servoAngle);
  setMotorSpeedPercent(state.speedPercent);

  return state;
}

// ------------------------- Comunicacion I2C --------------------------

bool sendBrightnessPercent(uint8_t percent) {
  if (percent > 100) {
    percent = 100;
  }

  Wire.beginTransmission(NANO_I2C_ADDRESS);
  Wire.write(percent);
  const uint8_t error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("Error I2C al enviar intensidad. Codigo: ");
    Serial.println(error);
    return false;
  }

  return true;
}

void updateCornerLedIndicator(float cornerFactor) {
  const bool cornerDetected = cornerFactor > 0.0f;

  if (cornerDetected == ledsIndicandoEsquina) {
    return;
  }

  const uint8_t targetBrightness =
      cornerDetected ? BRILLO_LED_EN_ESQUINA_PORCENTAJE
                     : brilloLedBasePorcentaje;

  if (!sendBrightnessPercent(targetBrightness)) {
    return;
  }

  ledsIndicandoEsquina = cornerDetected;
  Serial.print("Indicador de esquina: LEDs ");
  Serial.print(cornerDetected ? "encendidos al " : "restaurados al ");
  Serial.print(targetBrightness);
  Serial.println("%.");
}

bool readDistancesMm(uint16_t outputMm[SENSOR_COUNT]) {
  const uint8_t received = Wire.requestFrom(
      static_cast<uint8_t>(NANO_I2C_ADDRESS),
      static_cast<uint8_t>(DISTANCE_PACKET_BYTES)
  );

  if (received != DISTANCE_PACKET_BYTES) {
    Serial.print("Respuesta I2C incompleta: ");
    Serial.print(received);
    Serial.print("/");
    Serial.println(DISTANCE_PACKET_BYTES);

    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint8_t lowByte = Wire.read();
    const uint8_t highByte = Wire.read();
    const uint16_t decodedDistance =
        static_cast<uint16_t>(lowByte) |
        (static_cast<uint16_t>(highByte) << 8);
    outputMm[i] = decodedDistance == INVALID_DISTANCE_MM
                      ? DISTANCIA_SIN_ECO_MM
                      : decodedDistance;
  }

  return true;
}

bool updateStartupCorridorClassification(
    const uint16_t distancesMm[SENSOR_COUNT]
) {
  if (startupCorridorClassified) {
    return true;
  }

  const uint16_t leftDistanceMm = distancesMm[0];   // S1, 90 grados
  const uint16_t rightDistanceMm = distancesMm[4];  // S5, 90 grados
  if (leftDistanceMm == INVALID_DISTANCE_MM ||
      rightDistanceMm == INVALID_DISTANCE_MM) {
    return false;
  }

  startupCorridorWidthAccumulatorMm +=
      static_cast<uint32_t>(leftDistanceMm) + rightDistanceMm;
  ++startupCorridorSampleCount;

  if (startupCorridorSampleCount < MUESTRAS_PARA_CLASIFICAR_PASILLO) {
    return false;
  }

  startupCorridorWidthMm = static_cast<uint16_t>(
      startupCorridorWidthAccumulatorMm / startupCorridorSampleCount
  );
  startupCorridorIsWide =
      startupCorridorWidthMm >= UMBRAL_PASILLO_ANCHO_MM;
  selectedFinalAdvanceDurationMs = startupCorridorIsWide
      ? DURACION_AVANCE_FINAL_PASILLO_ANCHO_MS
      : DURACION_AVANCE_FINAL_PASILLO_ANGOSTO_MS;
  startupCorridorClassified = true;

  Serial.print("Pasillo inicial clasificado como ");
  Serial.print(startupCorridorIsWide ? "ANCHO" : "ANGOSTO");
  Serial.print(": S1 + S5 promedio = ");
  Serial.print(startupCorridorWidthMm / 10);
  Serial.print(".");
  Serial.print(startupCorridorWidthMm % 10);
  Serial.print(" cm; avance final = ");
  Serial.print(selectedFinalAdvanceDurationMs);
  Serial.println(" ms.");
  return true;
}

void printDistances(const uint16_t distancesMm[SENSOR_COUNT]) {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(": ");

    if (distancesMm[i] == INVALID_DISTANCE_MM) {
      Serial.print("sin eco");
    } else {
      // Imprime centimetros con un decimal, sin usar float.
      Serial.print(distancesMm[i] / 10);
      Serial.print(".");
      Serial.print(distancesMm[i] % 10);
      Serial.print(" cm");
    }

    if (i < SENSOR_COUNT - 1) {
      Serial.print(" | ");
    }
  }

  Serial.println();
}

void printControlState(const ControlState &state) {
  Serial.print("Error: ");
  Serial.print(state.error, 1);
  Serial.print(" | Rampa Kp: ");
  Serial.print(state.startupKpFactor * 100.0f, 0);
  Serial.print("%");
  Serial.print(" | Kp efectivo: ");
  Serial.print(state.effectiveKp, 4);
  Serial.print(" | P: ");
  Serial.print(state.pTerm, 1);
  Serial.print(" I: ");
  Serial.print(state.iTerm, 1);
  Serial.print(" D: ");
  Serial.print(state.dTerm, 1);
  Serial.print(" | Esquina: ");
  Serial.print(state.cornerFactor * 100.0f, 0);
  Serial.print("% | Proteccion lateral: ");
  Serial.print(state.lateralWallFactor * 100.0f, 0);
  Serial.print("% | Servo: ");
  Serial.print(state.servoAngle);
  Serial.print(" grados | Velocidad objetivo: ");
  Serial.print(state.speedPercent);
  Serial.print("% | Velocidad aplicada: ");
  Serial.print(velocidadAplicadaPorcentaje);
  Serial.print("% | Rumbo: ");
  Serial.print(currentYawDegrees, 1);
  Serial.print(" grados | Giro acumulado: ");
  Serial.print(accumulatedTurnDegrees, 1);
  Serial.print(" grados | Vueltas: ");
  Serial.print(completedLaps);
  Serial.print("/");
  Serial.println(VUELTAS_OBJETIVO);
}

// ---------------------- Control desde el monitor ---------------------

// Escribe un numero de 0 a 100 y Enter en el monitor serial.
// En el programa final tambien puedes llamar directamente a:
//   sendBrightnessPercent(valor);
void handleSerialBrightnessCommand() {
  static char input[8];
  static uint8_t length = 0;
  uint8_t processedBytes = 0;

  while (processedBytes < MAX_SERIAL_BYTES_PER_LOOP && Serial.available()) {
    ++processedBytes;
    const char character = static_cast<char>(Serial.read());

    if (character == '\r' || character == '\n') {
      if (length == 0) {
        continue;
      }

      input[length] = '\0';
      char *endPointer = nullptr;
      const long value = strtol(input, &endPointer, 10);

      if (*endPointer == '\0' && value >= 0 && value <= 100) {
        brilloLedBasePorcentaje = static_cast<uint8_t>(value);

        if (ledsIndicandoEsquina) {
          Serial.print("Intensidad base guardada: ");
          Serial.print(value);
          Serial.println("%. Se aplicara al salir de la esquina.");
        } else if (sendBrightnessPercent(brilloLedBasePorcentaje)) {
          Serial.print("Intensidad enviada al Nano: ");
          Serial.print(value);
          Serial.println("%");
        }
      } else {
        Serial.println("Valor invalido. Escribe un numero de 0 a 100.");
      }

      length = 0;
      continue;
    }

    if (length < sizeof(input) - 1) {
      input[length++] = character;
    } else {
      // Si la linea es demasiado larga, se descarta.
      length = 0;
      Serial.println("Entrada demasiado larga.");
    }
  }
}

// ----------------------------- Arduino -------------------------------

void setup() {
  Serial.begin(115200);

  beginEncoder();

  Wire.begin(
      ESP32_SDA_PIN,
      ESP32_SCL_PIN,
      I2C_FREQUENCY_HZ
  );
  Wire.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);
  beginBno085I2c();

  servoPwmReady = beginServoPwm();
  if (servoPwmReady) {
    writeServoAngle(SERVO_CENTRO_GRADOS);
  } else {
    Serial.println("ERROR: no se pudo iniciar el PWM del servo.");
  }

  motorPwmReady = beginMotorPwm();
  if (motorPwmReady) {
    stopMotor();
  } else {
    Serial.println("ERROR: no se pudo iniciar el PWM del motor.");
  }

  delay(10);

  // Estado seguro al arrancar: tira apagada.
  sendBrightnessPercent(brilloLedBasePorcentaje);

  beginDashboard();

  Serial.println();
  Serial.println("ESP32 maestro I2C listo.");
  Serial.print("Servo centrado en ");
  Serial.print(SERVO_CENTRO_GRADOS);
  Serial.println(" grados.");
  Serial.println("Motor detenido; arrancara al recibir sensores por I2C.");
  Serial.print("Encoder: A=D5, B=D9. PWM de espera: ");
  Serial.print(VELOCIDAD_DETECCION_ARRANQUE_PORCENTAJE);
  Serial.println("%.");
  Serial.print("BNO085 I2C en SDA=21/SCL=22. Objetivo: ");
  Serial.print(VUELTAS_OBJETIVO);
  Serial.println(" vueltas.");
  Serial.println("Esperando el primer paquete de rumbo para habilitar motor.");
  Serial.println("Escribe una intensidad de 0 a 100 y presiona Enter.");
}

void loop() {
  handleSerialBrightnessCommand();
  updateDashboard();

  static unsigned long lastPollMs = 0;
  static unsigned long lastValidSensorMs = 0;
  const unsigned long nowMs = millis();

  updateBno085(nowMs);
  updateFinalAdvance(nowMs);
  updateFinalReverse(nowMs);

  if (lapTrackingActive && !missionComplete && bno085HasHeading &&
      nowMs - lastBno085FrameMs >= BNO085_FAILSAFE_TIMEOUT_MS) {
    if (!bno085FailsafeActive) {
      bno085FailsafeActive = true;

      if (finalReverseActive) {
        Serial.println(
            "Ajuste final cancelado: se perdio el rumbo del BNO085."
        );
        finishThreeLapMission();
      } else if (EXIGIR_BNO085_PARA_MOVERSE) {
        stopMotor();
        writeServoAngle(SERVO_CENTRO_GRADOS);
        Serial.println("Motor detenido: sin datos de rumbo del BNO085.");
      } else {
        Serial.println(
            "AVISO: sin BNO085; continua movimiento sin conteo de vueltas."
        );
      }
    }
  }

  if (nowMs - lastPollMs >= SENSOR_POLL_INTERVAL_MS) {
    lastPollMs = nowMs;

    uint16_t distancesMm[SENSOR_COUNT];
    if (readDistancesMm(distancesMm)) {
      lastValidSensorMs = nowMs;
      distanceSensorFailsafeActive = false;

      for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        latestDistancesMm[i] = distancesMm[i];
      }

      // Mantiene el robot quieto durante las primeras muestras para que la
      // medida S1 + S5 represente el ancho real del pasillo de arranque.
      if (!updateStartupCorridorClassification(distancesMm)) {
        if (velocidadObjetivoPorcentaje > 0 ||
            velocidadAplicadaPorcentaje > 0) {
          stopMotor();
        }
        writeServoAngle(SERVO_CENTRO_GRADOS);
        return;
      }

      static unsigned long lastPrintMs = 0;
      const float cornerFactor = calculateCornerFactor(distancesMm[2]);

      // Al completar tres vueltas el paro queda enclavado. El BNO085 solo
      // bloquea el movimiento si EXIGIR_BNO085_PARA_MOVERSE esta activado.
      const bool bno085BlocksMovement =
          EXIGIR_BNO085_PARA_MOVERSE &&
          (!bno085HasHeading || bno085FailsafeActive);

      if (missionComplete || bno085BlocksMovement) {
        if (velocidadObjetivoPorcentaje > 0 ||
            velocidadAplicadaPorcentaje > 0 || retrocesoEscapeActivo) {
          stopMotor();
          writeServoAngle(SERVO_CENTRO_GRADOS);
        }
        return;
      }

      // El ajuste de llegada controla motor y servo desde updateFinalReverse().
      if (finalReverseActive) {
        return;
      }

      const bool reversing = updateReverseEscape(distancesMm[2], nowMs);

      updateCornerLedIndicator(cornerFactor);

      if (reversing) {
        if (nowMs - lastPrintMs >= SERIAL_PRINT_INTERVAL_MS) {
          lastPrintMs = nowMs;
          printDistances(distancesMm);

          if (retrocesoEscapeFrenando) {
            Serial.println(
                "Escape activo: frenando, servo centrado."
            );
          } else if (retrocesoEscapeLimiteAlcanzado) {
            Serial.println(
                "Escape en espera: motor detenido, servo centrado."
            );
          } else {
            Serial.print("Escape activo: retroceso al ");
            Serial.print(VELOCIDAD_RETROCESO_ESCAPE_PORCENTAJE);
            Serial.println("%, servo centrado.");
          }
        }
      } else {
        const ControlState state =
            updatePidControl(distancesMm, nowMs);

        updateMotorSoftStart(nowMs);

        if (nowMs - lastPrintMs >= SERIAL_PRINT_INTERVAL_MS) {
          lastPrintMs = nowMs;
          printDistances(distancesMm);
          printControlState(state);
        }
      }
    }
  }

  // Mantiene la rampa fluida aun entre dos consultas I2C.
  const bool bno085AllowsMovement =
      !EXIGIR_BNO085_PARA_MOVERSE ||
      (bno085HasHeading && !bno085FailsafeActive);

  if (!missionComplete && !finalReverseActive && bno085AllowsMovement &&
      !retrocesoEscapeActivo) {
    updateMotorSoftStart(nowMs);
  }

  if ((velocidadObjetivoPorcentaje > 0 || retrocesoEscapeActivo) &&
      nowMs - lastValidSensorMs >= SENSOR_FAILSAFE_TIMEOUT_MS) {
    if (finalReverseActive) {
      Serial.println(
          "Ajuste final cancelado: se perdieron los sensores de distancia."
      );
      distanceSensorFailsafeActive = true;
      finishThreeLapMission();
      updateCornerLedIndicator(0.0f);
      return;
    }

    retrocesoEscapeActivo = false;
    retrocesoEscapeLimiteAlcanzado = false;
    stopMotor();
    updateCornerLedIndicator(0.0f);

    if (!distanceSensorFailsafeActive) {
      distanceSensorFailsafeActive = true;
      Serial.println("Motor detenido: sin datos I2C de sensores.");
    }
  }
}
