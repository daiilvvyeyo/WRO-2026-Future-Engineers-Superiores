#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// ====================================================================
//                    CONFIGURACION DE LAS TIRAS LED
// ====================================================================
// Tres tiras horizontales de ocho NeoPixels cada una. Se consideran una
// cadena logica de 24 LEDs; si las tres entradas DIN estan conectadas en
// paralelo, cada tira reproducira igualmente los primeros ocho valores.
constexpr uint8_t NUMERO_DE_TIRAS = 3;
constexpr uint8_t LEDS_POR_TIRA = 8;
constexpr uint16_t NUMERO_DE_LEDS =
    static_cast<uint16_t>(NUMERO_DE_TIRAS) * LEDS_POR_TIRA;

// false enciende 1, 3, 5 y 7 contando fisicamente desde 1.
// Cambia a true si prefieres encender 2, 4, 6 y 8.
constexpr bool EMPEZAR_CON_PRIMER_LED_APAGADO = false;
// ====================================================================

// --------------------------- Configuracion ---------------------------
constexpr uint8_t I2C_ADDRESS = 0x08;
constexpr uint8_t SENSOR_COUNT = 5;

constexpr uint8_t TRIGGER_PINS[SENSOR_COUNT] = {2, 4, 6, 8, 10};
constexpr uint8_t ECHO_PINS[SENSOR_COUNT] = {3, 5, 7, 9, 11};

// Distribucion fisica:
// S1 = izquierda 90 grados, S2 = izquierda 25 grados, S3 = frontal,
// S4 = derecha 25 grados y S5 = derecha 90 grados.
// Se lee primero S3 frontal y despues se alternan ambos lados.
constexpr uint8_t SENSOR_SCAN_ORDER[SENSOR_COUNT] = {2, 0, 4, 1, 3};

constexpr uint8_t NEOPIXEL_PIN = 12;

// Color de la tira cuando esta encendida: blanco.
// Puedes cambiar estos tres valores para usar otro color.
constexpr uint8_t LED_RED = 255;
constexpr uint8_t LED_GREEN = 255;
constexpr uint8_t LED_BLUE = 255;

// Para el control no necesitamos esperar ecos de hasta 4 m.
// 12 ms permite medir aproximadamente hasta 2 m y reduce mucho el tiempo
// perdido cuando un sensor no recibe eco.
constexpr unsigned long ECHO_TIMEOUT_US = 12000UL;
constexpr uint16_t MAX_DISTANCE_MM = 2000;
constexpr uint16_t INVALID_DISTANCE_MM = 0xFFFF;

// Con cinco sensores, 10 ms deja aproximadamente 50 ms como minimo antes de
// volver a disparar el mismo sensor. Si aparece interferencia, sube a 15 o 20.
constexpr uint8_t INTER_SENSOR_DELAY_MS = 10;

Adafruit_NeoPixel strip(
    NUMERO_DE_LEDS,
    NEOPIXEL_PIN,
    NEO_GRB + NEO_KHZ800
);

// Estas variables se comparten entre loop() y las interrupciones de I2C.
volatile uint16_t distancesMm[SENSOR_COUNT] = {
    INVALID_DISTANCE_MM,
    INVALID_DISTANCE_MM,
    INVALID_DISTANCE_MM,
    INVALID_DISTANCE_MM,
    INVALID_DISTANCE_MM
};

volatile uint8_t requestedBrightnessPercent = 0;

// -------------------------- Sensores HC-SR04 -------------------------

uint16_t readDistanceMm(uint8_t triggerPin, uint8_t echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(3);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  const unsigned long durationUs =
      pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

  if (durationUs == 0) {
    return INVALID_DISTANCE_MM;
  }

  // Distancia aproximada:
  //   centimetros = duracion_us / 58
  //   milimetros  = duracion_us * 10 / 58
  const unsigned long distanceMm =
      (durationUs * 10UL + 29UL) / 58UL;

  if (distanceMm > MAX_DISTANCE_MM) {
    return INVALID_DISTANCE_MM;
  }

  return static_cast<uint16_t>(distanceMm);
}

void scanAllSensors() {
  for (uint8_t position = 0; position < SENSOR_COUNT; ++position) {
    const uint8_t sensorIndex = SENSOR_SCAN_ORDER[position];
    const uint16_t newDistance = readDistanceMm(
        TRIGGER_PINS[sensorIndex],
        ECHO_PINS[sensorIndex]
    );

    // Publica cada lectura inmediatamente; ya no espera a terminar los cinco.
    // En un ATmega328P, copiar uint16_t no es atomico.
    noInterrupts();
    distancesMm[sensorIndex] = newDistance;
    interrupts();

    // Permite aplicar una nueva intensidad sin esperar una ronda completa.
    updateNeoPixelsIfNeeded();
    delay(INTER_SENSOR_DELAY_MS);
  }
}

// ----------------------------- NeoPixel ------------------------------

void updateNeoPixelsIfNeeded() {
  static uint8_t appliedPercent = 0xFF;

  const uint8_t percent = requestedBrightnessPercent;
  if (percent == appliedPercent) {
    return;
  }

  appliedPercent = percent;

  // El valor recibido por I2C siempre esta limitado a 0..100.
  const uint8_t brightness255 =
      static_cast<uint8_t>(map(percent, 0, 100, 0, 255));

  // Se escala el color directamente para evitar perdidas acumuladas al
  // cambiar repetidamente el brillo global de la libreria.
  const uint8_t red =
      (static_cast<uint16_t>(LED_RED) * brightness255) / 255;
  const uint8_t green =
      (static_cast<uint16_t>(LED_GREEN) * brightness255) / 255;
  const uint8_t blue =
      (static_cast<uint16_t>(LED_BLUE) * brightness255) / 255;

  const uint32_t activeColor = strip.Color(red, green, blue);
  strip.clear();

  // En cada tira enciende un LED si y uno no. El calculo se reinicia al
  // comenzar cada segmento para que las tres tiras tengan el mismo patron.
  for (uint8_t stripIndex = 0; stripIndex < NUMERO_DE_TIRAS; ++stripIndex) {
    const uint16_t firstLed =
        static_cast<uint16_t>(stripIndex) * LEDS_POR_TIRA;

    for (uint8_t ledInStrip = 0; ledInStrip < LEDS_POR_TIRA; ++ledInStrip) {
      const bool evenPosition = (ledInStrip % 2U) == 0;
      const bool turnOn = EMPEZAR_CON_PRIMER_LED_APAGADO
                              ? !evenPosition
                              : evenPosition;
      if (turnOn) {
        strip.setPixelColor(firstLed + ledInStrip, activeColor);
      }
    }
  }

  strip.show();
}

// ------------------------------- I2C ---------------------------------

// El ESP32 escribe un byte con la intensidad solicitada, de 0 a 100.
void onI2CReceive(int byteCount) {
  if (byteCount <= 0 || !Wire.available()) {
    return;
  }

  uint8_t percent = Wire.read();

  // Descarta cualquier byte adicional para dejar limpio el buffer.
  while (Wire.available()) {
    Wire.read();
  }

  if (percent > 100) {
    percent = 100;
  }

  requestedBrightnessPercent = percent;
}

// El ESP32 solicita 10 bytes:
// sensor 1 LOW, sensor 1 HIGH, ... sensor 5 LOW, sensor 5 HIGH.
void onI2CRequest() {
  uint8_t packet[SENSOR_COUNT * 2];

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint16_t distance = distancesMm[i];
    packet[i * 2] = static_cast<uint8_t>(distance & 0xFF);
    packet[i * 2 + 1] = static_cast<uint8_t>(distance >> 8);
  }

  Wire.write(packet, sizeof(packet));
}

// ----------------------------- Arduino -------------------------------

void setup() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(TRIGGER_PINS[i], OUTPUT);
    digitalWrite(TRIGGER_PINS[i], LOW);
    pinMode(ECHO_PINS[i], INPUT);
  }

  strip.begin();
  strip.clear();
  strip.show();

  // En el Nano clasico: SDA = A4 y SCL = A5.
  Wire.begin(I2C_ADDRESS);

  // La libreria Wire del AVR puede activar pull-ups internos hacia 5 V.
  // Se desactivan porque este bus comparte lineas con un ESP32 de 3.3 V.
  // La DFR0478 ya incluye pull-ups de 10 kohm hacia 3.3 V en SDA y SCL.
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);

  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}

void loop() {
  updateNeoPixelsIfNeeded();
  scanAllSensors();
  updateNeoPixelsIfNeeded();
}
