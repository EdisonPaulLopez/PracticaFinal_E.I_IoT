/*
 * ============================================================
 *  SISTEMA DE ANÁLISIS DE SUELO — ESP32-CAM (SERVIDOR BLE)
 *  Versión corregida — Mayo 2025
 * ===========================================================0
 *  Librerías requeridas:
 *   - NimBLE-Arduino (h2zero) v1.4.x o v2.x
 *   - Edge Impulse SDK (exportado como librería Arduino)
 *   - esp_camera (incluida en ESP32 Arduino Core)
 * ============================================================
 */

// ─── MODELO EDGE IMPULSE ─────────────────────────────────────
#include <Suelos_inferencing.h>  

// ─── LIBRERÍAS ───────────────────────────────────────────────
#include "esp_camera.h"
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include "esp_heap_caps.h"

// ─── PINES AI THINKER ESP32-CAM ──────────────────────────────
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ─── GPIO DISPONIBLES EN AI THINKER ─────────────────────────
// GPIO4  = Flash LED (ALTO BRILLO — usar solo brevemente)
// GPIO2  = LED azul integrado en la placa
// GPIO12 = disponible en conector de expansión
// GPIO13 = disponible en conector de expansión
#define LED_STATUS_PIN   2    // LED azul de la placa 
#define FLASH_LED_PIN    4    // Flash LED
#define BTN_ENCENDIDO_PIN 13  // Botón de encendido/apagado
#define BTN_CAPTURA_PIN  12   // Botón de captura de imagen

// ─── BLE UUIDs ───────────────────────────────────────────────
// DEBEN coincidir EXACTAMENTE con el ESP32-Sensores cliente
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_DEVICE_NAME     "SueloSensor_CAM"

// ─── CONSTANTES ──────────────────────────────────────────────
#define MAX_LABEL_LEN  32
#define FLASH_MS       150   // Tiempo de encendido del flash en ms

// ─── OBJETOS NimBLE ──────────────────────────────────────────
NimBLEServer*         pServer         = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;

// ─── ESTADO ──────────────────────────────────────────────────
volatile bool deviceConnected    = false;
volatile bool sistemaActivo      = false;
volatile bool btnEncendidoFlag   = false;
volatile bool btnCapturaFlag     = false;

static char  soilTypeResult[MAX_LABEL_LEN] = "Sin_Datos";
static float topConfidence = 0.0f;

// ─── ISR ─────────────────────────────────────────────────────
void IRAM_ATTR isr_encendido() { btnEncendidoFlag = true; }
void IRAM_ATTR isr_captura()   { btnCapturaFlag   = true; }

// ─── CALLBACKS NimBLE ────────────────────────────────────────
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    Serial.printf("[BLE] Cliente conectado. Handle: %d  Addr: %s\n",
                  connInfo.getConnHandle(),
                  connInfo.getAddress().toString().c_str());
    // Opcional: detener advertising cuando hay un cliente conectado
    // NimBLEDevice::stopAdvertising();
  }

  void onDisconnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    Serial.printf("[BLE] Cliente desconectado. Razon: %d\n", reason);
    // Reiniciar advertising para aceptar nuevas conexiones
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising reiniciado");
  }
};

// ─── INICIALIZAR BLE ─────────────────────────────────────────
void initBLE() {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // Potencia máxima para mayor alcance
  NimBLEDevice::setMTU(128);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Crear servicio y característica
  NimBLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pCharacteristic->setValue("ESPERANDO");
  pService->start();

  // ── Advertising principal ─────────────────────────────────
  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setAppearance(0x0180);          // Generic Sensor
  pAdv->enableScanResponse(true);       // NECESARIO para UUID de 128 bits
  pAdv->setMinInterval(32);   // 32 × 0.625ms = 20ms
  pAdv->setMaxInterval(64);   // 64 × 0.625ms = 40ms

  // ── Scan response: UUID repetido para garantizar detección ─
  // El cliente usa setActiveScan(true), por lo que recibirá este paquete.
  NimBLEAdvertisementData scanData;
  scanData.setName(BLE_DEVICE_NAME);    // Nombre en el scan response
  scanData.addServiceUUID(SERVICE_UUID); // UUID también aquí
  pAdv->setScanResponseData(scanData);

  NimBLEDevice::startAdvertising();

  Serial.printf("[BLE] Servidor NimBLE iniciado: %s\n", BLE_DEVICE_NAME);
  Serial.printf("[BLE] Service UUID: %s\n", SERVICE_UUID);
}

// ─── INICIALIZAR CÁMARA ──────────────────────────────────────
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;       // 10 MHz — estable con PSRAM
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_96X96;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    Serial.println("[CAM] PSRAM disponible — frame buffer en PSRAM");
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    Serial.println("[CAM] Sin PSRAM — frame buffer en DRAM interna");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Error de inicialización: 0x%x\n", err);
    return false;
  }

  // Ajustes del sensor OV2640
  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s,  1);   // +1 brillo
  s->set_contrast(s,    1);   // +1 contraste
  s->set_saturation(s,  0);   // Saturación normal
  s->set_whitebal(s,    1);   // AWB activado
  s->set_awb_gain(s,    1);
  s->set_exposure_ctrl(s, 1); // AEC activado
  s->set_aec2(s,        1);

  // Descartar los primeros frames (sensor inestable al inicio)
  for (int i = 0; i < 3; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(100);
  }

  Serial.println("[CAM] Cámara inicializada correctamente");
  return true;
}

// ─── CAPTURA + INFERENCIA EDGE IMPULSE ───────────────────────
bool captureAndClassify() {
  Serial.println("[AI] Capturando imagen para inferencia...");

  // Encender flash brevemente para iluminar la muestra de suelo
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(80);   // Dar tiempo al sensor para auto-exposición

  camera_fb_t* fb = esp_camera_fb_get();

  digitalWrite(FLASH_LED_PIN, LOW);   // Apagar flash inmediatamente

  if (!fb) {
    Serial.println("[AI] Error: no se pudo capturar frame");
    return false;
  }

  size_t expectedSize = (size_t)EI_CLASSIFIER_INPUT_WIDTH *
                        (size_t)EI_CLASSIFIER_INPUT_HEIGHT;

  if (fb->len < expectedSize) {
    Serial.printf("[AI] Frame insuficiente: %u bytes, esperados: %u\n",
                  (unsigned)fb->len, (unsigned)expectedSize);
    esp_camera_fb_return(fb);
    return false;
  }

  // Alocar buffer de features — PSRAM si está disponible
  float* features = psramFound()
    ? (float*)ps_malloc(expectedSize * sizeof(float))
    : (float*)malloc(expectedSize * sizeof(float));

  if (!features) {
    Serial.printf("[AI] Sin memoria para features (%u bytes)\n",
                  (unsigned)(expectedSize * sizeof(float)));
    esp_camera_fb_return(fb);
    return false;
  }

  // Convertir grayscale uint8 → float [0.0 , 1.0]
  for (size_t i = 0; i < expectedSize; i++) {
    features[i] = (float)fb->buf[i] / 255.0f;
  }
  esp_camera_fb_return(fb);   // Liberar ASAP para no retener RAM de cámara

  // Crear señal para el clasificador EI
  signal_t signal;
  numpy::signal_from_buffer(features, expectedSize, &signal);

  // Ejecutar inferencia
  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR ei_err = run_classifier(&signal, &result, false);
  free(features);

  if (ei_err != EI_IMPULSE_OK) {
    Serial.printf("[AI] Error en inferencia: %d\n", ei_err);
    return false;
  }

  // Buscar la etiqueta con mayor confianza
  topConfidence = 0.0f;
  uint8_t bestIdx = 0;
  Serial.println("[AI] Resultados de clasificación:");
  for (uint8_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.printf("[AI]   %-20s %.1f%%\n",
      result.classification[i].label,
      result.classification[i].value * 100.0f);
    if (result.classification[i].value > topConfidence) {
      topConfidence = result.classification[i].value;
      bestIdx = i;
    }
  }

  strncpy(soilTypeResult,
          result.classification[bestIdx].label,
          MAX_LABEL_LEN - 1);
  soilTypeResult[MAX_LABEL_LEN - 1] = '\0';

  Serial.printf("[AI] Resultado final: %s (%.1f%%)\n",
    soilTypeResult, topConfidence * 100.0f);

  // Publicar por BLE: "TIPO_SUELO|CONFIANZA"
  // El cliente parsea en busca del separador '|'
  char payload[64];
  snprintf(payload, sizeof(payload), "%s|%.2f", soilTypeResult, topConfidence);

  pCharacteristic->setValue(payload);

  if (deviceConnected) {
    pCharacteristic->notify();
    Serial.printf("[BLE] Notificación enviada: %s\n", payload);
  } else {
    Serial.println("[BLE] Sin cliente conectado — dato guardado para lectura READ");
  }

  return true;
}

// ─── SETUP ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[SYS] ESP32-CAM — Análisis de Suelo v2");

  // GPIO
  pinMode(LED_STATUS_PIN,    OUTPUT);
  pinMode(FLASH_LED_PIN,     OUTPUT);
  pinMode(BTN_ENCENDIDO_PIN, INPUT_PULLUP);
  pinMode(BTN_CAPTURA_PIN,   INPUT_PULLUP);
  digitalWrite(LED_STATUS_PIN, LOW);
  digitalWrite(FLASH_LED_PIN,  LOW);   // Flash SIEMPRE apagado por defecto

  attachInterrupt(BTN_ENCENDIDO_PIN, isr_encendido, FALLING);
  attachInterrupt(BTN_CAPTURA_PIN,   isr_captura,   FALLING);

  // Diagnóstico de RAM
  Serial.printf("[MEM] RAM libre inicial: %lu bytes\n",
                (unsigned long)ESP.getFreeHeap());
  if (psramFound()) {
    Serial.printf("[MEM] PSRAM libre: %lu bytes\n",
                  (unsigned long)ESP.getFreePsram());
  } else {
    Serial.println("[MEM] ADVERTENCIA: PSRAM NO detectada");
    Serial.println("[MEM] El modelo EI puede no caber en RAM interna");
  }

  // Inicializar cámara ANTES de BLE
  // (esp_camera_init usa el controlador I2C interno; hacerlo antes
  //  de NimBLE evita conflictos con el scheduler de FreeRTOS)
  if (!initCamera()) {
    Serial.println("[ERR] Fallo crítico en cámara. Reiniciando en 5 s...");
    delay(5000);
    ESP.restart();
  }

  // Inicializar BLE — SIN btStop() antes
  // NimBLE solo usa BLE (no BT clásico) y gestiona el controlador internamente.
  // Llamar btStop() antes destruye el contexto que NimBLE necesita.
  initBLE();

  Serial.printf("[MEM] RAM libre post-init: %lu bytes\n",
                (unsigned long)ESP.getFreeHeap());
  Serial.println("[SYS] Listo. Presione ENCENDIDO para activar.");
}

// ─── LOOP ────────────────────────────────────────────────────
void loop() {
  static uint32_t lastBtnTime  = 0;
  static uint32_t ledBlinkTime = 0;
  static bool     ledState     = false;
  uint32_t now = millis();

  // ── Parpadeo del LED de estado ────────────────────────────
  // Rápido (300 ms) = esperando cliente BLE
  // Lento (1000 ms) = cliente conectado y sistema activo
  // Apagado         = sistema inactivo
  if (sistemaActivo) {
    uint32_t interval = deviceConnected ? 1000 : 300;
    if (now - ledBlinkTime > interval) {
      ledBlinkTime = now;
      ledState     = !ledState;
      digitalWrite(LED_STATUS_PIN, ledState ? HIGH : LOW);
    }
  }

  // ── Botón ENCENDIDO ───────────────────────────────────────
  if (btnEncendidoFlag && (now - lastBtnTime > 300)) {
    btnEncendidoFlag = false;
    lastBtnTime      = now;
    sistemaActivo    = !sistemaActivo;

    if (sistemaActivo) {
      Serial.println("[SYS] Sistema ACTIVO");
      pCharacteristic->setValue("SISTEMA_ON");
      if (deviceConnected) pCharacteristic->notify();
    } else {
      Serial.println("[SYS] Sistema INACTIVO");
      pCharacteristic->setValue("SISTEMA_OFF");
      if (deviceConnected) pCharacteristic->notify();
      // Apagar LED y flash al desactivar
      digitalWrite(LED_STATUS_PIN, LOW);
      digitalWrite(FLASH_LED_PIN,  LOW);
      ledState = false;
    }
  }

  // ── Botón CAPTURA (solo si sistema activo) ────────────────
  if (btnCapturaFlag && sistemaActivo && (now - lastBtnTime > 300)) {
    btnCapturaFlag = false;
    lastBtnTime    = now;

    Serial.println("[SYS] Captura solicitada");

    // Notificar al cliente que estamos capturando
    pCharacteristic->setValue("CAPTURANDO");
    if (deviceConnected) pCharacteristic->notify();

    delay(200);   // Pequeña pausa para que el cliente reciba la notificación

    bool ok = captureAndClassify();

    if (!ok) {
      Serial.println("[SYS] Error en clasificación");
      pCharacteristic->setValue("ERROR_CAM");
      if (deviceConnected) pCharacteristic->notify();
    }
    // El resultado ya se notificó dentro de captureAndClassify()
  }

  // Limpiar flag de captura si sistema inactivo
  if (!sistemaActivo) {
    btnCapturaFlag = false;
    digitalWrite(FLASH_LED_PIN, LOW);   // Seguridad: flash siempre apagado si inactivo
  }

  delay(10);
}
