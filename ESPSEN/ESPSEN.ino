/*
 * ============================================================
 *  SISTEMA DE ANÁLISIS DE SUELO — ESP32 SENSORES (CLIENTE BLE)
 *  Versión corregida — Mayo 2025
 * ============================================================
 *  Librerías requeridas (Arduino IDE → Manage Libraries):
 *   - NimBLE-Arduino   (h2zero)          v1.4.x o v2.x
 *   - Adafruit SH110X (Adafruit)
 *   - Adafruit GFX Library (Adafruit)
 *   - ArduinoJson      (Benoit Blanchon)
 *  Incluidas en ESP32 Core (no instalar aparte):
 *   - mbedTLS (AES + Base64), WiFi, HTTPClient, WiFiClientSecure
 * ============================================================
 */

// ─── LIBRERÍAS ───────────────────────────────────────────────
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "mbedtls/aes.h"
#include "mbedtls/base64.h"

// ─── CONFIGURACIÓN WiFi ──────────────────────────────────────
#define WIFI_SSID        "Edison"
#define WIFI_PASSWORD    "012345678"
#define WIFI_TIMEOUT_MS  15000

// ─── ENDPOINT SERVIDOR ───────────────────────────────────────
#define SERVER_URL  "http://10.40.160.34:5000/api/soil"
#define API_KEY     "suelos2026"

// ─── CLAVE AES-128 (exactamente 16 bytes) ────────────────────
// "SueloIoT2024Key!"
static const uint8_t AES_KEY[16] = {
  0x53,0x75,0x65,0x6C,0x6F,0x49,0x6F,0x54,
  0x32,0x30,0x32,0x34,0x4B,0x65,0x79,0x21
};
// "IVSuelo012345678"
static const uint8_t AES_IV[16] = {
  0x49,0x56,0x53,0x75,0x65,0x6C,0x6F,0x30,
  0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38
};

// ─── OLED ────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDR     0x3C
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire, -1);

// ─── PINES ───────────────────────────────────────────────────
// ADC1 únicamente (ADC2 no funciona con WiFi activo)
#define PIN_PH        34    // pH4502C  → GPIO34 (ADC1_CH6)
#define PIN_HUMEDAD   32    // YL-69    → GPIO35 (ADC1_CH7)

// LEDs
#define LED_VERDE     25
#define LED_AZUL1     33
#define LED_AZUL      26
#define LED_AMARILLO  27
#define LED_ROJO      14

// Botones
#define BTN_ENCENDIDO  4
#define BTN_CAPTURA   13
#define BTN_ENVIAR    15

// I2C
#define SDA_PIN 21
#define SCL_PIN 22

// ─── BLE UUIDs ───────────────────────────────────────────────
// Deben coincidir EXACTAMENTE con los del ESP32-CAM servidor
#define CAM_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CAM_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ─── CALIBRACIÓN pH4502C ─────────────────────────────────────
#define PH_ADC_AT_7   3100    
#define PH_SLOPE      198.0f  
#define PH_SAMPLES    20

// ─── CALIBRACIÓN YL-69 ───────────────────────────────────────
#define HUM_ADC_DRY   3500
#define HUM_ADC_WET    500
#define HUM_SAMPLES    10

// ─── STRUCT DATOS ────────────────────────────────────────────
struct SoilData {
  char  soilType[64];
  float ph;
  float humidity;
  char  timestamp[24];
  bool  valid;
} soilData = {"Sin_Datos", 0.0f, 0.0f, "", false};

// ─── ESTADO GLOBAL ───────────────────────────────────────────
bool sistemaActivo    = false;
bool datosCaptured    = false;
bool bleConnected     = false;
bool waitingCamResult = false;
String camResult      = "";

// ─── NimBLE — punteros globales ──────────────────────────────
NimBLEClient*              pClient     = nullptr;
NimBLERemoteCharacteristic* pRemoteChar = nullptr;

// ─── ISR FLAGS ───────────────────────────────────────────────
volatile bool btnEncendidoFlag = false;
volatile bool btnCapturaFlag   = false;
volatile bool btnEnviarFlag    = false;

void IRAM_ATTR isr_enc() { btnEncendidoFlag = true; }
void IRAM_ATTR isr_cap() { btnCapturaFlag   = true; }
void IRAM_ATTR isr_env() { btnEnviarFlag    = true; }

// ════════════════════════════════════════════════════════════
//  ADC
// ════════════════════════════════════════════════════════════
void initADC() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(PIN_PH,      INPUT);
  pinMode(PIN_HUMEDAD, INPUT);
  // Descartar primeras lecturas (warm-up)
  for (int i = 0; i < 5; i++) {
    analogRead(PIN_PH);
    analogRead(PIN_HUMEDAD);
    delay(10);
  }
}

float readPH() {
  uint32_t sum = 0;
  for (int i = 0; i < PH_SAMPLES; i++) {
    sum += analogRead(PIN_PH);
    delay(5);
  }
  float raw = (float)(sum / PH_SAMPLES);
  float ph  = 7.0f + ((float)PH_ADC_AT_7 - raw) / PH_SLOPE;
  ph = constrain(ph, 0.0f, 14.0f);
  Serial.printf("[PH]  Raw:%.0f  pH:%.2f\n", raw, ph);
  return ph;
}

float readHumidity() {
  uint32_t sum = 0;
  for (int i = 0; i < HUM_SAMPLES; i++) {
    sum += analogRead(PIN_HUMEDAD);
    delay(10);
  }
  float raw = (float)(sum / HUM_SAMPLES);
  float hum = ((float)HUM_ADC_DRY - raw) /
              ((float)HUM_ADC_DRY - (float)HUM_ADC_WET) * 100.0f;
  hum = constrain(hum, 0.0f, 100.0f);
  Serial.printf("[HUM] Raw:%.0f  Hum:%.1f%%\n", raw, hum);
  return hum;
}

// ─── CLASIFICACIÓN ───────────────────────────────────────────
void classifySoilComplete(const char* camType, float ph, float hum,
                           char* output, size_t outLen) {
  char texture[20]   = "";
  char fertility[32] = "";

  if      (hum > 75.0f)                   strncpy(texture, "lodosa",      19);
  else if (hum < 20.0f)                   strncpy(texture, "arenosa",     19);
  else if (hum >= 40.0f && hum <= 65.0f)  strncpy(texture, "humeda",      19);
  else                                    strncpy(texture, "semi-humeda", 19);

  if      (ph >= 5.5f && ph <= 7.5f && hum >= 25.0f && hum <= 80.0f)
                                          strncpy(fertility, "posiblemente fertil", 31);
  else if (ph < 5.5f)                    strncpy(fertility, "acida",               31);
  else if (ph > 7.5f)                    strncpy(fertility, "alcalina",            31);
  else                                   strncpy(fertility, "condicion atipica",   31);

  snprintf(output, outLen, "%s %s %s", camType, texture, fertility);
  if (outLen > 0) output[0] = toupper((unsigned char)output[0]);
}

// ════════════════════════════════════════════════════════════
//  AES-128 CBC + Base64
// ════════════════════════════════════════════════════════════
int aesEncryptToBase64(const char* plaintext, char* outputBuf, size_t outputLen) {
  size_t ptLen  = strlen(plaintext);
  size_t padded = ((ptLen / 16) + 1) * 16;

  if (padded > 512) {
    Serial.println("[AES] Payload demasiado largo");
    return -1;
  }

  uint8_t input[512]  = {0};
  uint8_t output[512] = {0};
  uint8_t iv[16];

  memcpy(input, plaintext, ptLen);
  uint8_t padByte = (uint8_t)(padded - ptLen);
  for (size_t i = ptLen; i < padded; i++) input[i] = padByte;
  memcpy(iv, AES_IV, 16);

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  int ret = mbedtls_aes_setkey_enc(&aes, AES_KEY, 128);
  if (ret != 0) { mbedtls_aes_free(&aes); return -1; }
  ret = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, padded, iv, input, output);
  mbedtls_aes_free(&aes);
  if (ret != 0) return -1;

  size_t b64Len = 0;
  ret = mbedtls_base64_encode(
    (unsigned char*)outputBuf, outputLen, &b64Len, output, padded);
  if (ret != 0) return -1;
  outputBuf[b64Len] = '\0';
  return (int)b64Len;
}

// ─── OLED ────────────────────────────────────────────────────
void oledShowBoot() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(8,  4); display.println("ANALIZADOR DE SUELO");
  display.setCursor(18,18); display.println("Ing. Electronica");
  display.setCursor(10,32); display.println("Presione ENCENDIDO");
  display.drawRect(0,0,128,64,SH110X_WHITE);
  display.display();
}

void oledMsg(const char* l1, const char* l2 = "",
             const char* l3 = "", const char* l4 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,  0); display.println(l1);
  display.setCursor(0, 16); display.println(l2);
  display.setCursor(0, 32); display.println(l3);
  display.setCursor(0, 48); display.println(l4);
  display.display();
}

void oledShowResults() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  char l1[22] = {0}, l2[22] = {0};
  size_t len = strlen(soilData.soilType);
  strncpy(l1, soilData.soilType, 21);
  if (len > 21) strncpy(l2, soilData.soilType + 21, 21);

  display.setCursor(0,  0); display.print(l1);
  display.setCursor(0, 12); display.print(l2);
  display.drawFastHLine(0, 26, 128, SH110X_WHITE);

  char phStr[16], humStr[16];
  snprintf(phStr,  sizeof(phStr),  "pH:%.2f",  soilData.ph);
  snprintf(humStr, sizeof(humStr), "H:%.1f%%", soilData.humidity);
  display.setCursor( 0, 32); display.print(phStr);
  display.setCursor(68, 32); display.print(humStr);
  display.setCursor( 0, 48);
  display.print(bleConnected ? "BLE:OK " : "BLE:-- ");
  display.print(datosCaptured ? "LISTO" : "---");
  display.display();
}

// ─── TIMESTAMP NTP ───────────────────────────────────────────
void getTimestamp(char* buf, size_t len) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 3000)) {
    snprintf(buf, len, "1970-01-01T00:00:00");
    return;
  }
  strftime(buf, len, "%Y-%m-%dT%H:%M:%S", &timeinfo);
}

// ─── ENVÍO WiFi ───────────────────────────────────────────────
bool sendToDatabase() {
  oledMsg("Conectando WiFi...", WIFI_SSID);
  digitalWrite(LED_AMARILLO, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > WIFI_TIMEOUT_MS) {
      Serial.println("[WIFI] Timeout");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      digitalWrite(LED_AMARILLO, LOW);
      return false;
    }
    delay(250);
  }
  Serial.printf("[WIFI] IP: %s\n", WiFi.localIP().toString().c_str());

  configTime(-5 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  delay(1200);
  getTimestamp(soilData.timestamp, sizeof(soilData.timestamp));

  char plaintext[256];
  snprintf(plaintext, sizeof(plaintext),
    "{\"soil_type\":\"%s\",\"ph\":%.2f,\"humidity\":%.1f,\"timestamp\":\"%s\"}",
    soilData.soilType, soilData.ph, soilData.humidity, soilData.timestamp);
  Serial.printf("[JSON] %s\n", plaintext);

  char encrypted[512];
  int encLen = aesEncryptToBase64(plaintext, encrypted, sizeof(encrypted));
  if (encLen < 0) {
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
    digitalWrite(LED_AMARILLO, LOW);
    return false;
  }

  char payload[640];
  snprintf(payload, sizeof(payload),
    "{\"data\":\"%s\",\"iv\":\"IVSuelo012345678\",\"api_key\":\"%s\"}",
    encrypted, API_KEY);

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Key", API_KEY);
  http.setTimeout(10000);

  int code = http.POST(payload);
  bool ok  = (code == 200 || code == 201);
  Serial.printf("[HTTP] Código: %d\n", code);

  http.end();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  digitalWrite(LED_AMARILLO, LOW);
  return ok;
}

// ════════════════════════════════════════════════════════════
//  NimBLE — Callbacks
// ════════════════════════════════════════════════════════════

// Notificación BLE recibida desde la CAM
void notifyCallback(NimBLERemoteCharacteristic* pChar,
                    uint8_t* pData, size_t length, bool isNotify) {
  char buf[128] = {0};
  size_t n = (length < sizeof(buf) - 1) ? length : sizeof(buf) - 1;
  memcpy(buf, pData, n);
  camResult        = String(buf);
  waitingCamResult = false;
  Serial.printf("[BLE] Notificacion recibida: %s\n", buf);
}

// ── Callbacks de scan (NimBLE v2.x usa NimBLEScanCallbacks) ──
class MyScanCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice* device) override {
    Serial.printf("[BLE] Dispositivo: %s  RSSI:%d\n",
                  device->getAddress().toString().c_str(),
                  device->getRSSI());

    if (device->isAdvertisingService(NimBLEUUID(CAM_SERVICE_UUID))) {
      Serial.println("[BLE] >>> ESP32-CAM encontrada — deteniendo scan");
      NimBLEDevice::getScan()->stop();
      _foundAddr   = device->getAddress();
      _found       = true;
      _connecting  = false;   // se activará en loop()
    }
  }

  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    Serial.printf("[BLE] Scan terminado. Dispositivos encontrados: %d\n",
                  results.getCount());
  }

  bool          _found      = false;
  bool          _connecting = false;
  NimBLEAddress _foundAddr;
};

MyScanCallbacks* scanCallbacks = nullptr;

// ─── CONEXIÓN ────────────────────────────────────────────────
bool connectToCam(const NimBLEAddress& addr) {
  if (pClient) {
    if (pClient->isConnected()) pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    pClient     = nullptr;
    pRemoteChar = nullptr;
  }

  delay(300);

  pClient = NimBLEDevice::createClient(addr);
  pClient->setConnectionParams(16, 32, 0, 400);
  pClient->setConnectTimeout(15);

  Serial.printf("[BLE] Conectando a %s ...\n", addr.toString().c_str());

  // ── Reintentos ───────────────────────────────────────────
  bool conectado = false;
  for (int intento = 1; intento <= 3; intento++) {
    Serial.printf("[BLE] Intento %d/3...\n", intento);
    if (pClient->connect()) {
      conectado = true;
      Serial.println("[BLE] Conectado al servidor");
      break;
    }
    if (intento < 3) delay(1000);
  }

  if (!conectado) {
    Serial.println("[BLE] connect() falló tras 3 intentos");
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
    return false;
  }

  // ── Pausa para estabilizar antes de descubrir servicios ──
  delay(500);

  NimBLERemoteService* svc = pClient->getService(CAM_SERVICE_UUID);
  if (!svc) {
    Serial.println("[BLE] Servicio no encontrado en el servidor");
    pClient->disconnect();
    return false;
  }

  pRemoteChar = svc->getCharacteristic(CAM_CHARACTERISTIC_UUID);
  if (!pRemoteChar) {
    Serial.println("[BLE] Característica no encontrada");
    pClient->disconnect();
    return false;
  }

  if (pRemoteChar->canNotify()) {
    delay(200);
    bool ok = pRemoteChar->subscribe(true, notifyCallback);
    Serial.printf("[BLE] Subscribe: %s\n", ok ? "OK" : "FALLO");
    if (!ok) {
      pClient->disconnect();
      return false;
    }
  }

  bleConnected = true;
  return true;
}
// ─── INICIAR SCAN ────────────────────────────────────────────
void startBLEScan() {
  if (!scanCallbacks) scanCallbacks = new MyScanCallbacks();
  scanCallbacks->_found      = false;
  scanCallbacks->_connecting = false;

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(scanCallbacks);
  pScan->setActiveScan(true);    // ACTIVO: recibe scan response con UUID 128 bits
  pScan->setInterval(100);
  pScan->setWindow(99);          // Window ≈ Interval → scan continuo
  pScan->clearResults();         // Limpiar caché de resultados anteriores
  pScan->start(8, false);        // 8 s, no bloquear
  Serial.println("[BLE] Scan activo iniciado (8 s)...");
}

// ─── SETUP ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[SYS] SueloSmart — ESP32 Sensores v2");

  // LEDs
  pinMode(LED_VERDE,    OUTPUT); digitalWrite(LED_VERDE,    LOW);
  pinMode(LED_AZUL1,     OUTPUT); digitalWrite(LED_AZUL1,     LOW);
  pinMode(LED_AZUL,     OUTPUT); digitalWrite(LED_AZUL,     LOW);
  pinMode(LED_AMARILLO, OUTPUT); digitalWrite(LED_AMARILLO, LOW);
  pinMode(LED_ROJO,     OUTPUT); digitalWrite(LED_ROJO,     LOW);

  // Botones con pull-up interno
  pinMode(BTN_ENCENDIDO, INPUT_PULLUP);
  pinMode(BTN_CAPTURA,   INPUT_PULLUP);
  pinMode(BTN_ENVIAR,    INPUT_PULLUP);
  attachInterrupt(BTN_ENCENDIDO, isr_enc, FALLING);
  attachInterrupt(BTN_CAPTURA,   isr_cap, FALLING);
  attachInterrupt(BTN_ENVIAR,    isr_env, FALLING);

  // I2C + OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("[ERR] OLED no detectada — verificar cableado I2C");
  } else {
    oledShowBoot();
  }

  // ADC
  initADC();

  // WiFi apagado al inicio
  WiFi.mode(WIFI_OFF);

  // NimBLE init
  NimBLEDevice::init("SueloSensor_SEN");
  NimBLEDevice::setMTU(128);
  NimBLEDevice::setPower(ESP_PWR_LVL_N0);

  Serial.printf("[MEM] RAM libre: %d bytes\n", ESP.getFreeHeap());
  Serial.println("[SYS] Listo — presione ENCENDIDO");
}

// ─── LOOP ────────────────────────────────────────────────────
void loop() {
  static uint32_t lastBtn  = 0;
  static uint32_t bleTimer = 0;
  uint32_t now = millis();

  // ── CONEXIÓN: activada desde loop, nunca desde callback ──
  if (sistemaActivo
      && scanCallbacks
      && scanCallbacks->_found
      && !scanCallbacks->_connecting) {

    scanCallbacks->_connecting = true;   // Evitar re-entrada
    scanCallbacks->_found      = false;

    oledMsg("CAM encontrada", "Conectando BLE...");
    Serial.println("[SYS] Iniciando conexion BLE...");

    if (connectToCam(scanCallbacks->_foundAddr)) {
      oledMsg("BLE Conectado!", "ESP32-CAM OK",
              "Presione CAPTURA", "para analizar");
      digitalWrite(LED_AZUL1, HIGH);
    } else {
      oledMsg("BLE: fallo", "Reintentando en 20s");
      bleConnected = false;
      bleTimer     = now - 15000;   // Reintentar en 5 s
    }
    scanCallbacks->_connecting = false;
  }

  // ── Botón ENCENDIDO ───────────────────────────────────────
  if (btnEncendidoFlag && (now - lastBtn > 300)) {
    btnEncendidoFlag = false;
    lastBtn          = now;
    sistemaActivo    = !sistemaActivo;

    if (sistemaActivo) {
      digitalWrite(LED_VERDE, HIGH);
      oledMsg("Sistema ACTIVO", "Buscando CAM...");
      Serial.println("[SYS] Sistema ACTIVO");
      bleTimer = 0;   // Forzar scan inmediato
      startBLEScan();
    } else {
      sistemaActivo = false;
      datosCaptured = false;
      bleConnected  = false;
      NimBLEDevice::getScan()->stop();
      if (pClient && pClient->isConnected()) pClient->disconnect();
      if (pClient) {
        NimBLEDevice::deleteClient(pClient);
        pClient     = nullptr;
        pRemoteChar = nullptr;
      }
      digitalWrite(LED_VERDE,    LOW);
      digitalWrite(LED_AZUL,     LOW);
      digitalWrite(LED_AZUL1,    LOW);
      digitalWrite(LED_AMARILLO, LOW);
      digitalWrite(LED_ROJO,     LOW);
      WiFi.mode(WIFI_OFF);
      oledShowBoot();
      Serial.println("[SYS] Sistema INACTIVO");
    }
  }

  // ── Botón CAPTURA ─────────────────────────────────────────
  if (btnCapturaFlag && sistemaActivo && (now - lastBtn > 300)) {
    btnCapturaFlag = false;
    lastBtn        = now;

    digitalWrite(LED_AZUL, HIGH);
    oledMsg("Tomando muestras...", "Por favor espere");
    Serial.println("[SYS] Captura iniciada");

    // 1. Sensores físicos
    soilData.ph       = readPH();
    soilData.humidity = readHumidity();
    soilData.valid    = true;

    // 2. Resultado visual de la CAM por BLE
    String tipoVisual = "Suelo";

    if (bleConnected && pClient && pClient->isConnected() && pRemoteChar) {
      waitingCamResult = true;
      camResult        = "";
      oledMsg("Esperando foto CAM...", "Presione btn CAM");

      uint32_t t0 = millis();
      while (waitingCamResult && (millis() - t0 < 10000)) delay(50);

      if (!camResult.isEmpty()
          && camResult != "ESPERANDO"
          && camResult != "SISTEMA_ON"
          && camResult != "SISTEMA_OFF"
          && camResult != "CAPTURANDO"
          && camResult != "ERROR_CAM") {
        int sep = camResult.indexOf('|');
        tipoVisual = (sep > 0) ? camResult.substring(0, sep) : camResult;
        tipoVisual.replace('_', ' ');
        Serial.printf("[BLE] Tipo visual CAM: %s\n", tipoVisual.c_str());
      } else {
        Serial.println("[BLE] Sin resultado útil de CAM — clasificando solo sensores");
        tipoVisual = "Suelo";
      }
    } else {
      oledMsg("Sin BLE con CAM", "Solo sensores...");
      delay(1200);

      // Si perdimos conexión BLE, marcarla como inactiva
      if (bleConnected && pClient && !pClient->isConnected()) {
        bleConnected = false;
        digitalWrite(LED_AZUL1, LOW);
        Serial.println("[BLE] Conexion perdida durante captura");
      }
    }

    char tipoBuf[32];
    tipoVisual.toCharArray(tipoBuf, sizeof(tipoBuf));
    classifySoilComplete(tipoBuf, soilData.ph, soilData.humidity,
                         soilData.soilType, sizeof(soilData.soilType));

    datosCaptured = true;
    digitalWrite(LED_AZUL1, bleConnected ? HIGH : LOW);
    oledShowResults();
    Serial.printf("[SYS] Resultado: %s | pH:%.2f | Hum:%.1f%%\n",
                  soilData.soilType, soilData.ph, soilData.humidity);
  }

  // ── Botón ENVIAR ──────────────────────────────────────────
  if (btnEnviarFlag && sistemaActivo && (now - lastBtn > 300)) {
    btnEnviarFlag = false;
    lastBtn       = now;

    if (!datosCaptured) {
      oledMsg("Sin datos!", "Capture primero");
      delay(2000);
      oledShowResults();
      return;
    }

    bool ok = sendToDatabase();

    if (ok) {
      Serial.println("[SYS] Envio exitoso");
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_AMARILLO, HIGH); delay(200);
        digitalWrite(LED_AMARILLO, LOW);  delay(200);
      }
      oledMsg("Enviado OK!", soilData.timestamp, soilData.soilType);
    } else {
      Serial.println("[SYS] Error de envio");
      digitalWrite(LED_ROJO, HIGH);
      oledMsg("ERROR al enviar", "Verificar WiFi", "o servidor");
      delay(3000);
      digitalWrite(LED_ROJO, LOW);
      oledShowResults();
    }
  }

  // ── Re-escaneo BLE automático cada 20 s si no conectado ──
  // Bloqueado si ya hay un _found pendiente o estamos conectando
  if (sistemaActivo
      && !bleConnected
      && scanCallbacks
      && !scanCallbacks->_found
      && !scanCallbacks->_connecting
      && (now - bleTimer > 20000)) {
    bleTimer = now;
    Serial.println("[BLE] Re-escaneo automático");
    startBLEScan();
  }

  delay(10);
}
