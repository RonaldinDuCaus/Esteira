#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <time.h>
#include "root_ca.h"

// O Wi-Fi é configurado pelo portal local da própria ESP32.
const char *API_BASE_URL = "https://ai.kronux.com.br";
const char *DEVICE_ID = "esp32-esteira-01";
const char *CAMERA_DEVICE_ID = "esp32-cam-01";
const char *DEVICE_TOKEN = "token site";
const char *CONFIG_PORTAL_SSID = "Fabrica20";
const char *CONFIG_PORTAL_PASSWORD = "@208862Sfc";

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define LCD_SDA 21
#define LCD_SCL 22
#define MOTOR_IN1 33
#define MOTOR_IN2 32
#define MOTOR_ENA 25
#define TRIG 27
#define ECHO 26

const unsigned long WIFI_RETRY_MS = 5000;
const unsigned long EVENT_RETRY_MS = 2000;
const unsigned long RESULT_POLL_MS = 1000;
const unsigned long CONFIG_POLL_MS = 5000;
const unsigned long CONFIG_LEASE_MS = 15000;
const unsigned long HEARTBEAT_MS = 5000;
const unsigned long RESULT_DISPLAY_MS = 3000;
const unsigned long SENSOR_READ_MS = 100;
const unsigned long SENSOR_SAFETY_READ_MS = 50;
const unsigned long CLOCK_RETRY_MS = 30000;
const unsigned long RELEASE_MAX_MS = 3000;
const unsigned long CONFIG_RESPONSE_MAX_MS = 10000;
const unsigned long PORTAL_TIMEOUT_SECONDS = 120;
const unsigned long NETWORK_TIMEOUT_MS = 3000;
const unsigned long TLS_TIMEOUT_SECONDS = 5;
const float CLEAR_MARGIN_CM = 1.0f;

int motorSpeed = 255;
volatile float detectionLimitCm = 10.0;
int requiredReadings = 3;
int minimumStopSeconds = 5;
int resultTimeoutSeconds = 20;
int releaseConfidencePercent = 86;
int appliedConfigVersion = 0;
volatile bool remoteRunEnabled = false;
bool needsReview = false;
bool releaseBlocked = false;
float lastConfidencePercent = -1.0f;
bool runPreferencesReady = false;
String lastRunToken;
String activeRunToken;
String rejectedRunToken;
String bootId;
volatile unsigned long lastConfigSuccessAt = 0;
volatile uint32_t authorizationGeneration = 0;
volatile bool safetyStopPending = false;
volatile bool clearanceTimeoutPending = false;
volatile bool discardNextRunCommand = false;
Preferences runPreferences;

enum ConveyorState {
  RUNNING,
  SENDING_EVENT,
  WAITING_RESULT,
  SHOWING_RESULT,
  WAITING_OBJECT_CLEAR,
  CLEARING_OBJECT
};

volatile ConveyorState state = RUNNING;
String clientEventId;
String serverEventId;
String lastResult;
unsigned long localCounter = 0;
unsigned long stateStartedAt = 0;
unsigned long detectionStartedAt = 0;
volatile unsigned long releaseStartedAt = 0;
unsigned long lastWiFiAttemptAt = 0;
unsigned long lastEventAttemptAt = 0;
unsigned long lastResultPollAt = 0;
unsigned long lastConfigAt = 0;
unsigned long lastHeartbeatAt = 0;
unsigned long lastSensorAt = 0;
unsigned long lastClockRequestAt = 0;
unsigned long wifiDisconnectedAt = 0;
float lastDistanceCm = -1;
int detectionReadings = 0;
int clearReadings = 0;
WiFiManager wifiManager;
bool clockSynchronized = false;
bool clockWarningShown = false;
bool sensorErrorShown = false;
bool resultTimeoutShown = false;
volatile bool motorRunning = false;
bool sensorTaskReady = false;
bool motorPwmReady = false;
float sampledDistanceCm = -1;
unsigned long sampledAt = 0;
portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;

bool beginHttp(HTTPClient &http, WiFiClient &plainClient,
               WiFiClientSecure &secureClient, const String &url) {
  (void)plainClient;
  if (!url.startsWith("https://") || !clockSynchronized ||
      WiFi.status() != WL_CONNECTED) return false;
  secureClient.setCACert(ROOT_CA_CERT);
  secureClient.setHandshakeTimeout(TLS_TIMEOUT_SECONDS);
  http.setConnectTimeout(NETWORK_TIMEOUT_MS);
  http.setTimeout(NETWORK_TIMEOUT_MS);
  return http.begin(secureClient, url);
}

void addDeviceHeaders(HTTPClient &http) {
  http.addHeader("Authorization", "Bearer " + String(DEVICE_TOKEN));
  http.addHeader("X-Device-ID", DEVICE_ID);
}

String fitLcd(String value) {
  value.trim();
  if (value.length() > 16) value = value.substring(0, 16);
  while (value.length() < 16) value += " ";
  return value;
}

void showLcd(const String &line1, const String &line2) {
  lcd.setCursor(0, 0);
  lcd.print(fitLcd(line1));
  lcd.setCursor(0, 1);
  lcd.print(fitLcd(line2));
}

void stopMotor() {
  ledcWrite(MOTOR_ENA, 0);
  motorRunning = false;
}

// Chamado também pelo sensor/evento Wi-Fi: apenas dados simples, sem String/NVS/LCD.
// A geração impede que uma resposta HTTP iniciada antes da falha rearme a placa.
void latchSafetyStop(bool clearanceTimedOut = false) {
  portENTER_CRITICAL(&sensorMux);
  remoteRunEnabled = false;
  safetyStopPending = true;
  if (!clearanceTimedOut) discardNextRunCommand = true;
  if (clearanceTimedOut) clearanceTimeoutPending = true;
  authorizationGeneration++;
  portEXIT_CRITICAL(&sensorMux);
  stopMotor();
}

void serviceSafetyStops() {
  portENTER_CRITICAL(&sensorMux);
  const bool pending = safetyStopPending;
  const bool clearanceTimedOut = clearanceTimeoutPending;
  safetyStopPending = false;
  clearanceTimeoutPending = false;
  portEXIT_CRITICAL(&sensorMux);
  if (!pending) return;
  activeRunToken = "";
  stopMotor();
  if (state == CLEARING_OBJECT) state = WAITING_OBJECT_CLEAR;
  if (clearanceTimedOut) {
    releaseBlocked = true;
    showLcd("PECA NA ESTEIRA", "VERIFIQUE SENSOR");
    Serial.println("Saída não confirmada no prazo; libere novamente após conferir a peça.");
  } else {
    showLcd("ESTEIRA PARADA", "LIBERE NO SITE");
    Serial.println("Autorização invalidada por perda de rede ou prazo; novo comando necessário.");
  }
}

bool validRunToken(const String &token) {
  if (token.length() != 32) return false;
  for (size_t index = 0; index < token.length(); index++) {
    const char value = token[index];
    if (!((value >= '0' && value <= '9') || (value >= 'a' && value <= 'f'))) return false;
  }
  return true;
}

bool rememberRunToken(const String &token) {
  if (!runPreferencesReady || !validRunToken(token)) return false;
  if (token == lastRunToken) return true;
  if (runPreferences.putString("last_token", token) != token.length()) {
    Serial.println("Falha ao salvar autorização; motor permanece bloqueado.");
    return false;
  }
  lastRunToken = token;
  return true;
}

void startMotor() {
  portENTER_CRITICAL(&sensorMux);
  const float currentDistance = sampledDistanceCm;
  const unsigned long sampleTime = sampledAt;
  const bool pendingStop = safetyStopPending || clearanceTimeoutPending;
  const uint32_t startGeneration = authorizationGeneration;
  portEXIT_CRITICAL(&sensorMux);
  const bool runningNormally = state == RUNNING && currentDistance >= detectionLimitCm;
  const bool clearingPiece = state == CLEARING_OBJECT && currentDistance > 0 &&
      millis() - releaseStartedAt < RELEASE_MAX_MS;
  if ((!runningNormally && !clearingPiece) || WiFi.status() != WL_CONNECTED ||
      !clockSynchronized || !sensorTaskReady || !motorPwmReady || motorSpeed <= 0 ||
      pendingStop || !sampleTime || millis() - sampleTime > 250 ||
      !remoteRunEnabled || !lastConfigSuccessAt ||
      millis() - lastConfigSuccessAt > CONFIG_LEASE_MS) {
    stopMotor();
    return;
  }
  const bool wasMotorRunning = motorRunning;
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  ledcWrite(MOTOR_ENA, motorSpeed);
  motorRunning = true;
  // Se a tarefa de segurança cortou entre a checagem e o PWM, não reponha energia.
  portENTER_CRITICAL(&sensorMux);
  const bool invalidated = authorizationGeneration != startGeneration || safetyStopPending;
  const float latestDistance = sampledDistanceCm;
  portEXIT_CRITICAL(&sensorMux);
  if (invalidated || !remoteRunEnabled || latestDistance <= 0 ||
      (state == RUNNING && latestDistance < detectionLimitCm) ||
      (state == CLEARING_OBJECT && millis() - releaseStartedAt >= RELEASE_MAX_MS)) {
    stopMotor();
    return;
  }
  if (!wasMotorRunning) {
    if (state == CLEARING_OBJECT) showLcd("LIBERANDO PECA", "MOTOR LIGADO");
    else showLcd("SISTEMA ESTEIRA", "ESTEIRA: LIGADA");
    Serial.println("Motor ligado: condições de operação verificadas.");
  }
}

float measureDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long duration = pulseIn(ECHO, HIGH, 30000);
  if (!duration) return -1;
  return duration * 0.0343f / 2.0f;
}

// Esta tarefa continua lendo o sensor mesmo durante requisições à VPS.
// Ela desliga o motor e invalida autorizações; Strings, NVS e LCD ficam no loop.
void sensorSafetyTask(void *parameter) {
  (void)parameter;
  for (;;) {
    const float distanceCm = measureDistance();
    portENTER_CRITICAL(&sensorMux);
    sampledDistanceCm = distanceCm;
    sampledAt = millis();
    portEXIT_CRITICAL(&sensorMux);
    const unsigned long now = millis();
    const bool releaseTimedOut = state == CLEARING_OBJECT &&
        now - releaseStartedAt >= RELEASE_MAX_MS;
    const bool configExpired = !lastConfigSuccessAt ||
        now - lastConfigSuccessAt > CONFIG_LEASE_MS;
    if ((remoteRunEnabled && (configExpired || WiFi.status() != WL_CONNECTED)) ||
        (releaseTimedOut && !clearanceTimeoutPending)) {
      latchSafetyStop(releaseTimedOut);
    }
    if (distanceCm <= 0 || releaseTimedOut || configExpired || !remoteRunEnabled ||
        WiFi.status() != WL_CONNECTED ||
        (state != CLEARING_OBJECT && distanceCm < detectionLimitCm)) {
      ledcWrite(MOTOR_ENA, 0);
      motorRunning = false;
    }
    vTaskDelay(pdMS_TO_TICKS(SENSOR_SAFETY_READ_MS));
  }
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiDisconnectedAt = 0;
    return;
  }
  stopMotor();
  if (remoteRunEnabled) latchSafetyStop();
  activeRunToken = "";
  if (!wifiDisconnectedAt) {
    wifiDisconnectedAt = millis();
    showLcd("SEM CONEXAO WIFI", "TENTANDO...");
  }
  if (millis() - lastWiFiAttemptAt < WIFI_RETRY_MS) return;
  lastWiFiAttemptAt = millis();
  WiFi.reconnect();

  // Mantém a esteira parada enquanto o portal estiver aberto.
  if (millis() - wifiDisconnectedAt >= 60000UL) {
    showLcd("CONFIGURE WIFI", "192.168.4.1");
    wifiManager.startConfigPortal(CONFIG_PORTAL_SSID, CONFIG_PORTAL_PASSWORD);
    wifiDisconnectedAt = 0;
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.mode(WIFI_STA);
      WiFi.reconnect();
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Wi-Fi reconectado pelo portal.");
      showLcd("WIFI CONECTADO", "VERIFICANDO...");
    }
  }
}

void startWiFiProvisioning() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_ID);
  wifiManager.setConnectTimeout(20);
  wifiManager.setSaveConnectTimeout(20);
  wifiManager.setConfigPortalTimeout(PORTAL_TIMEOUT_SECONDS);
  wifiManager.setAPClientCheck(false);
  wifiManager.setWebPortalClientCheck(false);
  wifiManager.setWiFiAutoReconnect(true);
  wifiManager.setAPCallback([](WiFiManager *) {
    showLcd("REDE: MINI FAB.", "192.168.4.1");
    Serial.println("Conecte em MiniFabrica-Esteira e abra http://192.168.4.1");
  });
  if (!wifiManager.autoConnect(CONFIG_PORTAL_SSID, CONFIG_PORTAL_PASSWORD)) {
    showLcd("SEM CONEXAO WIFI", "TENTANDO...");
    WiFi.mode(WIFI_STA);
    WiFi.reconnect();
    return;
  }
  Serial.print("Wi-Fi conectado. IP: ");
  Serial.println(WiFi.localIP());
}

void updateClock() {
  if (WiFi.status() != WL_CONNECTED) {
    clockSynchronized = false;
    return;
  }
  if (time(nullptr) >= 1700000000) {
    clockSynchronized = true;
    clockWarningShown = false;
    return;
  }
  clockSynchronized = false;
  if (!lastClockRequestAt || millis() - lastClockRequestAt >= CLOCK_RETRY_MS) {
    configTime(0, 0, "pool.ntp.org", "time.cloudflare.com");
    lastClockRequestAt = millis();
    Serial.println("Sincronizando relógio para conexão HTTPS.");
  }
  if (!clockWarningShown && state == RUNNING) {
    showLcd("AJUSTANDO HORA", "ESTEIRA PARADA");
    clockWarningShown = true;
  }
}

void acknowledgeConfig(int version) {
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  HTTPClient http;
  String url = String(API_BASE_URL) + "/api/v1/device/config/ack?device_id=" + DEVICE_ID;
  if (!beginHttp(http, plainClient, secureClient, url)) return;
  addDeviceHeaders(http);
  http.addHeader("Content-Type", "application/json");
  int statusCode = http.POST("{\"version\":" + String(version) + "}");
  http.end();
  if (statusCode >= 200 && statusCode < 300) appliedConfigVersion = version;
}

void pollRemoteConfig() {
  if (WiFi.status() != WL_CONNECTED || millis() - lastConfigAt < CONFIG_POLL_MS) return;
  serviceSafetyStops();
  lastConfigAt = millis();
  const unsigned long requestStartedAt = millis();
  portENTER_CRITICAL(&sensorMux);
  const uint32_t requestGeneration = authorizationGeneration;
  portEXIT_CRITICAL(&sensorMux);
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  HTTPClient http;
  String url = String(API_BASE_URL) + "/api/v1/device/config?device_id=" + DEVICE_ID +
      "&boot_id=" + bootId;
  if (!beginHttp(http, plainClient, secureClient, url)) return;
  addDeviceHeaders(http);
  int statusCode = http.GET();
  String body = statusCode == 200 ? http.getString() : "";
  http.end();
  if (statusCode != 200) {
    Serial.printf("Configuração da esteira: resposta HTTP %d\n", statusCode);
    return;
  }

  DynamicJsonDocument document(1536);
  if (deserializeJson(document, body) != DeserializationError::Ok) return;
  JsonObject config = document["config"].as<JsonObject>();
  if (config.isNull() || !document["config_version"].is<int>() ||
      document["config_version"].as<int>() < 1) return;
  // Verifique o prazo ANTIGO antes de renovar qualquer dado recebido.
  if (remoteRunEnabled && (!lastConfigSuccessAt ||
      millis() - lastConfigSuccessAt > CONFIG_LEASE_MS)) latchSafetyStop();
  serviceSafetyStops();
  const bool staleResponse = requestGeneration != authorizationGeneration ||
      millis() - requestStartedAt > CONFIG_RESPONSE_MAX_MS ||
      WiFi.status() != WL_CONNECTED;
  const bool discardCommand = discardNextRunCommand;
  motorSpeed = constrain(config["motor_speed"] | motorSpeed, 0, 255);
  detectionLimitCm = constrain(config["detection_limit_cm"] | detectionLimitCm, 2.0f, 100.0f);
  requiredReadings = constrain(config["required_readings"] | requiredReadings, 1, 20);
  minimumStopSeconds = constrain(config["stop_timeout_seconds"] | minimumStopSeconds, 1, 60);
  resultTimeoutSeconds = constrain(config["result_timeout_seconds"] | resultTimeoutSeconds, 1, 120);
  releaseConfidencePercent = constrain(config["release_confidence_percent"] | releaseConfidencePercent, 0, 100);
  const bool requestedRun = config["run_enabled"] | false;
  const String requestedToken = config["run_token"] | "";
  const String requestedBootId = config["run_boot_id"] | "";
  const bool wasEnabled = remoteRunEnabled;
  bool acceptRun = false;
  bool newAuthorization = false;
  if (!requestedRun || !validRunToken(requestedToken) || requestedBootId != bootId) {
    activeRunToken = "";
  } else if (staleResponse || discardCommand) {
    // Uma resposta atrasada não pode voltar na consulta seguinte como comando novo.
    rejectedRunToken = requestedToken;
    rememberRunToken(requestedToken);
    activeRunToken = "";
    Serial.println("Liberação recebida após falha/prazo: envie um novo comando pelo site.");
  } else if (remoteRunEnabled && requestedToken == activeRunToken) {
    acceptRun = true;
  } else if (requestedToken != lastRunToken && requestedToken != rejectedRunToken &&
             rememberRunToken(requestedToken)) {
    activeRunToken = requestedToken;
    acceptRun = true;
    newAuthorization = true;
  } else {
    activeRunToken = "";
  }
  portENTER_CRITICAL(&sensorMux);
  // O sensor/evento Wi-Fi pode invalidar a autorização durante a escrita no NVS.
  if (requestGeneration != authorizationGeneration || safetyStopPending) acceptRun = false;
  if (!staleResponse && requestGeneration == authorizationGeneration && !safetyStopPending) {
    discardNextRunCommand = false;
  }
  lastConfigSuccessAt = millis();
  remoteRunEnabled = acceptRun;
  portEXIT_CRITICAL(&sensorMux);
  if (newAuthorization && acceptRun) {
    needsReview = false;
    releaseBlocked = false;
  }
  if (!remoteRunEnabled) {
    activeRunToken = "";
    if (state == CLEARING_OBJECT) state = WAITING_OBJECT_CLEAR;
    stopMotor();
    if (wasEnabled) showLcd("ESTEIRA PARADA", "AGUARDANDO SITE");
  }
  int version = document["config_version"] | 0;
  acknowledgeConfig(version);
}

void sendHeartbeat() {
  if (WiFi.status() != WL_CONNECTED || millis() - lastHeartbeatAt < HEARTBEAT_MS) return;
  lastHeartbeatAt = millis();
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  HTTPClient http;
  String url = String(API_BASE_URL) + "/api/v1/device/heartbeat?device_id=" + DEVICE_ID;
  if (!beginHttp(http, plainClient, secureClient, url)) return;
  addDeviceHeaders(http);
  http.addHeader("Content-Type", "application/json");
  DynamicJsonDocument document(768);
  document["firmware_version"] = "esteira-api-1.3.0";
  document["boot_id"] = bootId;
  document["status"]["boot_id"] = bootId;
  document["status"]["wifi_rssi"] = WiFi.RSSI();
  document["status"]["state"] = static_cast<int>(state);
  document["status"]["distance_cm"] = lastDistanceCm;
  document["status"]["local_counter"] = localCounter;
  document["status"]["config_version"] = appliedConfigVersion;
  document["status"]["motor_running"] = motorRunning;
  document["status"]["run_enabled"] = remoteRunEnabled;
  document["status"]["config_fresh"] = lastConfigSuccessAt && millis() - lastConfigSuccessAt <= CONFIG_LEASE_MS;
  document["status"]["needs_review"] = needsReview;
  document["status"]["release_blocked"] = releaseBlocked;
  document["status"]["confidence_percent"] = lastConfidencePercent;
  document["status"]["motor_driver_ready"] = motorPwmReady;
  document["status"]["sensor_ready"] = sensorTaskReady;
  String body;
  serializeJson(document, body);
  http.POST(body);
  http.end();
}

void beginDetection(float distanceCm) {
  stopMotor();
  lastConfidencePercent = -1.0f;
  localCounter++;
  clientEventId = String(DEVICE_ID) + "-" + bootId + "-" + String(localCounter);
  serverEventId = "";
  lastDistanceCm = distanceCm;
  state = SENDING_EVENT;
  stateStartedAt = millis();
  detectionStartedAt = millis();
  lastEventAttemptAt = 0;
  showLcd("OBJETO DETECTADO", "ENVIANDO FOTO...");
}

void beginClearance() {
  if (!remoteRunEnabled || needsReview || state != WAITING_OBJECT_CLEAR) return;
  releaseStartedAt = millis();
  state = CLEARING_OBJECT;
  clearReadings = 0;
  showLcd("LIBERANDO PECA", "AGUARDE...");
  Serial.println("Liberando peça do sensor por tempo limitado.");
}

void sendDetectionEvent() {
  if (WiFi.status() != WL_CONNECTED || millis() - lastEventAttemptAt < EVENT_RETRY_MS) return;
  lastEventAttemptAt = millis();
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  HTTPClient http;
  String url = String(API_BASE_URL) + "/api/v1/conveyor/events";
  if (!beginHttp(http, plainClient, secureClient, url)) return;
  addDeviceHeaders(http);
  http.addHeader("Content-Type", "application/json");
  DynamicJsonDocument document(512);
  document["client_event_id"] = clientEventId;
  document["sensor_distance_cm"] = lastDistanceCm;
  document["local_counter"] = localCounter;
  document["camera_device_id"] = CAMERA_DEVICE_ID;
  String body;
  serializeJson(document, body);
  int statusCode = http.POST(body);
  String response = (statusCode == 200 || statusCode == 201) ? http.getString() : "";
  http.end();
  if (statusCode != 200 && statusCode != 201) {
    Serial.printf("Evento da esteira: resposta HTTP %d\n", statusCode);
    if (statusCode == 401 || statusCode == 403) {
      showLcd("ERRO DE ACESSO", "CONFIRA TOKEN");
    } else if (statusCode == 404) {
      showLcd("DISPOSITIVO", "NAO CADASTRADO");
    }
    return;
  }

  DynamicJsonDocument result(768);
  if (deserializeJson(result, response) != DeserializationError::Ok) return;
  serverEventId = result["event_id"].as<String>();
  if (!serverEventId.length()) {
    Serial.println("A API não devolveu o ID do evento.");
    return;
  }
  state = WAITING_RESULT;
  stateStartedAt = millis();
  lastResultPollAt = 0;
  resultTimeoutShown = false;
  showLcd("FOTO SOLICITADA", "AGUARDANDO IA");
}

void pollRecognitionResult() {
  if (WiFi.status() != WL_CONNECTED || !serverEventId.length()) return;
  if (millis() - lastResultPollAt < RESULT_POLL_MS) return;
  lastResultPollAt = millis();
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  HTTPClient http;
  String url = String(API_BASE_URL) + "/api/v1/conveyor/events/" + serverEventId + "/result";
  if (!beginHttp(http, plainClient, secureClient, url)) return;
  addDeviceHeaders(http);
  int statusCode = http.GET();
  String response = statusCode == 200 ? http.getString() : "";
  http.end();
  if (statusCode != 200) {
    Serial.printf("Resultado da esteira: resposta HTTP %d\n", statusCode);
    if (statusCode == 401 || statusCode == 403) {
      showLcd("ERRO DE ACESSO", "CONFIRA TOKEN");
    }
    return;
  }

  DynamicJsonDocument document(1024);
  if (deserializeJson(document, response) != DeserializationError::Ok) return;
  String statusValue = document["status"] | "";
  const float confidence = document["confidence"].is<float>() ?
      document["confidence"].as<float>() : -1.0f;
  lastConfidencePercent = isfinite(confidence) && confidence >= 0 && confidence <= 1 ?
      confidence * 100.0f : -1.0f;
  if (statusValue == "recognized") {
    String identifier = document["component_identifier"] | "IDENTIFICADO";
    lastResult = identifier;
    showLcd("ID COMPONENTE:", identifier);
  } else if (statusValue == "unknown") {
    lastResult = "DESCONHECIDO";
    showLcd("NAO RECONHECIDO", "VERIFIQUE A PECA");
  } else if (statusValue == "failed") {
    lastResult = "ERRO NA CAMERA";
    showLcd("FALHA NA CAPTURA", "VERIFIQUE SIST.");
  } else {
    if (!resultTimeoutShown &&
        millis() - stateStartedAt > (unsigned long)resultTimeoutSeconds * 1000UL) {
      showLcd("AINDA AGUARDANDO", "CONEXAO/IA");
      Serial.println("Resultado demorando; motor permanece parado.");
      resultTimeoutShown = true;
    }
    return;
  }
  if (statusValue != "recognized" || lastConfidencePercent < releaseConfidencePercent) {
    needsReview = true;
    remoteRunEnabled = false;
    activeRunToken = "";
    stopMotor();
    Serial.printf("Esteira parada para conferência: confiança %.1f%%, mínimo %d%%.\n",
                  lastConfidencePercent, releaseConfidencePercent);
  }
  state = SHOWING_RESULT;
  stateStartedAt = millis();
}

void updateSensorState() {
  if (millis() - lastSensorAt < SENSOR_READ_MS) return;
  lastSensorAt = millis();
  portENTER_CRITICAL(&sensorMux);
  const float distanceCm = sampledDistanceCm;
  portEXIT_CRITICAL(&sensorMux);
  lastDistanceCm = distanceCm;
  if (distanceCm <= 0) {
    detectionReadings = 0;
    clearReadings = 0;
    if (state == RUNNING && !sensorErrorShown) {
      showLcd("ERRO NO SENSOR", "ESTEIRA PARADA");
      Serial.println("Sensor sem leitura válida; motor parado.");
      sensorErrorShown = true;
    }
    return;
  }
  if (sensorErrorShown) {
    sensorErrorShown = false;
  }

  if (state == RUNNING) {
    if (!remoteRunEnabled || !lastConfigSuccessAt ||
        millis() - lastConfigSuccessAt > CONFIG_LEASE_MS) {
      detectionReadings = 0;
      return;
    }
    if (distanceCm < detectionLimitCm) detectionReadings++;
    else detectionReadings = 0;
    if (detectionReadings >= requiredReadings) {
      detectionReadings = 0;
      beginDetection(distanceCm);
    }
  } else if (state == WAITING_OBJECT_CLEAR) {
    if (millis() - detectionStartedAt < (unsigned long)minimumStopSeconds * 1000UL) return;
    if (distanceCm > detectionLimitCm + CLEAR_MARGIN_CM) clearReadings++;
    else clearReadings = 0;
    if (clearReadings >= requiredReadings) {
      clearReadings = 0;
      state = RUNNING;
    }
  } else if (state == CLEARING_OBJECT) {
    if (distanceCm > detectionLimitCm + CLEAR_MARGIN_CM) clearReadings++;
    else clearReadings = 0;
    if (clearReadings >= requiredReadings) {
      clearReadings = 0;
      state = RUNNING;
      showLcd("ESTEIRA LIVRE", "PROXIMA PECA");
      Serial.println("Peça saiu do sensor; esteira segue para a próxima.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  motorPwmReady = ledcAttach(MOTOR_ENA, 5000, 8);
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();
  showLcd("SISTEMA ESTEIRA", "INICIANDO...");
  char bootBuffer[33];
  snprintf(bootBuffer, sizeof(bootBuffer), "%08lx%08lx%08lx%08lx",
      (unsigned long)esp_random(), (unsigned long)esp_random(),
      (unsigned long)esp_random(), (unsigned long)esp_random());
  bootId = bootBuffer;
  runPreferencesReady = runPreferences.begin("conveyor", false);
  if (runPreferencesReady) lastRunToken = runPreferences.getString("last_token", "");
  WiFi.setSleep(false);
  stopMotor();
  if (String(DEVICE_TOKEN) == "MESMO_DEVICE_API_TOKEN_DO_ENV" ||
      String(CONFIG_PORTAL_PASSWORD) == "DEFINA_SENHA_DO_PORTAL" ||
      strlen(DEVICE_TOKEN) < 16 || strlen(CONFIG_PORTAL_PASSWORD) < 8 ||
      strlen(CONFIG_PORTAL_PASSWORD) > 63 ||
      !String(API_BASE_URL).startsWith("https://")) {
    Serial.println("Configure token, senha do portal e endereço HTTPS antes de usar.");
    showLcd("CONFIGURE TOKEN", "E SENHA WIFI");
    while (true) delay(1000);
  }
  if (!motorPwmReady) {
    Serial.println("Falha ao configurar PWM; motor permanece bloqueado.");
    showLcd("ERRO MOTOR/PWM", "ESTEIRA PARADA");
  }
  WiFi.onEvent([](WiFiEvent_t event) {
    (void)event;
    latchSafetyStop();
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  sensorTaskReady = xTaskCreatePinnedToCore(
      sensorSafetyTask, "sensor_seguro", 4096, nullptr, 2, nullptr, 1) == pdPASS;
  if (!sensorTaskReady) {
    Serial.println("Falha ao iniciar leitura de segurança do sensor.");
    showLcd("ERRO NO SENSOR", "ESTEIRA PARADA");
  }
  startWiFiProvisioning();
  showLcd("ESTEIRA PARADA", "AGUARDANDO SITE");
  updateClock();
}

void loop() {
  connectWiFi();
  serviceSafetyStops();
  updateClock();
  if (remoteRunEnabled && (!lastConfigSuccessAt ||
      millis() - lastConfigSuccessAt > CONFIG_LEASE_MS)) {
    latchSafetyStop();
  }
  serviceSafetyStops();
  // Durante os até 3 segundos de saída, confirme o sensor sem bloquear em HTTPS.
  // A tarefa independente continua impondo os prazos e a perda de Wi-Fi.
  if (state != CLEARING_OBJECT) {
    pollRemoteConfig();
    serviceSafetyStops();
    sendHeartbeat();
    serviceSafetyStops();
  }

  // Verifique a saída antes de atualizar o estado com uma leitura tardia.
  if (state == CLEARING_OBJECT && millis() - releaseStartedAt >= RELEASE_MAX_MS) {
    latchSafetyStop(true);
    serviceSafetyStops();
  }
  updateSensorState();
  if (state == WAITING_OBJECT_CLEAR && remoteRunEnabled && !needsReview &&
      millis() - detectionStartedAt >= (unsigned long)minimumStopSeconds * 1000UL) {
    beginClearance();
  }

  // O motor só recebe energia após rede, relógio e sensor estarem prontos.
  if (state == RUNNING || state == CLEARING_OBJECT) startMotor();

  if (state == SENDING_EVENT) sendDetectionEvent();
  else if (state == WAITING_RESULT) pollRecognitionResult();
  else if (state == SHOWING_RESULT && millis() - stateStartedAt >= RESULT_DISPLAY_MS) {
    state = WAITING_OBJECT_CLEAR;
    clearReadings = 0;
    if (needsReview) showLcd("CONFIRA A PECA", "LIBERE NO SITE");
    else showLcd("PREPARANDO", "PROXIMA PECA");
  }
  delay(10);
}
