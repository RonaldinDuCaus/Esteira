#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <HTTPClient.h>
#include <SD_MMC.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <driver/ledc.h>
#include <time.h>
#include "esp_camera.h"
#include "root_ca.h"

// O Wi-Fi é configurado pelo portal local da própria ESP32-CAM.
// A API final usa HTTPS com o certificado raiz de root_ca.h.
const char *API_BASE_URL = "https://ai.kronux.com.br";
const char *DEVICE_ID = "esp32-cam-01";
const char *DEVICE_TOKEN = "token site";
const char *CONFIG_PORTAL_SSID = "Esteira6";
const char *CONFIG_PORTAL_PASSWORD = "@208862Sfc";

const unsigned long WIFI_RETRY_INTERVAL_MS = 5000;
const unsigned long CONFIG_POLL_INTERVAL_MS = 30000;
const unsigned long HEARTBEAT_INTERVAL_MS = 15000;
const unsigned long CLOCK_RETRY_INTERVAL_MS = 30000;
const unsigned long HARDWARE_RETRY_INTERVAL_MS = 10000;
const unsigned long UPLOAD_RETRY_INTERVAL_MS = 3000;
const unsigned long CONFIG_PORTAL_WINDOW_MS = 120000;
const ledc_channel_t FLASH_PWM_CHANNEL = LEDC_CHANNEL_2;
const ledc_timer_t FLASH_PWM_TIMER = LEDC_TIMER_1;
const ledc_mode_t FLASH_PWM_MODE = LEDC_LOW_SPEED_MODE;
const uint32_t FLASH_PWM_FREQUENCY = 5000;
const char *PENDING_FILE = "/camera_pending.txt";
const char *PENDING_TEMP_FILE = "/camera_pending.tmp";

// Pinagem padrão da ESP32-CAM AI Thinker.
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define FLASH_GPIO_NUM 4

String pendingRequestId;
String pendingUploadUrl;
String pendingImagePath;
String pendingFailureMessage;
unsigned long lastPollAt = 0;
unsigned long lastWiFiAttemptAt = 0;
unsigned long lastConfigAt = 0;
unsigned long lastHeartbeatAt = 0;
unsigned long lastClockRequestAt = 0;
unsigned long lastHardwareAttemptAt = 0;
unsigned long lastUploadAttemptAt = 0;
unsigned long commandPollIntervalMs = 1500;
bool flashEnabled = true;
int flashIntensityPct = 30;
bool flashPwmReady = false;
unsigned long configPortalStartedAt = 0;
String cameraConfigError;
int appliedConfigVersion = 0;
unsigned long wifiDisconnectedAt = 0;
WiFiManager wifiManager;
bool clockSynchronized = false;
bool storageReady = false;
bool cameraReady = false;

void setFlashIntensity(int percent) {
  if (!flashPwmReady) {
    digitalWrite(FLASH_GPIO_NUM, LOW);
    return;
  }
  const uint32_t duty = (constrain(percent, 0, 100) * 255UL + 50) / 100;
  if (ledc_set_duty(FLASH_PWM_MODE, FLASH_PWM_CHANNEL, duty) != ESP_OK ||
      ledc_update_duty(FLASH_PWM_MODE, FLASH_PWM_CHANNEL) != ESP_OK) {
    ledc_stop(FLASH_PWM_MODE, FLASH_PWM_CHANNEL, 0);
    flashPwmReady = false;
    pinMode(FLASH_GPIO_NUM, OUTPUT);
    digitalWrite(FLASH_GPIO_NUM, LOW);
  }
}

void initializeFlash() {
  // GPIO 4 só fica livre porque SD_MMC.begin usa o modo 1-bit.
  // Usa o ESP-IDF para reservar explicitamente timer 1, separado do XCLK
  // (timer 0). No core 3.x, escolher apenas o canal não reserva um timer.
  ledc_timer_config_t timer = {};
  timer.speed_mode = FLASH_PWM_MODE;
  timer.timer_num = FLASH_PWM_TIMER;
  timer.duty_resolution = LEDC_TIMER_8_BIT;
  timer.freq_hz = FLASH_PWM_FREQUENCY;
  timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_channel_config_t channel = {};
  channel.gpio_num = FLASH_GPIO_NUM;
  channel.speed_mode = FLASH_PWM_MODE;
  channel.channel = FLASH_PWM_CHANNEL;
  channel.timer_sel = FLASH_PWM_TIMER;
  channel.intr_type = LEDC_INTR_DISABLE;
  channel.duty = 0;
  channel.hpoint = 0;
  flashPwmReady = ledc_timer_config(&timer) == ESP_OK &&
                  ledc_channel_config(&channel) == ESP_OK;
  setFlashIntensity(0);
  if (!flashPwmReady) Serial.println("Flash PWM indisponível; iluminação desligada.");
}

String absoluteUrl(const String &path) {
  if (path.startsWith("http://") || path.startsWith("https://")) return path;
  return String(API_BASE_URL) + path;
}

bool beginHttp(HTTPClient &http, WiFiClient &plainClient,
               WiFiClientSecure &secureClient, const String &url) {
  (void)plainClient;
  const String base = API_BASE_URL;
  if (!base.startsWith("https://") || !url.startsWith(base + "/") ||
      !clockSynchronized) return false;
  secureClient.setCACert(ROOT_CA_CERT);
  secureClient.setHandshakeTimeout(10);
  http.setConnectTimeout(5000);
  http.setTimeout(15000);
  return http.begin(secureClient, url);
}

void addDeviceHeaders(HTTPClient &http) {
  http.addHeader("Authorization", "Bearer " + String(DEVICE_TOKEN));
  http.addHeader("X-Device-ID", DEVICE_ID);
}

String safePathPart(String value) {
  value.toUpperCase();
  String safe;
  for (size_t index = 0; index < value.length(); index++) {
    const char current = value.charAt(index);
    if ((current >= 'A' && current <= 'Z') ||
        (current >= '0' && current <= '9') || current == '-' || current == '_') {
      safe += current;
    }
  }
  return safe.length() ? safe : "SEM_ID";
}

bool initializeCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // Reserva para a maior resolução oferecida no site antes de reduzir o sensor.
  // Sem PSRAM, restringe a QVGA para manter memória disponível para HTTPS.
  config.frame_size = psramFound() ? FRAMESIZE_SVGA : FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count = psramFound() ? 2 : 1;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  if (esp_camera_init(&config) != ESP_OK) return false;
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor || sensor->set_framesize(sensor, FRAMESIZE_QVGA) != 0 ||
      sensor->set_quality(sensor, 15) != 0) {
    esp_camera_deinit();
    return false;
  }
  appliedConfigVersion = 0;
  lastConfigAt = millis() - CONFIG_POLL_INTERVAL_MS;
  return true;
}

framesize_t parseFrameSize(const String &value) {
  if (value == "SVGA") return FRAMESIZE_SVGA;
  if (value == "VGA") return FRAMESIZE_VGA;
  return FRAMESIZE_QVGA;
}

bool initializeStorage() {
  // O modo 1-bit reduz conflitos de pinos na ESP32-CAM AI Thinker.
  if (!SD_MMC.begin("/sdcard", true)) return false;
  if (SD_MMC.cardType() == CARD_NONE) {
    SD_MMC.end();
    return false;
  }
  if (!SD_MMC.exists("/components")) SD_MMC.mkdir("/components");
  if (!SD_MMC.exists("/components")) {
    SD_MMC.end();
    return false;
  }
  return true;
}

void connectWiFi() {
  if (wifiManager.getConfigPortalActive()) wifiManager.process();
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiManager.getConfigPortalActive()) wifiManager.stopConfigPortal();
    wifiDisconnectedAt = 0;
    return;
  }
  // O portal tem uma janela finita; depois volta a tentar a rede já salva.
  // Isso evita ficar preso no portal quando o roteador apenas reinicia.
  if (wifiManager.getConfigPortalActive()) {
    if (millis() - configPortalStartedAt < CONFIG_PORTAL_WINDOW_MS) return;
    wifiManager.stopConfigPortal();
    WiFi.mode(WIFI_STA);
    WiFi.begin();
    wifiDisconnectedAt = millis();
    lastWiFiAttemptAt = millis();
    return;
  }
  if (!wifiDisconnectedAt) wifiDisconnectedAt = millis();
  if (millis() - lastWiFiAttemptAt < WIFI_RETRY_INTERVAL_MS) return;
  lastWiFiAttemptAt = millis();
  WiFi.reconnect();

  // Depois de um minuto sem a rede salva, reabre o site de configuração.
  if (millis() - wifiDisconnectedAt >= 60000UL) {
    Serial.println("Wi-Fi indisponível. Abrindo portal em http://192.168.4.1");
    configPortalStartedAt = millis();
    wifiManager.startConfigPortal(CONFIG_PORTAL_SSID, CONFIG_PORTAL_PASSWORD);
    wifiDisconnectedAt = 0;
  }
}

void startWiFiProvisioning() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_ID);
  wifiManager.setConnectTimeout(20);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(0);
  wifiManager.setWiFiAutoReconnect(true);
  wifiManager.setAPCallback([](WiFiManager *) {
    Serial.println("Conecte em MiniFabrica-Camera e abra http://192.168.4.1");
  });
  const bool connected = wifiManager.autoConnect(CONFIG_PORTAL_SSID, CONFIG_PORTAL_PASSWORD);
  configPortalStartedAt = millis();
  wifiDisconnectedAt = connected ? 0 : millis();
  if (!connected) {
    Serial.println("Portal Wi-Fi aberto; a câmera continuará tentando se conectar.");
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
    return;
  }
  clockSynchronized = false;
  if (!lastClockRequestAt || millis() - lastClockRequestAt >= CLOCK_RETRY_INTERVAL_MS) {
    configTime(0, 0, "pool.ntp.org", "time.cloudflare.com");
    lastClockRequestAt = millis();
    Serial.println("Sincronizando relógio para HTTPS.");
  }
}

bool savePendingUpload() {
  if (SD_MMC.exists(PENDING_TEMP_FILE)) SD_MMC.remove(PENDING_TEMP_FILE);
  File file = SD_MMC.open(PENDING_TEMP_FILE, FILE_WRITE);
  if (!file) return false;
  const String contents = pendingRequestId + "\n" + pendingUploadUrl + "\n" +
                          pendingImagePath + "\n" + pendingFailureMessage + "\n";
  const bool written = file.print(contents) == contents.length();
  file.flush();
  file.close();
  if (!written) {
    SD_MMC.remove(PENDING_TEMP_FILE);
    return false;
  }
  if (SD_MMC.exists(PENDING_FILE)) SD_MMC.remove(PENDING_FILE);
  if (!SD_MMC.rename(PENDING_TEMP_FILE, PENDING_FILE)) {
    SD_MMC.remove(PENDING_TEMP_FILE);
    return false;
  }
  return true;
}

void clearPendingUpload() {
  pendingRequestId = "";
  pendingUploadUrl = "";
  pendingImagePath = "";
  pendingFailureMessage = "";
  if (SD_MMC.exists(PENDING_FILE)) SD_MMC.remove(PENDING_FILE);
}

void loadPendingUpload() {
  if (!SD_MMC.exists(PENDING_FILE)) return;
  File file = SD_MMC.open(PENDING_FILE, FILE_READ);
  if (!file) return;
  pendingRequestId = file.readStringUntil('\n');
  pendingUploadUrl = file.readStringUntil('\n');
  pendingImagePath = file.readStringUntil('\n');
  // A quarta linha é opcional para manter compatibilidade com a versão 1.1.0.
  pendingFailureMessage = file.available() ? file.readStringUntil('\n') : "";
  pendingRequestId.trim();
  pendingUploadUrl.trim();
  pendingImagePath.trim();
  pendingFailureMessage.trim();
  file.close();
  if (!pendingRequestId.length() || !pendingUploadUrl.length() ||
      !pendingImagePath.length()) {
    Serial.println("Registro de envio pendente incompleto; liberando nova captura.");
    clearPendingUpload();
    return;
  }
  lastUploadAttemptAt = millis() - UPLOAD_RETRY_INTERVAL_MS;
}

bool captureToSd(const String &componentIdentifier, const String &requestId,
                 String &savedPath) {
  setFlashIntensity(0);
  const String directory = "/components/" + safePathPart(componentIdentifier);
  if (!SD_MMC.exists(directory) && !SD_MMC.mkdir(directory)) return false;
  savedPath = directory + "/" + safePathPart(requestId) + ".jpg";

  if (flashEnabled && flashIntensityPct > 0) {
    setFlashIntensity(flashIntensityPct);
    delay(180);
  }
  // Descarta o quadro anterior, inclusive no modo com um único buffer.
  camera_fb_t *previousFrame = esp_camera_fb_get();
  if (!previousFrame) {
    setFlashIntensity(0);
    return false;
  }
  esp_camera_fb_return(previousFrame);
  camera_fb_t *frame = esp_camera_fb_get();
  setFlashIntensity(0);
  if (!frame) return false;
  File image = SD_MMC.open(savedPath, FILE_WRITE);
  if (!image) {
    esp_camera_fb_return(frame);
    return false;
  }
  const size_t written = image.write(frame->buf, frame->len);
  image.close();
  const bool success = written == frame->len;
  esp_camera_fb_return(frame);
  if (!success) SD_MMC.remove(savedPath);
  return success;
}

void retryHardware() {
  if (storageReady && cameraReady) return;
  if (lastHardwareAttemptAt &&
      millis() - lastHardwareAttemptAt < HARDWARE_RETRY_INTERVAL_MS) return;
  lastHardwareAttemptAt = millis();
  if (!storageReady) {
    storageReady = initializeStorage();
    if (storageReady) {
      Serial.println("microSD pronto.");
      if (!pendingRequestId.length()) loadPendingUpload();
    } else {
      Serial.println("microSD indisponível; nova tentativa em 10 segundos.");
    }
  }
  if (!cameraReady) {
    cameraReady = initializeCamera();
    Serial.println(cameraReady ? "Câmera pronta." :
                "Câmera indisponível; nova tentativa em 10 segundos.");
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
  String body = "{\"version\":" + String(version) + "}";
  const int statusCode = http.POST(body);
  http.end();
  if (statusCode >= 200 && statusCode < 300) appliedConfigVersion = version;
}

void pollRemoteConfig() {
  if (WiFi.status() != WL_CONNECTED || !clockSynchronized || !cameraReady ||
      millis() - lastConfigAt < CONFIG_POLL_INTERVAL_MS) return;
  lastConfigAt = millis();
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  HTTPClient http;
  String url = String(API_BASE_URL) + "/api/v1/device/config?device_id=" + DEVICE_ID;
  if (!beginHttp(http, plainClient, secureClient, url)) return;
  addDeviceHeaders(http);
  int statusCode = http.GET();
  String body = statusCode == 200 ? http.getString() : "";
  http.end();
  if (statusCode != 200) return;

  DynamicJsonDocument document(1536);
  if (deserializeJson(document, body) != DeserializationError::Ok) return;
  JsonObject config = document["config"];
  int version = document["config_version"] | 0;
  const unsigned long nextPollInterval = config["poll_interval_ms"] | commandPollIntervalMs;
  const bool nextFlashEnabled = config["flash_enabled"] | flashEnabled;
  const int nextFlashIntensity = config["flash_intensity_pct"] | 30;
  int quality = config["jpeg_quality"] | 15;
  String frameSize = config["frame_size"] | "QVGA";
  if (nextPollInterval < 500 || nextPollInterval > 10000 ||
      nextFlashIntensity < 0 || nextFlashIntensity > 100 || quality < 8 || quality > 30 ||
      (frameSize != "QVGA" && frameSize != "VGA" && frameSize != "SVGA")) {
    cameraConfigError = "Configuração da câmera fora dos limites permitidos.";
    return;
  }
  if (!psramFound() && frameSize != "QVGA") {
    cameraConfigError = "PSRAM indisponível: selecione resolução QVGA.";
    return;
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor || sensor->set_quality(sensor, quality) != 0 ||
      sensor->set_framesize(sensor, parseFrameSize(frameSize)) != 0) {
    Serial.println("Falha ao aplicar configuração da câmera; tentando novamente.");
    cameraConfigError = "Falha ao aplicar resolução ou qualidade da câmera.";
    return;
  }
  commandPollIntervalMs = nextPollInterval;
  flashEnabled = nextFlashEnabled;
  flashIntensityPct = nextFlashIntensity;
  setFlashIntensity(0);
  cameraConfigError = "";
  acknowledgeConfig(version);
}

void sendHeartbeat() {
  if (WiFi.status() != WL_CONNECTED || millis() - lastHeartbeatAt < HEARTBEAT_INTERVAL_MS) return;
  lastHeartbeatAt = millis();
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  HTTPClient http;
  String url = String(API_BASE_URL) + "/api/v1/device/heartbeat?device_id=" + DEVICE_ID;
  if (!beginHttp(http, plainClient, secureClient, url)) return;
  addDeviceHeaders(http);
  http.addHeader("Content-Type", "application/json");
  DynamicJsonDocument document(1024);
  document["firmware_version"] = "camera-api-1.2.0";
  document["status"]["wifi_rssi"] = WiFi.RSSI();
  document["status"]["pending_upload"] = pendingRequestId.length() > 0;
  document["status"]["micro_sd_ready"] = storageReady;
  document["status"]["camera_ready"] = cameraReady;
  document["status"]["clock_ready"] = clockSynchronized;
  document["status"]["config_version"] = appliedConfigVersion;
  document["status"]["flash_enabled"] = flashEnabled;
  document["status"]["flash_intensity_pct"] = flashIntensityPct;
  document["status"]["flash_pwm_ready"] = flashPwmReady;
  document["status"]["psram_ready"] = psramFound();
  document["status"]["config_error"] = cameraConfigError;
  document["status"]["pending_failure"] = pendingFailureMessage.length() > 0;
  String body;
  serializeJson(document, body);
  http.POST(body);
  http.end();
}

bool uploadPendingImage() {
  if (!pendingRequestId.length() ||
      WiFi.status() != WL_CONNECTED || !clockSynchronized) return false;
  if (millis() - lastUploadAttemptAt < UPLOAD_RETRY_INTERVAL_MS) return false;
  lastUploadAttemptAt = millis();
  if (pendingFailureMessage.length()) {
    if (reportFailure("/api/v1/camera/requests/" + pendingRequestId + "/fail",
                      pendingFailureMessage)) {
      Serial.println("Falha registrada na VPS; foto preservada no microSD.");
      clearPendingUpload();
    }
    return false;
  }
  if (!storageReady) return false;
  File image = SD_MMC.open(pendingImagePath, FILE_READ);
  if (!image || image.isDirectory() || image.size() == 0) {
    image.close();
    Serial.println("Foto pendente ausente ou vazia no microSD; registrando falha na VPS.");
    pendingFailureMessage = "Fotografia pendente ausente ou vazia no microSD.";
    savePendingUpload();
    return false;
  }

  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  HTTPClient http;
  const String url = absoluteUrl(pendingUploadUrl);
  if (!beginHttp(http, plainClient, secureClient, url)) {
    image.close();
    return false;
  }
  addDeviceHeaders(http);
  http.addHeader("Content-Type", "image/jpeg");
  http.setTimeout(45000);
  const int statusCode = http.sendRequest("POST", &image, image.size());
  const String response = statusCode == 409 ? http.getString() : "";
  image.close();
  http.end();
  if (statusCode >= 200 && statusCode < 300) {
    clearPendingUpload();
    return true;
  }
  if (statusCode == 409) {
    if (response.indexOf("Esta captura já foi concluída.") >= 0) {
      Serial.println("Foto já recebida pela VPS; liberando próxima captura.");
      clearPendingUpload();
      return true;
    }
    Serial.println("Pedido de captura encerrado na VPS; foto preservada no microSD.");
    clearPendingUpload();
    return false;
  }
  if (statusCode == 404) {
    Serial.println("Pedido não encontrado na VPS; foto preservada no microSD.");
    clearPendingUpload();
    return false;
  }
  if (statusCode == 400 || statusCode == 413 || statusCode == 415 || statusCode == 422) {
    pendingFailureMessage = "Fotografia rejeitada pela VPS (HTTP " + String(statusCode) +
                            "). Original preservado no microSD para diagnóstico.";
    // Persiste a falha, não o reenvio da mesma fotografia rejeitada, após reiniciar.
    if (!savePendingUpload()) Serial.println("Não foi possível persistir a falha no microSD.");
    if (reportFailure("/api/v1/camera/requests/" + pendingRequestId + "/fail",
                      pendingFailureMessage)) clearPendingUpload();
    return false;
  }
  Serial.printf("Envio da foto: resposta HTTP %d; nova tentativa depois.\n", statusCode);
  return false;
}

bool reportFailure(const String &failureUrl, const String &message) {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  HTTPClient http;
  if (!beginHttp(http, plainClient, secureClient, absoluteUrl(failureUrl))) return false;
  addDeviceHeaders(http);
  http.addHeader("Content-Type", "application/json");
  DynamicJsonDocument document(256);
  document["message"] = message;
  String body;
  serializeJson(document, body);
  const int statusCode = http.POST(body);
  http.end();
  return (statusCode >= 200 && statusCode < 300) || statusCode == 404 || statusCode == 409;
}

void pollCameraCommand() {
  if (WiFi.status() != WL_CONNECTED || !clockSynchronized ||
      !storageReady || !cameraReady || appliedConfigVersion <= 0 ||
      pendingRequestId.length()) return;
  if (millis() - lastPollAt < commandPollIntervalMs) return;
  lastPollAt = millis();

  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  HTTPClient http;
  const String url = String(API_BASE_URL) +
                     "/api/v1/camera/commands/next?device_id=" + DEVICE_ID;
  if (!beginHttp(http, plainClient, secureClient, url)) return;
  addDeviceHeaders(http);
  const int statusCode = http.GET();
  if (statusCode == 204) {
    http.end();
    return;
  }
  if (statusCode != 200) {
    Serial.printf("Consulta da câmera: resposta HTTP %d\n", statusCode);
    http.end();
    return;
  }
  const String body = http.getString();
  http.end();

  DynamicJsonDocument document(1536);
  if (deserializeJson(document, body) != DeserializationError::Ok) return;
  const String requestId = document["request_id"].as<String>();
  String storageFolder = document["storage_folder"].as<String>();
  if (!storageFolder.length()) storageFolder = document["component_identifier"].as<String>();
  const String uploadUrl = document["upload_url"].as<String>();
  const String failureUrl = document["failure_url"].as<String>();
  if (!requestId.length() || !storageFolder.length() ||
      !uploadUrl.length() || !failureUrl.length()) return;

  String imagePath;
  if (!captureToSd(storageFolder, requestId, imagePath)) {
    reportFailure(failureUrl, "Falha ao capturar ou salvar a fotografia no microSD.");
    return;
  }

  pendingRequestId = requestId;
  pendingUploadUrl = uploadUrl;
  pendingImagePath = imagePath;
  pendingFailureMessage = "";
  if (!savePendingUpload()) {
    Serial.println("Não foi possível registrar o envio pendente no microSD.");
    reportFailure(failureUrl, "Falha ao registrar o envio pendente no microSD.");
    pendingRequestId = "";
    pendingUploadUrl = "";
    pendingImagePath = "";
    pendingFailureMessage = "";
    return;
  }
  lastUploadAttemptAt = millis() - UPLOAD_RETRY_INTERVAL_MS;
  uploadPendingImage();
}

void setup() {
  Serial.begin(115200);
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, LOW);
  if (String(DEVICE_TOKEN) == "MESMO_DEVICE_API_TOKEN_DO_ENV" ||
      String(CONFIG_PORTAL_PASSWORD) == "DEFINA_SENHA_DO_PORTAL" ||
      strlen(DEVICE_TOKEN) < 16 || strlen(CONFIG_PORTAL_PASSWORD) < 8 ||
      strlen(CONFIG_PORTAL_PASSWORD) > 63 ||
      !String(API_BASE_URL).startsWith("https://")) {
    Serial.println("Configure token, senha do portal e endereço HTTPS antes de usar.");
    while (true) delay(1000);
  }
  retryHardware();
  initializeFlash();
  WiFi.setSleep(false);
  startWiFiProvisioning();
  updateClock();
}

void loop() {
  connectWiFi();
  updateClock();
  retryHardware();
  pollRemoteConfig();
  sendHeartbeat();
  if (pendingRequestId.length()) {
    uploadPendingImage();
    delay(50);
    return;
  }
  pollCameraCommand();
  delay(50);
}
