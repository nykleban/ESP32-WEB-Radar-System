#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <LittleFS.h>

// wifi
static const char* STA_SSID = "MONY";
static const char* STA_PASS = "90300462";
static const char* AP_SSID  = "KLEBAN_RADAR";
static const char* AP_PASS  = "12345678";

// pins
static const int trigPin  = 5;
static const int echoPin  = 18;
static const int servoPin = 13;

static const int angleMin = 15;
static const int angleMax = 165;

// крок сервомотора: кожні 50мс рухаємось на 1°
static const uint32_t servoStepPeriodMs = 50;
// час на стабілізацію після руху
static const uint32_t servoSettleMs     = 45;

// таймаут ехо ~1м
static const uint32_t pulseTimeoutUs = 6000;
static const float    soundCmPerUs   = 0.0343f;

// поріг виявлення перешкоди (см)
static const int OBSTACLE_THRESHOLD_CM = 35;

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static Servo servoMotor;

static AsyncWebSocketClient* activeClient = nullptr;

static bool     staConnecting = false;
static uint32_t staStartMs    = 0;
static uint32_t staRetryMs    = 0;

static int      currentAngle    = 90;
static int      scanDir         = 1;
static uint32_t lastServoStepMs = 0;
static bool     waitingSettle   = false;
static uint32_t movedAtMs       = 0;

static uint32_t lastCleanupMs = 0;
static uint32_t lastLogMs     = 0;

// ──────────────────────────────────────────────
static void startSta() {
    WiFi.begin(STA_SSID, STA_PASS);
    staConnecting = true;
    staStartMs    = millis();
    Serial.printf("[WiFi] STA connecting to: %s\n", STA_SSID);
}

static void startWifi() {
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);

    // канал 8 = той самий що й MONY (з netsh scan)
    WiFi.softAP(AP_SSID, AP_PASS, 8);
    Serial.printf("[WiFi] AP started  IP: %s  ch:8\n",
                  WiFi.softAPIP().toString().c_str());

    startSta();
}

static void updateStaConnect() {
    uint32_t now = millis();

    if (staConnecting) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[WiFi] STA connected  IP: %s\n",
                          WiFi.localIP().toString().c_str());
            staConnecting = false;
            return;
        }
        if (now - staStartMs > 15000) {
            Serial.println("[WiFi] STA timeout — will retry in 30s");
            staConnecting = false;
            staRetryMs    = now;
        }
        return;
    }

    // якщо вже підключені — нічого не робимо
    if (WiFi.status() == WL_CONNECTED) return;

    // реконект через 30с після невдалої спроби
    if (now - staRetryMs >= 30000) {
        Serial.println("[WiFi] STA retry...");
        startSta();
    }
}

// ──────────────────────────────────────────────
static int measureDistanceCm() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    uint32_t dur = pulseIn(echoPin, HIGH, pulseTimeoutUs);
    if (dur == 0) return 999;
    return (int)lroundf((dur * soundCmPerUs) / 2.0f);
}

static void sendRadar(int angle, int distCm, int dir, uint32_t tms) {
    AsyncWebSocketClient* c = activeClient;
    if (!c || c->status() != WS_CONNECTED) return;
    if (!c->canSend() || c->queueIsFull()) return;

    char msg[64];
    int n = snprintf(msg, sizeof(msg),
                     "{\"a\":%d,\"d\":%d,\"dir\":%d,\"t\":%u}",
                     angle, distCm, dir, (unsigned)tms);
    if (n > 0) c->text(msg, (size_t)n);
}

// ──────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 Radar Starting ===");

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    if (!LittleFS.begin(true)) {
        Serial.println("[FS] LittleFS FAILED!");
        while (true) delay(100);
    }

    servoMotor.setPeriodHertz(50);
    servoMotor.attach(servoPin, 500, 2400);
    servoMotor.write(currentAngle);

    startWifi();

    ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient* client,
                  AwsEventType type, void*, uint8_t*, size_t) {
        if (type == WS_EVT_CONNECT) {
            Serial.printf("[WS] Connect  id=%u  ip=%s\n",
                          client->id(),
                          client->remoteIP().toString().c_str());
            client->client()->setNoDelay(true);
            activeClient = client;
        }
        if (type == WS_EVT_DISCONNECT) {
            Serial.printf("[WS] Disconnect  id=%u\n", client->id());
            if (activeClient && activeClient->id() == client->id())
                activeClient = nullptr;
        }
    });

    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(LittleFS, "/index.html", "text/html");
    });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(LittleFS, "/style.css", "text/css");
    });
    server.begin();
    Serial.println("[HTTP] Server started");
}

void loop() {
    updateStaConnect();

    uint32_t now = millis();

    // очистка WS клієнтів раз на 500мс
    if (now - lastCleanupMs >= 500) {
        ws.cleanupClients();
        lastCleanupMs = now;
    }

    // крок сервомотора
    if (!waitingSettle && (now - lastServoStepMs >= servoStepPeriodMs)) {
        lastServoStepMs = now;

        if (currentAngle >= angleMax) scanDir = -1;
        if (currentAngle <= angleMin) scanDir = +1;
        currentAngle += scanDir;

        servoMotor.write(currentAngle);
        movedAtMs    = now;
        waitingSettle = true;
    }

    // вимір після стабілізації
    if (waitingSettle && (now - movedAtMs >= servoSettleMs)) {
        waitingSettle = false;

        int dist = measureDistanceCm();

        // ── логування перешкоди ──
        if (dist < OBSTACLE_THRESHOLD_CM) {
            Serial.printf("⚠  OBSTACLE  angle=%3d°  dist=%3d cm\n",
                          currentAngle, dist);
        }

        // ── загальний лог раз на секунду ──
        if (now - lastLogMs >= 1000) {
            lastLogMs = now;
            Serial.printf("[RADAR] angle=%d° dist=%dcm dir=%+d clients=%u sta=%s\n",
                          currentAngle, dist, scanDir, ws.count(),
                          WiFi.status() == WL_CONNECTED
                              ? WiFi.localIP().toString().c_str()
                              : "no");
        }

        sendRadar(currentAngle, dist, scanDir, movedAtMs);
    }
}