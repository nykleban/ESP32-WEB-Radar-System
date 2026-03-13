#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <LittleFS.h>

// wifi
static const char* STA_SSID = "nykleban";
static const char* STA_PASS = "1234567z";

static const char* AP_SSID  = "KLEBAN_RADAR";
static const char* AP_PASS  = "12345678";

// pins
static const int trigPin  = 5;
static const int echoPin  = 18;
static const int servoPin = 13;

static const int angleMin = 15;
static const int angleMax = 165;

static const uint32_t servoStepPeriodMs = 70;


static const uint32_t servoSettleMs = 60;


// timeout echo
static const uint32_t pulseTimeoutUs = 6000;

// speed of sound cm/us
static const float soundCmPerUs = 0.0343f;

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static Servo servoMotor;

static AsyncWebSocketClient* activeClient = nullptr;
static uint32_t seqId = 0;

static bool staConnecting = false;
static uint32_t staStartMs = 0;

static int currentAngle = 90;
static int scanDir = 1;

static uint32_t lastServoStepMs = 0;
// це прапорець чи серво вже отримало нову позицію
static bool waitingSettle = false;

// момент, коли серво реально отримало новий кут (для коректного t)
static uint32_t movedAtMs = 0;

static uint32_t lastLogMs = 0;
static uint32_t lastCleanupMs = 0;

static void startWifi() {
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);

    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());

    WiFi.begin(STA_SSID, STA_PASS);
    staConnecting = true;
    staStartMs = millis();
    Serial.printf("STA connecting to: %s\n", STA_SSID);
}

static void updateStaConnect() {
    if(!staConnecting) return;

    if(WiFi.status() == WL_CONNECTED) {
        Serial.printf("STA OK IP: %s\n", WiFi.localIP().toString().c_str());
        staConnecting = false;
        return;
    }

    if(millis() - staStartMs > 12000) {
        Serial.println("STA timeout. Using AP only.");
        staConnecting = false;
    }
}

static int measureDistanceCm() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    uint32_t durationUs = pulseIn(echoPin, HIGH, pulseTimeoutUs);
    if(durationUs == 0) return 999;

    float cm = (durationUs * soundCmPerUs) / 2.0f;
    return (int)lroundf(cm);
}
struct RadarPkt {
    int angle;
    int dist;
    int dir;
    uint32_t tms;
};

static void sendRadar(int angle, int distanceCm, int dir, uint32_t tms) {
    AsyncWebSocketClient* c = activeClient;
    if(!c || c->status() != WS_CONNECTED) return;

    static uint32_t lastSentMs = 0;
    uint32_t now = millis();
    if((uint32_t)(now - lastSentMs) < 70) return; 
    lastSentMs = now;

    if(!c->canSend() || c->queueIsFull()) return;

    char msg[64];
    int n = snprintf(msg, sizeof(msg), "{\"a\":%d,\"d\":%d,\"dir\":%d,\"t\":%u}", angle, distanceCm, dir, (unsigned)tms);
    if(n <= 0) return;

    c->text(msg, (size_t)n);
}


void setup() {
    Serial.begin(115200);
    Serial.println("Starting ESP32 Radar...");

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    if(!LittleFS.begin(true)) {
        Serial.println("LittleFS Failed!");
        while(true) delay(100);
    }

    servoMotor.setPeriodHertz(50);
    servoMotor.attach(servoPin, 500, 2400);
    servoMotor.write(currentAngle);

    startWifi();

    ws.onEvent([](AsyncWebSocket * server, AsyncWebSocketClient * client,
    AwsEventType type, void *arg, uint8_t *data, size_t len) {

        if(type == WS_EVT_CONNECT) {
            Serial.printf("WS connect id=%u ip=%s\n",
                          client->id(),
                          client->remoteIP().toString().c_str());

            activeClient = client;
            client->client()->setNoDelay(true);
        }

        if(type == WS_EVT_DISCONNECT) {
            Serial.printf("WS disconnect id=%u\n", client->id());
            if(activeClient && activeClient->id() == client->id()) {
                activeClient = nullptr;
            }
        }
    });

    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(LittleFS, "/style.css", "text/css");
    });

    server.begin();
}

void loop() {
    updateStaConnect();

    uint32_t nowMs = millis();

    if(nowMs - lastCleanupMs >= 500) {
        ws.cleanupClients();
        lastCleanupMs = nowMs;
    }

    if(!waitingSettle && (nowMs - lastServoStepMs >= servoStepPeriodMs)) {
        lastServoStepMs = nowMs;

        if(currentAngle >= angleMax) scanDir = -1;
        if(currentAngle <= angleMin) scanDir = +1;

        currentAngle += scanDir;
        servoMotor.write(currentAngle);

        movedAtMs = nowMs;
        waitingSettle = true;
    }

    //після settle робимо вимір і відправляємо
    if(waitingSettle && (nowMs - movedAtMs >= servoSettleMs)) {
        waitingSettle = false;

        int distanceCm = measureDistanceCm();

        sendRadar(currentAngle, distanceCm, scanDir, movedAtMs);

        if(nowMs - lastLogMs >= 1000) {
            lastLogMs = nowMs;
            Serial.printf("Angle=%d Dist=%dcm Clients=%u Active=%s\n",
                          currentAngle, distanceCm, ws.count(),
                          activeClient ? "yes" : "no");
        }
    }

}
