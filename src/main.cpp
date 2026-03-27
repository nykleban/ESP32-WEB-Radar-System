#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <LittleFS.h>

// ── Налаштування WiFi ──────────────────────────────────────────────────────
static const char* STA_SSID = "MONY";
static const char* STA_PASS = "90300462";
static const char* AP_SSID  = "KLEBAN_RADAR";
static const char* AP_PASS  = "12345678";

// ── Піни ──────────────────────────────────────────────────────────────────
static const int trigPin  = 5;
static const int echoPin  = 18;
static const int servoPin = 13;

// ── Діапазон кутів сервомотора ────────────────────────────────────────────
static const int angleMin = 15;
static const int angleMax = 165;

// ── Таймінг сканування ────────────────────────────────────────────────────
// 65 мс = 50 мс кроку + ~15 мс запас для трьох вимірів медіанного фільтра
static const uint32_t servoStepPeriodMs = 65;
// Час стабілізації сервомотора після переміщення на 1°
static const uint32_t servoSettleMs     = 45;

// ── Ультразвуковий датчик ─────────────────────────────────────────────────
// Таймаут ехо-сигналу (~1 м → 5800 мкс, беремо 6000 мкс із запасом)
static const uint32_t pulseTimeoutUs = 6000;
static const float    soundCmPerUs   = 0.0343f;

// ── Зони виявлення (см) ───────────────────────────────────────────────────
// Зона 2 — критична небезпека (червона)
static const int ZONE_CRITICAL_CM = 15;
// Зона 1 — попередження (помаранчева)
static const int ZONE_WARNING_CM  = 35;
// Зона 0 — безпечно / відсутність об'єкта (зелена або нічого)

// ─────────────────────────────────────────────────────────────────────────
// ISR для безблокуючого вимірювання відлуння
//
// Замість блокуючого pulseIn() (блокує ЦП до 6 мс) використовується
// апаратне переривання на обидва фронти сигналу (CHANGE).
// ISR фіксує мікросекундні мітки початку і кінця ехо-імпульсу.
// Основний цикл не блокується: readEchoCm() чекає з yield(), що дозволяє
// планувальнику FreeRTOS виконувати WiFi- та WebSocket-задачі.
// ─────────────────────────────────────────────────────────────────────────
volatile uint32_t echoStartUs   = 0;
volatile uint32_t echoDurUs     = 0;
volatile bool     echoReady     = false;
volatile bool     echoMeasuring = false;

void IRAM_ATTR echoISR() {
    if (digitalRead(echoPin) == HIGH) {
        // Передній фронт: запускаємо таймер
        echoStartUs   = micros();
        echoMeasuring = true;
        echoReady     = false;
    } else if (echoMeasuring) {
        // Задній фронт: фіксуємо тривалість ехо-імпульсу
        echoDurUs     = micros() - echoStartUs;
        echoReady     = true;
        echoMeasuring = false;
    }
}

// ── Об'єкти та змінні стану ───────────────────────────────────────────────
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static Servo          servoMotor;

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

// ── WiFi ───────────────────────────────────────────────────────────────────
static void startSta() {
    WiFi.begin(STA_SSID, STA_PASS);
    staConnecting = true;
    staStartMs    = millis();
    Serial.printf("[WiFi] STA connecting to: %s\n", STA_SSID);
}

static void startWifi() {
    WiFi.disconnect(true, true);
    // Коротка затримка для гарантованого скидання WiFi-стека ESP32.
    // Знаходиться в setup(), тому не блокує основний робочий цикл loop().
    delay(100);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);

    // Канал AP тепер обирається автоматично.
    // Раніше канал 8 був захардкоджений (відповідав домашньому роутеру),
    // але при зміні каналу роутера STA не міг підключитись.
    // ESP32 автоматично узгодить канал AP із каналом STA після підключення.
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WiFi] AP started  IP: %s\n",
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
            Serial.println("[WiFi] STA timeout -- will retry in 30s");
            staConnecting = false;
            staRetryMs    = now;
        }
        return;
    }
    if (WiFi.status() == WL_CONNECTED) return;
    if (now - staRetryMs >= 30000) {
        Serial.println("[WiFi] STA retry...");
        startSta();
    }
}

// ── Ультразвуковий датчик ─────────────────────────────────────────────────

/**
 * Генерує один ультразвуковий імпульс (10 мкс HIGH на TRIG).
 * ISR echoISR() апаратно фіксує відповідь на піні ECHO.
 */
static void triggerPulse() {
    echoReady     = false;
    echoMeasuring = false;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
}

/**
 * Чекає результату ISR і повертає відстань у сантиметрах.
 *
 * Використовує yield() замість busy-wait: планувальник FreeRTOS отримує
 * керування під час очікування ехо, що дозволяє WiFi- та TCP-стеку
 * продовжувати роботу без затримок (критично для AsyncWebServer).
 *
 * Повертає 999, якщо ехо не прийшло в межах pulseTimeoutUs (об'єкт
 * поза зоною дії або відбиття відсутнє).
 */
static int readEchoCm() {
    uint32_t deadline = micros() + pulseTimeoutUs + 500;
    while (!echoReady) {
        if (micros() > deadline) return 999;
        yield();
    }
    uint32_t dur = echoDurUs;
    if (dur == 0 || dur > pulseTimeoutUs) return 999;
    return (int)lroundf((dur * soundCmPerUs) / 2.0f);
}

/**
 * Медіанний фільтр з трьох вимірів.
 *
 * Датчик HC-SR04 схильний до хибних спрацювань через:
 *   - відбиття під кутом >15° від поверхні
 *   - перехресні акустичні завади
 *   - вібрацію механічних елементів
 *
 * Три послідовних виміри з паузою 300 мкс між ними → сортування мережею
 * (3 порівняння, оптимально для 3 елементів) → повертається медіана.
 * Загальний час: ≤ 18 мс (вміщується в 45 мс servoSettleMs).
 */
static int measureFiltered() {
    int s[3];
    for (int i = 0; i < 3; i++) {
        triggerPulse();
        s[i] = readEchoCm();
        if (i < 2) delayMicroseconds(300);
    }
    // Сортуюча мережа для 3 елементів (3 порівняння, без циклів)
    if (s[0] > s[1]) { int t = s[0]; s[0] = s[1]; s[1] = t; }
    if (s[1] > s[2]) { int t = s[1]; s[1] = s[2]; s[2] = t; }
    if (s[0] > s[1]) { int t = s[0]; s[0] = s[1]; s[1] = t; }
    return s[1];  // медіана
}

// ── WebSocket ─────────────────────────────────────────────────────────────

/**
 * Надсилає JSON-пакет УСІМ підключеним WebSocket-клієнтам (broadcast).
 *
 * Раніше використовувався один вказівник activeClient — це означало,
 * що лише останній підключений браузер отримував дані.
 * ws.textAll() надсилає пакет усім активним клієнтам одночасно.
 *
 * Формат пакету:
 *   { "a": кут°, "d": відстань_см, "dir": напрямок, "t": час_мс, "z": зона }
 *
 * Поле "z" (зона):
 *   0 — безпечно або об'єкт відсутній
 *   1 — попередження (об'єкт < ZONE_WARNING_CM)
 *   2 — критично   (об'єкт < ZONE_CRITICAL_CM)
 */
static void sendRadar(int angle, int distCm, int dir, uint32_t tms) {
    if (ws.count() == 0) return;

    int zone = (distCm >= 999)             ? 0 :
               (distCm < ZONE_CRITICAL_CM) ? 2 :
               (distCm < ZONE_WARNING_CM)  ? 1 : 0;

    char msg[80];
    int n = snprintf(msg, sizeof(msg),
                     "{\"a\":%d,\"d\":%d,\"dir\":%d,\"t\":%u,\"z\":%d}",
                     angle, distCm, dir, (unsigned)tms, zone);
    if (n > 0 && n < (int)sizeof(msg))
        ws.textAll(msg, (size_t)n);
}

// ── Setup ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 Radar Starting ===");

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    // Підключаємо ISR на обидва фронти (CHANGE = RISING + FALLING)
    attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);

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
            Serial.printf("[WS] Connect    id=%u  ip=%s  total=%u\n",
                          client->id(),
                          client->remoteIP().toString().c_str(),
                          ws.count());
            client->client()->setNoDelay(true);
        }
        if (type == WS_EVT_DISCONNECT) {
            Serial.printf("[WS] Disconnect id=%u  total=%u\n",
                          client->id(), ws.count());
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

// ── Loop ───────────────────────────────────────────────────────────────────
void loop() {
    updateStaConnect();
    uint32_t now = millis();

    // Очистка відключених WS-клієнтів раз на 500 мс
    if (now - lastCleanupMs >= 500) {
        ws.cleanupClients();
        lastCleanupMs = now;
    }

    // Крок сервомотора: рухаємось на 1° кожні servoStepPeriodMs мс
    if (!waitingSettle && (now - lastServoStepMs >= servoStepPeriodMs)) {
        lastServoStepMs = now;

        if (currentAngle >= angleMax) scanDir = -1;
        if (currentAngle <= angleMin) scanDir = +1;
        currentAngle += scanDir;

        servoMotor.write(currentAngle);
        movedAtMs     = now;
        waitingSettle = true;
    }

    // Вимір після стабілізації сервомотора
    if (waitingSettle && (now - movedAtMs >= servoSettleMs)) {
        waitingSettle = false;

        int dist = measureFiltered();

        // Логування за зонами (три рівні тривоги)
        if (dist < ZONE_CRITICAL_CM) {
            Serial.printf("[!!] CRITICAL  angle=%3d  dist=%3d cm\n",
                          currentAngle, dist);
        } else if (dist < ZONE_WARNING_CM) {
            Serial.printf("[~]  WARNING   angle=%3d  dist=%3d cm\n",
                          currentAngle, dist);
        }

        // Загальний лог раз на секунду
        if (now - lastLogMs >= 1000) {
            lastLogMs = now;
            Serial.printf("[RADAR] ang=%d dir=%+d dist=%dcm clients=%u sta=%s\n",
                          currentAngle, scanDir, dist, ws.count(),
                          WiFi.status() == WL_CONNECTED
                              ? WiFi.localIP().toString().c_str()
                              : "no");
        }

        sendRadar(currentAngle, dist, scanDir, movedAtMs);
    }
}
