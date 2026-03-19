#include <Arduino.h>

//================================================================
// ライブラリ
//================================================================
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <time.h>
#include <stdlib.h>
#include <WiFiNTP.h>
#include <DFRobot_DHT20.h>
#include <DHT.h>           // DHT22屋外温度センサー用
#include <SparkFun_AS3935.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <LittleFS.h>
#include <math.h> // for round()
#include <ArduinoOTA.h>
#include <pico/mutex.h>
#include "hardware/watchdog.h" // Watchdog機能のために追加

// ★設定ファイルを読み込む
#include "config.h"
#undef DEBUG_PRINT
#undef DEBUG_PRINTLN
#define SUPPRESS_SEND_PWM_BY_TIMER_INFO
#define NO_LED_FEEDBACK_CODE
#define SEND_PWM_BY_TIMER
#include <IRremote.hpp>
#include "my_remotes.h"

// ★Mutexの定義
auto_init_mutex(data_mutex); // 共有データ保護用
auto_init_mutex(wifi_mutex); // Wi-Fiアクセス保護用

// ★Watchdogとフリーズ検出関連
#define CRASH_MAGIC_VALUE 0xDEADBEEF
const unsigned long FREEZE_TIMEOUT_MS = 15000; // v19.5.0: 10秒→15秒に延長（WiFi/HTTP処理中の誤検出防止）
volatile unsigned long lastCore0Pet = 0;

// ★ Watchdog pet のインラインヘルパー (Core0がmutex待ち中でも安全に呼べる)
static inline void petWatchdog() { lastCore0Pet = millis(); }

// ★コア間通信用の変数
volatile bool core0_hw_init_complete = false;
volatile bool core1_wifi_complete = false;
volatile int wolTriggerAction = 0; // 0:なし, 1:Desktop, 2:Server

// ★Core0からCore1へのアクションリクエスト用フラグ
volatile bool requestManualLog = false;
volatile bool requestGetSwitchbotIds = false;
volatile bool requestSendLineTest = false;
volatile bool requestResyncNtp = false;
volatile bool requestSendLightningAlert = false;
volatile int lightningDistanceForAlert = 0;
volatile bool requestSendWolDesktop = false;
volatile bool requestSendWolServer = false;
volatile bool requestGetUltrasonicDistance = false;
volatile bool requestSendDistanceStop = false;    // Core0→Core1: 子機に/distance_stopを送信
volatile bool requestSendDistanceStart = false;   // Core0→Core1: 子機に/distance_startを送信
volatile bool requestExitUltrasonic = false;      // Core1→Core0: ULTRASONIC_MONITORを終了導指

// IR APIリモコン送信用 (Core1がセット → Core0が処理)
volatile char irSendDevice[32] = "";
volatile char irSendCommand[32] = "";
volatile bool irSendRequested = false;

// SwitchBotコマンドキュー: Core0からのリクエストをCore1が実行
struct SwitchBotPendingCmd {
    char deviceId[64];
    char command[32];
    char parameter[64];
    char commandType[16];
    bool pending;
} pendingSwitchBotCmd = {"", "", "", "command", false};

volatile bool requestSendBathClear = false; // Core0→Core1: 子機に /bath_clear を送信

// ★Core1からCore0への情報共有用バッファ
volatile char sharedIpAddress[16] = "Connecting...";

//================================================================
// ★★★ 設定項目 (本体の動作設定) ★★★
//================================================================
// --- 雷センサー感度設定 ---
const uint8_t LIGHTNING_WATCHDOG_THRESHOLD = 1; // 雷検知の強さの閾値 (調整範囲: 1～10)
const uint8_t LIGHTNING_SPIKE_REJECTION = 1;    // 電気的ノイズの除去レベル (調整範囲: 1～11)
const uint8_t INITIAL_NOISE_LEVEL = 1;          // 周囲の環境ノイズの初期レベル (調整範囲: 1～7)

// --- 動作設定 ---
const unsigned long LONG_PRESS_DURATION_MS = 1000;
const unsigned long INACTIVITY_TIMEOUT_MS = 5000;
const unsigned long BACKLIGHT_DURATION_MS = 3000;
const unsigned long BACKLIGHT_FADE_MS = 100;        // v19.0.0: バックライトのフェード時間（0.1秒）
const unsigned long SENSOR_READ_INTERVAL_MS = 3000; // 温湿度センサーの読み取り間隔
const unsigned long IP_DISPLAY_DURATION_MS = 5000;
const int HISTORY_SIZE = 3;
const int LCD_COLS = 20;
const int LCD_ROWS = 4;

//================================================================
// ピン定義
//================================================================
namespace Pins
{
    const int LCD_RS = 2, LCD_E = 3, LCD_D4 = 4, LCD_D5 = 5, LCD_D6 = 6, LCD_D7 = 7;
    const int LCD_BACKLIGHT = 14;
    const int LED_R = 28, LED_G = 27, LED_B = 26;
    const int BUTTON = 15;
    const int I2C_SDA = 0, I2C_SCL = 1;
    const int LIGHTNING_IRQ = 16;
    const int DHT22_PIN = 17;  // DHT22屋外温度センサー
    const int AS3935_ADDR = 0x03;
    const int IR_RECEIVE_PIN = 18;
    const int IR_SEND_PIN = 19;
}

//================================================================
// グローバル状態管理
//================================================================
namespace State
{
    // --- 動作モード ---
    enum Mode
    {
        MAIN_DISPLAY,
        MENU,
        HISTORY,
        SWITCHBOT_APPLIANCE_SELECT,
        DEVICE_CONTROL,
        WAKE_ON_LAN,
        ULTRASONIC_MONITOR,
        SENSOR_DIAGNOSTICS,
        IR_APPLIANCE_SELECT,
        IR_DEVICE_CONTROL,
        IR_RECEIVE_MODE
    };

    // --- メニューの状態 ---
    struct MenuState
    {
        Mode currentMode = MAIN_DISPLAY;
        int menuSelection = 0;
        int deviceSelection = 0;
        int commandSelection = 0;
    };

    // --- システムの状態 ---
    struct SystemState
    {
        bool illuminationOn = false;
        bool backlightAlwaysOn = false;
        bool ntpInitialized = false;
        bool isAutoResync = false;
        bool needsRedraw = true;
        bool forceMainScreenRedraw = true;
        uint8_t currentNoiseLevel = 2;
        bool initialLogSent = false;
        bool dht20_initialized = false;
        bool dht22_outdoor_initialized = false;  // DHT22屋外センサー初期化フラグ
        bool firstHttpAccessBlinkDone = false;   // 初回HTTPアクセス時LED点滅済みフラグ
        bool as3935_initialized = false;
        int displayMode = 0;
        bool displayModeManuallySet = false;

        // v19.0.0: バックライトPWM制御用
        bool backlightFading = false;
        bool backlightTargetOn = false;
        unsigned long backlightFadeStartTime = 0;
        int backlightCurrentBrightness = 0;
        
        bool forceClockRefresh = false;
        bool debugModeRuntime = DEBUG; // 初期値はconfig.hのDEBUGに従う
        int irLearningApplianceIndex = -1;
        int irLearningButtonIndex = -1;
        bool irLearningComplete = false;
        bool irLearningWaitingSerial = true;
        bool bathAlertActive = false;    // お風呂通知アクティブフラグ
    };

    // --- センサーデータ ---
    struct SensorData
    {
        float temperature = -999.0;
        float humidity = -999.0;
        float outdoorTemperature = -999.0;  // DHT22屋外温度
        float outdoorHumidity = -999.0;     // DHT22屋外湿度
        float ultrasonicDistance = -1.0;
        char lastEventTime[20] = "N/A";
        String lastEventType = "None"; // "Lightning" or "Noise"
        int lastLightningDistance = -1;
    };

    // --- 本体メモリに記録する雷履歴 ---
    struct EventRecord
    {
        char timestamp[20];
        String type;
        int distance;
    };

    struct HistoryState
    {
        EventRecord records[HISTORY_SIZE];
        int index = 0;
        int count = 0;
    };

    // --- タイマー ---
    struct TimerState
    {
        unsigned long lastActivity = 0;
        unsigned long backlightOn = 0;
        // フルスクリーンメッセージ用
        char timedMessageLine0[LCD_COLS + 1] = "";
        char timedMessageLine1[LCD_COLS + 1] = "";
        unsigned long timedMessageEndTime = 0;
        bool returnToMainAfterMessage = false;
        bool messageNeedsRedraw = false; // メッセージの再描画要求フラグ
        // 2行目上書きメッセージ用
        char timedOverlayLine1[LCD_COLS + 1] = "";
        unsigned long timedOverlayEndTime = 0;
        
        // LED点滅用 (ノンブロッキング)
        bool ledBlinkActive = false;
        unsigned long ledBlinkNextToggle = 0;
        int ledBlinkRemainingToggles = 0;
        int ledBlinkInterval = 0;
        String ledBlinkColor = "";
        bool ledCurrentState = false;
        bool ledBlinkInfinite = false;   // trueなら無限点滅 (お風呂通知用)
        unsigned long lastDistanceUpdateTime = 0; // 子機からの距離プッシュ最終受信時刻
    };

    // --- 暗号化コンテキスト ---
    struct SHA256_CTX
    {
        uint8_t data[64];
        uint32_t datalen;
        uint64_t bitlen;
        uint32_t state[8];
    };

    // --- グローバルオブジェクトと状態変数のインスタンス ---
    MenuState menu;
    SystemState system;
    SensorData sensors;
    HistoryState history;
    TimerState timers;

    LiquidCrystal lcd(Pins::LCD_RS, Pins::LCD_E, Pins::LCD_D4, Pins::LCD_D5, Pins::LCD_D6, Pins::LCD_D7);
    DFRobot_DHT20 dht20;
    SparkFun_AS3935 lightning(Pins::AS3935_ADDR);
    WiFiServer server(80);
    String childIpAddress = "";
    volatile bool lightningInterruptFlag = false;
}

// DHT22屋外温度センサー（グローバルオブジェクト）
DHT g_dht22(Pins::DHT22_PIN, DHT22);
const float DHT22_TEMP_OFFSET = 0.0;   // 温度キャリブレーションオフセット
const float DHT22_HUM_OFFSET = -15.0;  // 湿度キャリブレーションオフセット

// DHT22の安定化設定
const unsigned long DHT22_MIN_READ_INTERVAL_MS = 2500; // DHT22の最小読み取り間隔（2.5秒に設定）
const int DHT22_MAX_RETRY = 0;  // 読み取りリトライ回数（Core0ブロック防止のため0に設定）
const unsigned long DHT22_RETRY_DELAY_MS = 100; // リトライ間の待機時間
static unsigned long lastDHT22ReadTime = 0; // 前回の読み取り時刻
static float lastValidTemp = -999.0;  // 前回の有効な温度値
static float lastValidHum = -999.0;   // 前回の有効な湿度値

//================================================================
// 暗号化ユーティリティ (編集不要)
//================================================================
namespace Crypto
{
    const uint32_t K[64] = {0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); }
    void sha256_transform(State::SHA256_CTX *ctx, const uint8_t data[])
    {
        uint32_t a, b, c, d, e, f, g, h, i, j, t1, t2, m[64];
        for (i = 0, j = 0; i < 16; ++i, j += 4)
            m[i] = (data[j] << 24) | (data[j + 1] << 16) | (data[j + 2] << 8) | (data[j + 3]);
        for (; i < 64; ++i)
            m[i] = (((m[i - 2] >> 17) | (m[i - 2] << 15)) ^ ((m[i - 2] >> 19) | (m[i - 2] << 13)) ^ (m[i - 2] >> 10)) + m[i - 7] + (((m[i - 15] >> 7) | (m[i - 15] << 25)) ^ ((m[i - 15] >> 18) | (m[i - 15] << 14)) ^ (m[i - 15] >> 3)) + m[i - 16];
        a = ctx->state[0];
        b = ctx->state[1];
        c = ctx->state[2];
        d = ctx->state[3];
        e = ctx->state[4];
        f = ctx->state[5];
        g = ctx->state[6];
        h = ctx->state[7];
        for (i = 0; i < 64; ++i)
        {
            t1 = h + (((e >> 6) | (e << 26)) ^ ((e >> 11) | (e << 21)) ^ ((e >> 25) | (e << 7))) + ((e & f) ^ (~e & g)) + K[i] + m[i];
            t2 = (((a >> 2) | (a << 30)) ^ ((a >> 13) | (a << 19)) ^ ((a >> 22) | (a << 10))) + ((a & b) ^ (a & c) ^ (b & c));
            h = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }
        ctx->state[0] += a;
        ctx->state[1] += b;
        ctx->state[2] += c;
        ctx->state[3] += d;
        ctx->state[4] += e;
        ctx->state[5] += f;
        ctx->state[6] += g;
        ctx->state[7] += h;
    }
    void sha256_init(State::SHA256_CTX *ctx)
    {
        ctx->datalen = 0;
        ctx->bitlen = 0;
        ctx->state[0] = 0x6a09e667;
        ctx->state[1] = 0xbb67ae85;
        ctx->state[2] = 0x3c6ef372;
        ctx->state[3] = 0xa54ff53a;
        ctx->state[4] = 0x510e527f;
        ctx->state[5] = 0x9b05688c;
        ctx->state[6] = 0x1f83d9ab;
        ctx->state[7] = 0x5be0cd19;
    }
    void sha256_update(State::SHA256_CTX *ctx, const uint8_t data[], size_t len)
    {
        for (uint32_t i = 0; i < len; ++i)
        {
            ctx->data[ctx->datalen] = data[i];
            ctx->datalen++;
            if (ctx->datalen == 64)
            {
                sha256_transform(ctx, ctx->data);
                ctx->bitlen += 512;
                ctx->datalen = 0;
            }
        }
    }
    void sha256_final(State::SHA256_CTX *ctx, uint8_t hash[])
    {
        uint32_t i = ctx->datalen;
        if (ctx->datalen < 56)
        {
            ctx->data[i++] = 0x80;
            while (i < 56)
                ctx->data[i++] = 0x00;
        }
        else
        {
            ctx->data[i++] = 0x80;
            while (i < 64)
                ctx->data[i++] = 0x00;
            sha256_transform(ctx, ctx->data);
            memset(ctx->data, 0, 56);
        }
        ctx->bitlen += ctx->datalen * 8;
        ctx->data[63] = ctx->bitlen;
        ctx->data[62] = ctx->bitlen >> 8;
        ctx->data[61] = ctx->bitlen >> 16;
        ctx->data[60] = ctx->bitlen >> 24;
        ctx->data[59] = ctx->bitlen >> 32;
        ctx->data[58] = ctx->bitlen >> 40;
        ctx->data[57] = ctx->bitlen >> 48;
        ctx->data[56] = ctx->bitlen >> 56;
        sha256_transform(ctx, ctx->data);
        for (i = 0; i < 4; ++i)
        {
            hash[i] = (ctx->state[0] >> (24 - i * 8)) & 0xff;
            hash[i + 4] = (ctx->state[1] >> (24 - i * 8)) & 0xff;
            hash[i + 8] = (ctx->state[2] >> (24 - i * 8)) & 0xff;
            hash[i + 12] = (ctx->state[3] >> (24 - i * 8)) & 0xff;
            hash[i + 16] = (ctx->state[4] >> (24 - i * 8)) & 0xff;
            hash[i + 20] = (ctx->state[5] >> (24 - i * 8)) & 0xff;
            hash[i + 24] = (ctx->state[6] >> (24 - i * 8)) & 0xff;
            hash[i + 28] = (ctx->state[7] >> (24 - i * 8)) & 0xff;
        }
    }
    void hmac_sha256(const uint8_t *key, size_t keylen, const uint8_t *data, size_t datalen, uint8_t *out)
    {
        State::SHA256_CTX ctx;
        uint8_t k_ipad[65], k_opad[65], tk[32];
        memset(k_ipad, 0, sizeof(k_ipad));
        memset(k_opad, 0, sizeof(k_opad));
        if (keylen > 64)
        {
            sha256_init(&ctx);
            sha256_update(&ctx, key, keylen);
            sha256_final(&ctx, tk);
            key = tk;
            keylen = 32;
        }
        memcpy(k_ipad, key, keylen);
        memcpy(k_opad, key, keylen);
        for (int i = 0; i < 64; i++)
        {
            k_ipad[i] ^= 0x36;
            k_opad[i] ^= 0x5c;
        }
        sha256_init(&ctx);
        sha256_update(&ctx, k_ipad, 64);
        sha256_update(&ctx, data, datalen);
        sha256_final(&ctx, out);
        sha256_init(&ctx);
        sha256_update(&ctx, k_opad, 64);
        sha256_update(&ctx, out, 32);
        sha256_final(&ctx, out);
    }
    String base64_encode(const uint8_t *data, size_t len)
    {
        static const char *b64_table = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        String ret;
        ret.reserve((len + 2) / 3 * 4);
        for (size_t i = 0; i < len; i += 3)
        {
            uint32_t octet_a = i < len ? data[i] : 0;
            uint32_t octet_b = i + 1 < len ? data[i + 1] : 0;
            uint32_t octet_c = i + 2 < len ? data[i + 2] : 0;
            uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;
            ret += b64_table[(triple >> 3 * 6) & 0x3F];
            ret += b64_table[(triple >> 2 * 6) & 0x3F];
            ret += b64_table[(triple >> 1 * 6) & 0x3F];
            ret += b64_table[(triple >> 0 * 6) & 0x3F];
        }
        if (len % 3 == 1)
        {
            ret[ret.length() - 1] = '=';
            ret[ret.length() - 2] = '=';
        }
        if (len % 3 == 2)
        {
            ret[ret.length() - 1] = '=';
        }
        return ret;
    }
}

//================================================================
// プロトタイプ宣言
//================================================================
namespace Menu
{
    void changeMode(State::Mode newMode);
}
namespace Display
{
    void printLcdLine(int line, const char *text);
    void updateMainDisplay();
    void showTimedMessage(const char *line0, const char *line1, unsigned long duration, bool returnToMain);
}
namespace Network
{
    void init();
    void update();
    void sendSwitchBotCommand(const char *deviceId, const char *command, const char *parameter, const char *commandType = "command");
    void sendWakeOnLan(const char *macStr);
    void requestDistance_internal();
    bool connectToWiFi();
    void logDataToGoogleSheet(String params);
    void handleWolPolling();
    void getSwitchBotDeviceList_internal();
    void sendLineTestMessage_internal();
}
namespace Utils
{
    void toggleBacklightMode();
    void toggleIlluminationMode();
    void rebootDevice();
    void requestBlinkLED(String color, int times, int duration); // ノンブロッキング版に変更
    void requestBlinkLEDInfinite(String color, int intervalMs);  // 無限点滅 (お風呂通知用)
    void stopLEDBlink();                                          // 点滅停止 (お風呂通知クリア用)
    void handleLEDBlink(); // ループ内で呼ぶ
}
namespace Sensors
{
    void init();
    void calibrateSensor();
    void updateDht();
    void handleLightningInterrupt();
}
void handlePeriodicTasks();
void error_loop();

namespace IRRemote
{
    void init()
    {
        IrSender.begin(Pins::IR_SEND_PIN);
        IrReceiver.begin(Pins::IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
    }

    void sendIrSignal(const IrSignal *signal)
    {
        if (signal == nullptr) return;

        if (signal->type == IR_TYPE_RAW)
        {
            if (signal->rawData != nullptr && signal->rawLen > 0)
            {
                uint16_t *rawBuffer = new uint16_t[signal->rawLen];
                if (rawBuffer != nullptr)
                {
                    for (size_t i = 0; i < signal->rawLen; i++)
                    {
                        rawBuffer[i] = pgm_read_word(&signal->rawData[i]);
                    }
                    IrSender.sendRaw(rawBuffer, signal->rawLen, 38);
                    delay(50);
                    IrSender.sendRaw(rawBuffer, signal->rawLen, 38);
                    delete[] rawBuffer;
                }
            }
        }
        else if (signal->type == IR_TYPE_DECODED)
        {
            switch (signal->protocol)
            {
            case NEC:
                IrSender.sendNEC(signal->address, signal->command, 0);
                break;
            case KASEIKYO:
                IrSender.sendKaseikyo_Denon(signal->address, signal->command, 0);
                break;
            default:
                break;
            }
        }
    }

    void receiveIrSignal()
    {
        if (IrReceiver.decode())
        {
            Serial.println("\n=== IR Signal Received ===");
            if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)
            {
                Serial.println("Repeat Signal");
            }
            else
            {
                IrReceiver.printIRResultShort(&Serial);
                IrReceiver.printIRSendUsage(&Serial);
                
                Serial.println("\n// Copy for my_remotes.h:");
                Serial.print("const uint16_t rawData_NEW[] PROGMEM = ");
                IrReceiver.printIRResultRawFormatted(&Serial, false);
                Serial.println(";");
            }
            Serial.println("==========================\n");
            IrReceiver.resume();
        }
    }

    void handle()
    {
        if (State::menu.currentMode == State::IR_RECEIVE_MODE)
        {
            receiveIrSignal();
        }
    }
}

//================================================================
// メニュー定義
//================================================================
namespace Menu
{
    struct MenuItem
    {
        const char *text;
        void (*action)();
    };

    void enterHistory() { changeMode(State::HISTORY); }
    void enterSwitchbotMenu() { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }
    void enterWakeOnLan() { changeMode(State::WAKE_ON_LAN); }
    void enterUltrasonic() {
        // 子機に測定開始を依頼し、表示リセット
        requestSendDistanceStart = true;
        mutex_enter_blocking(&data_mutex);
        State::sensors.ultrasonicDistance = -1.0;
        State::timers.lastDistanceUpdateTime = millis(); // 即座にタイムアウトしないよう初期化
        mutex_exit(&data_mutex);
        changeMode(State::ULTRASONIC_MONITOR);
    }
    void enterDiagnostics() { changeMode(State::SENSOR_DIAGNOSTICS); }
    void enterDeviceControl()
    {
        State::menu.deviceSelection = State::menu.menuSelection;
        State::menu.commandSelection = 0;
        changeMode(State::DEVICE_CONTROL);
    }

    void enterIrDeviceControl()
    {
        State::menu.deviceSelection = State::menu.menuSelection;
        State::menu.commandSelection = 0;
        changeMode(State::IR_DEVICE_CONTROL);
    }

    void enterIrSendMenu() { changeMode(State::IR_APPLIANCE_SELECT); }

    void toggleDisplayMode()
    {
        State::system.displayMode = (State::system.displayMode == 0) ? 1 : 0;
        const char *modeName = (State::system.displayMode == 0) ? "Default" : "New";
        Display::showTimedMessage("Display Mode", modeName, 2000, true);
    }

    void toggleDebugMode()
    {
        State::system.debugModeRuntime = !State::system.debugModeRuntime;
        Display::showTimedMessage("Debug Mode",
                                  State::system.debugModeRuntime ? "Enabled" : "Disabled",
                                  2000, true);
    }

    namespace
    {
        // 共通のアクション定義
        void actionToggleBacklight() { Utils::toggleBacklightMode(); Display::showTimedMessage("Backlight Mode", State::system.backlightAlwaysOn ? "Always ON" : "Auto OFF", 2000, true); }
        void actionToggleIllumination() { Utils::toggleIlluminationMode(); Display::showTimedMessage("Illumination", State::system.illuminationOn ? "ON" : "OFF", 2000, true); }
        void actionResyncTime() { requestResyncNtp = true; Display::showTimedMessage("Time Sync", "Requested.", 2000, true); }
        void actionShowIP() {
             char ipAddr[16];
             mutex_enter_blocking(&data_mutex);
             strcpy(ipAddr, (const char *)sharedIpAddress);
             mutex_exit(&data_mutex);
             Display::showTimedMessage("IP Address:", ipAddr, 2000, true);
        }
        void actionGetSwitchBotIDs() { requestGetSwitchbotIds = true; Display::showTimedMessage("SwitchBot", "Check Serial Mon.", 2000, true); }
        void actionSendLineTest() { requestSendLineTest = true; Display::showTimedMessage("LINE Test", "Sent.", 2000, true); }
        void actionManualLog() { requestManualLog = true; Display::showTimedMessage("Log Data", "Sent.", 2000, true); }
        void actionCalibrateSensor() { Sensors::calibrateSensor(); Display::showTimedMessage("Calibrate Sensor", "Executed.", 2000, true); }
        void actionIRReceive() { changeMode(State::IR_RECEIVE_MODE); Display::showTimedMessage("IR Receive", "Check Serial", 2000, false); }

        const MenuItem mainMenuDebug[] = {
            {"1. Lightning Log", enterHistory},
            {"2. Device Control", enterSwitchbotMenu},
            {"3. Wake on LAN", enterWakeOnLan},
            {"4. Backlight Mode", actionToggleBacklight},
            {"5. RGB Illumination", actionToggleIllumination},
            {"6. Measure Dist", enterUltrasonic},
            {"7. IR Send", enterIrSendMenu},
            {"8. Display Mode", toggleDisplayMode},
            {"9. Toggle Debug Mode", toggleDebugMode},
            {"10. Resync Time", actionResyncTime},
            {"11. Reboot", Utils::rebootDevice},
            {"12. Show IP Address", actionShowIP},
            {"13. Get SwitchBot IDs", actionGetSwitchBotIDs},
            {"14. Send LINE Test", actionSendLineTest},
            {"15. Log Data Manually", actionManualLog},
            {"16. Sensor Diag", enterDiagnostics},
            {"17. Calibrate Sensor", actionCalibrateSensor},
            {"18. IR Receive", actionIRReceive}
        };

        const MenuItem mainMenuStandard[] = {
            {"1. Lightning Log", enterHistory},
            {"2. Device Control", enterSwitchbotMenu},
            {"3. Wake on LAN", enterWakeOnLan},
            {"4. Backlight Mode", actionToggleBacklight},
            {"5. RGB Illumination", actionToggleIllumination},
            {"6. Measure Dist", enterUltrasonic},
            {"7. IR Send", enterIrSendMenu},
            {"8. Display Mode", toggleDisplayMode},
            {"9. Toggle Debug Mode", toggleDebugMode},
            {"10. Resync Time", actionResyncTime},
            {"11. Reboot", Utils::rebootDevice}
        };

        inline const MenuItem *getMainMenuItems()
        {
            // DebugModeが有効ならフルメニューを表示
            if (State::system.debugModeRuntime) {
                return mainMenuDebug;
            }
            return mainMenuStandard;
        }

        inline int getMainMenuCount()
        {
            if (State::system.debugModeRuntime) {
                return static_cast<int>(sizeof(mainMenuDebug) / sizeof(mainMenuDebug[0]));
            }
            return static_cast<int>(sizeof(mainMenuStandard) / sizeof(mainMenuStandard[0]));
        }
    }

    const MenuItem applianceMenu[] = {
        {"1. Light", enterDeviceControl}, {"2. TV", enterDeviceControl}, {"3. Air Conditioner", enterDeviceControl}, {"4. Fan", enterDeviceControl}, {"5. Speaker", enterDeviceControl}, {"6. Others", enterDeviceControl}, {"7. Back", []()
                                                                                                                                                                                                                             { changeMode(State::MENU); }}};
    const int APPLIANCE_MENU_COUNT = sizeof(applianceMenu) / sizeof(applianceMenu[0]);

    const MenuItem wolMenu[] = {
        {"1. Desktop PC", []()
         { requestSendWolDesktop = true; Display::showTimedMessage("WoL: Desktop PC", "Executed.", 2000, true); }},
        {"2. Server PC", []()
         { requestSendWolServer = true; Display::showTimedMessage("WoL: Server PC", "Executed.", 2000, true); }}};
    const int WOL_MENU_COUNT = sizeof(wolMenu) / sizeof(wolMenu[0]);

    const MenuItem lightControlMenu[] = {
        {"On", []()
         { requestSendWolDesktop = true; Display::showTimedMessage("WoL: Desktop PC", "Executed.", 2000, true); }},
        {"Off", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_LIGHT, "turnOff", "default"); Display::showTimedMessage("Light: Off", "Executed.", 2000, false); }},
        {"Bright+", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_LIGHT, "brightnessUp", "default"); Display::showTimedMessage("Light: Bright+", "Executed.", 2000, false); }},
        {"Bright-", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_LIGHT, "brightnessDown", "default"); Display::showTimedMessage("Light: Bright-", "Executed.", 2000, false); }},
        {"Warm", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_LIGHT, "setColorTemperature", "2700"); Display::showTimedMessage("Light: Warm", "Executed.", 2000, false); }},
        {"White", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_LIGHT, "setColorTemperature", "6500"); Display::showTimedMessage("Light: White", "Executed.", 2000, false); }},
        {"IR: On/Off", []() { IRRemote::sendIrSignal(&signal_LIGHT_POWER); Display::showTimedMessage("IR: Light", "On/Off Sent", 2000, false); }},
        {"IR: Bright+", []() { IRRemote::sendIrSignal(&signal_LIGHT_BRIGHT_UP); Display::showTimedMessage("IR: Light", "Bright+ Sent", 2000, false); }},
        {"IR: Bright-", []() { IRRemote::sendIrSignal(&signal_LIGHT_BRIGHT_DOWN); Display::showTimedMessage("IR: Light", "Bright- Sent", 2000, false); }},
        {"IR: Warm", []() { IRRemote::sendIrSignal(&signal_LIGHT_COLOR_WARM); Display::showTimedMessage("IR: Light", "Warm Sent", 2000, false); }},
        {"IR: White", []() { IRRemote::sendIrSignal(&signal_LIGHT_COLOR_COOL); Display::showTimedMessage("IR: Light", "White Sent", 2000, false); }},
        {"IR: Full", []() { IRRemote::sendIrSignal(&signal_LIGHT_FULL_LIGHT); Display::showTimedMessage("IR: Light", "Full Sent", 2000, false); }},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};
    const MenuItem tvControlMenu[] = {
        {"Power", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_TV, "turnOn", "default", "command"); Display::showTimedMessage("TV: Power", "Executed.", 2000, false); }},
        {"CH +", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_TV, "channelAdd", "default"); Display::showTimedMessage("TV: CH +", "Executed.", 2000, false); }},
        {"CH -", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_TV, "channelSub", "default"); Display::showTimedMessage("TV: CH -", "Executed.", 2000, false); }},
        {"Vol +", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_TV, "volumeAdd", "default"); Display::showTimedMessage("TV: Vol +", "Executed.", 2000, false); }},
        {"Vol -", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_TV, "volumeSub", "default"); Display::showTimedMessage("TV: Vol -", "Executed.", 2000, false); }},
        {"IR: Power", []() { IRRemote::sendIrSignal(&signal_TV_POWER); Display::showTimedMessage("IR: TV", "Power Sent", 2000, false); }},
        {"IR: Input", []() { IRRemote::sendIrSignal(&signal_TV_INPUT); Display::showTimedMessage("IR: TV", "Input Sent", 2000, false); }},
        {"IR: Vol+", []() { IRRemote::sendIrSignal(&signal_TV_VOL_UP); Display::showTimedMessage("IR: TV", "Vol+ Sent", 2000, false); }},
        {"IR: Vol-", []() { IRRemote::sendIrSignal(&signal_TV_VOL_DOWN); Display::showTimedMessage("IR: TV", "Vol- Sent", 2000, false); }},
        {"IR: CH+", []() { IRRemote::sendIrSignal(&signal_TV_CH_UP); Display::showTimedMessage("IR: TV", "CH+ Sent", 2000, false); }},
        {"IR: CH-", []() { IRRemote::sendIrSignal(&signal_TV_CH_DOWN); Display::showTimedMessage("IR: TV", "CH- Sent", 2000, false); }},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};
    const MenuItem acControlMenu[] = {
        {"Run", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_AC, "turnOn", "default"); Display::showTimedMessage("AC: Run", "Executed.", 2000, false); }},
        {"Stop", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_AC, "turnOff", "default"); Display::showTimedMessage("AC: Stop", "Executed.", 2000, false); }},
        {"Temp +", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_AC, "setTemperature", "26,auto,1,on"); Display::showTimedMessage("AC: Temp +", "Executed.", 2000, false); }},
        {"Temp -", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_AC, "setTemperature", "24,auto,1,on"); Display::showTimedMessage("AC: Temp -", "Executed.", 2000, false); }},
        {"Cooling", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_AC, "setAll", "25,2,1,on"); Display::showTimedMessage("AC: Cooling", "Executed.", 2000, false); }},
        {"Heating", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_AC, "setAll", "22,5,1,on"); Display::showTimedMessage("AC: Heating", "Executed.", 2000, false); }},
        {"Dehumidify", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_AC, "setAll", "25,3,1,on"); Display::showTimedMessage("AC: Dehumidify", "Executed.", 2000, false); }},
        {"IR: Cool 25", []() { IRRemote::sendIrSignal(&signal_AIRCON_COOL_ON); Display::showTimedMessage("IR: AC", "Cool 25 Sent", 2000, false); }},
        {"IR: Heat 25", []() { IRRemote::sendIrSignal(&signal_AIRCON_HEAT_ON); Display::showTimedMessage("IR: AC", "Heat 25 Sent", 2000, false); }},
        {"IR: Off", []() { IRRemote::sendIrSignal(&signal_AIRCON_OFF); Display::showTimedMessage("IR: AC", "Off Sent", 2000, false); }},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};
    const MenuItem fanControlMenu[] = {
        {"On", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_FAN, "press", "運転"); Display::showTimedMessage("Fan: On", "Executed.", 2000, false); }},
        {"Off", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_FAN, "press", "切/入"); Display::showTimedMessage("Fan: Off", "Executed.", 2000, false); }},
        {"Speed +", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_FAN, "press", "風量+"); Display::showTimedMessage("Fan: Speed +", "Executed.", 2000, false); }},
        {"Speed -", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_FAN, "press", "風量-"); Display::showTimedMessage("Fan: Speed -", "Executed.", 2000, false); }},
        {"Swing", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_FAN, "press", "スウィング"); Display::showTimedMessage("Fan: Swing", "Executed.", 2000, false); }},
        {"IR: Power", []() { IRRemote::sendIrSignal(&signal_FAN_POWER); Display::showTimedMessage("IR: Fan", "Power Sent", 2000, false); }},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};
    const MenuItem speakerControlMenu[] = {
        {"Power", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_SPEAKER, "press", "Power"); Display::showTimedMessage("Speaker: Power", "Executed.", 2000, false); }},
        {"Vol +", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_SPEAKER, "volumeAdd", "default"); Display::showTimedMessage("Speaker: Vol +", "Executed.", 2000, false); }},
        {"Vol -", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_SPEAKER, "volumeSub", "default"); Display::showTimedMessage("Speaker: Vol -", "Executed.", 2000, false); }},
        {"Prev", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_SPEAKER, "previousTrack", "default"); Display::showTimedMessage("Speaker: Prev", "Executed.", 2000, false); }},
        {"Next", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_SPEAKER, "nextTrack", "default"); Display::showTimedMessage("Speaker: Next", "Executed.", 2000, false); }},
        {"IR: Power", []() { IRRemote::sendIrSignal(&signal_SPEAKER_POWER); Display::showTimedMessage("IR: Speaker", "Power Sent", 2000, false); }},
        {"IR: Vol+", []() { IRRemote::sendIrSignal(&signal_SPEAKER_VOL_UP); Display::showTimedMessage("IR: Speaker", "Vol+ Sent", 2000, false); }},
        {"IR: Vol-", []() { IRRemote::sendIrSignal(&signal_SPEAKER_VOL_DOWN); Display::showTimedMessage("IR: Speaker", "Vol- Sent", 2000, false); }},
        {"IR: Mute", []() { IRRemote::sendIrSignal(&signal_SPEAKER_MUTE); Display::showTimedMessage("IR: Speaker", "Mute Sent", 2000, false); }},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};
    const MenuItem othersControlMenu[] = {
        {"Command 1", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_OTHERS, "turnOn", "default"); Display::showTimedMessage("Others: Cmd 1", "Executed.", 2000, false); }},
        {"Command 2", []()
         { Network::sendSwitchBotCommand(DEVICE_ID_OTHERS, "turnOff", "default"); Display::showTimedMessage("Others: Cmd 2", "Executed.", 2000, false); }},
        {"Back", []()
         { changeMode(State::SWITCHBOT_APPLIANCE_SELECT); }}};

    // --- IR Menus ---
    const MenuItem irLightMenu[] = {
        {"On/Off", []() { IRRemote::sendIrSignal(&signal_LIGHT_POWER); Display::showTimedMessage("Light", "On/Off Sent", 2000, false); }},
        {"Bright+", []() { IRRemote::sendIrSignal(&signal_LIGHT_BRIGHT_UP); Display::showTimedMessage("Light", "Bright+ Sent", 2000, false); }},
        {"Bright-", []() { IRRemote::sendIrSignal(&signal_LIGHT_BRIGHT_DOWN); Display::showTimedMessage("Light", "Bright- Sent", 2000, false); }},
        {"Warm", []() { IRRemote::sendIrSignal(&signal_LIGHT_COLOR_WARM); Display::showTimedMessage("Light", "Warm Sent", 2000, false); }},
        {"White", []() { IRRemote::sendIrSignal(&signal_LIGHT_COLOR_COOL); Display::showTimedMessage("Light", "White Sent", 2000, false); }},
        {"Full/Away", []() { IRRemote::sendIrSignal(&signal_LIGHT_FULL_LIGHT); Display::showTimedMessage("Light", "Full Sent", 2000, false); }},
        {"Back", []() { changeMode(State::IR_APPLIANCE_SELECT); }}};

    const MenuItem irTvMenu[] = {
        {"Power", []() { IRRemote::sendIrSignal(&signal_TV_POWER); Display::showTimedMessage("TV", "Power Sent", 2000, false); }},
        {"Input", []() { IRRemote::sendIrSignal(&signal_TV_INPUT); Display::showTimedMessage("TV", "Input Sent", 2000, false); }},
        {"Vol+", []() { IRRemote::sendIrSignal(&signal_TV_VOL_UP); Display::showTimedMessage("TV", "Vol+ Sent", 2000, false); }},
        {"Vol-", []() { IRRemote::sendIrSignal(&signal_TV_VOL_DOWN); Display::showTimedMessage("TV", "Vol- Sent", 2000, false); }},
        {"CH+", []() { IRRemote::sendIrSignal(&signal_TV_CH_UP); Display::showTimedMessage("TV", "CH+ Sent", 2000, false); }},
        {"CH-", []() { IRRemote::sendIrSignal(&signal_TV_CH_DOWN); Display::showTimedMessage("TV", "CH- Sent", 2000, false); }},
        {"YouTube", []() { IRRemote::sendIrSignal(&signal_TV_YOUTUBE); Display::showTimedMessage("TV", "YouTube Sent", 2000, false); }},
        {"Back", []() { changeMode(State::IR_APPLIANCE_SELECT); }}};

    const MenuItem irAcMenu[] = {
        {"Power", []() { IRRemote::sendIrSignal(&signal_AIRCON_POWER); Display::showTimedMessage("AC", "Power Sent", 2000, false); }},
        {"Temp+", []() { IRRemote::sendIrSignal(&signal_AIRCON_TEMP_UP); Display::showTimedMessage("AC", "Temp+ Sent", 2000, false); }},
        {"Temp-", []() { IRRemote::sendIrSignal(&signal_AIRCON_TEMP_DOWN); Display::showTimedMessage("AC", "Temp- Sent", 2000, false); }},
        {"Mode", []() { IRRemote::sendIrSignal(&signal_AIRCON_MODE); Display::showTimedMessage("AC", "Mode Sent", 2000, false); }},
        {"Fan", []() { IRRemote::sendIrSignal(&signal_AIRCON_FAN); Display::showTimedMessage("AC", "Fan Sent", 2000, false); }},
        {"Eco", []() { IRRemote::sendIrSignal(&signal_AIRCON_ECO); Display::showTimedMessage("AC", "Eco Sent", 2000, false); }},
        {"Back", []() { changeMode(State::IR_APPLIANCE_SELECT); }}};

    const MenuItem irFanMenu[] = {
        {"Power", []() { IRRemote::sendIrSignal(&signal_FAN_POWER); Display::showTimedMessage("Fan", "Power Sent", 2000, false); }},
        {"Speed+", []() { IRRemote::sendIrSignal(&signal_FAN_SPEED_UP); Display::showTimedMessage("Fan", "Speed+ Sent", 2000, false); }},
        {"Speed-", []() { IRRemote::sendIrSignal(&signal_FAN_SPEED_DOWN); Display::showTimedMessage("Fan", "Speed- Sent", 2000, false); }},
        {"Swing", []() { IRRemote::sendIrSignal(&signal_FAN_SWING); Display::showTimedMessage("Fan", "Swing Sent", 2000, false); }},
        {"Back", []() { changeMode(State::IR_APPLIANCE_SELECT); }}};

    const MenuItem irSpeakerMenu[] = {
        {"Power", []() { IRRemote::sendIrSignal(&signal_SPEAKER_POWER); Display::showTimedMessage("Speaker", "Power Sent", 2000, false); }},
        {"Vol+", []() { IRRemote::sendIrSignal(&signal_SPEAKER_VOL_UP); Display::showTimedMessage("Speaker", "Vol+ Sent", 2000, false); }},
        {"Vol-", []() { IRRemote::sendIrSignal(&signal_SPEAKER_VOL_DOWN); Display::showTimedMessage("Speaker", "Vol- Sent", 2000, false); }},
        {"Play/Pause", []() { IRRemote::sendIrSignal(&signal_SPEAKER_PLAY_PAUSE); Display::showTimedMessage("Speaker", "Play/Pause Sent", 2000, false); }},
        {"Skip+", []() { IRRemote::sendIrSignal(&signal_SPEAKER_NEXT); Display::showTimedMessage("Speaker", "Skip+ Sent", 2000, false); }},
        {"Skip-", []() { IRRemote::sendIrSignal(&signal_SPEAKER_PREV); Display::showTimedMessage("Speaker", "Skip- Sent", 2000, false); }},
        {"Folder+", []() { IRRemote::sendIrSignal(&signal_SPEAKER_FOLDER_NEXT); Display::showTimedMessage("Speaker", "Folder+ Sent", 2000, false); }},
        {"Folder-", []() { IRRemote::sendIrSignal(&signal_SPEAKER_FOLDER_PREV); Display::showTimedMessage("Speaker", "Folder- Sent", 2000, false); }},
        {"Fwd", []() { IRRemote::sendIrSignal(&signal_SPEAKER_FFWD); Display::showTimedMessage("Speaker", "Fwd Sent", 2000, false); }},
        {"Rew", []() { IRRemote::sendIrSignal(&signal_SPEAKER_REWIND); Display::showTimedMessage("Speaker", "Rew Sent", 2000, false); }},
        {"SD/USB/CD", []() { IRRemote::sendIrSignal(&signal_SPEAKER_SOURCE_SD); Display::showTimedMessage("Speaker", "SD/USB/CD Sent", 2000, false); }},
        {"FM/AM", []() { IRRemote::sendIrSignal(&signal_SPEAKER_SOURCE_FM); Display::showTimedMessage("Speaker", "FM/AM Sent", 2000, false); }},
        {"LINE/MIC", []() { IRRemote::sendIrSignal(&signal_SPEAKER_SOURCE_LINE); Display::showTimedMessage("Speaker", "LINE/MIC Sent", 2000, false); }},
        {"Back", []() { changeMode(State::IR_APPLIANCE_SELECT); }}};

    const MenuItem irApplianceMenu[] = {
        {"1. Light", enterIrDeviceControl},
        {"2. TV", enterIrDeviceControl},
        {"3. Air Conditioner", enterIrDeviceControl},
        {"4. Fan", enterIrDeviceControl},
        {"5. Speaker", enterIrDeviceControl},
        {"6. IR Receive", []() { changeMode(State::IR_RECEIVE_MODE); Display::showTimedMessage("IR Receive", "Check Serial", 2000, false); }},
        {"7. Back", []() { changeMode(State::MENU); }}};
    
    const int IR_APPLIANCE_MENU_COUNT = sizeof(irApplianceMenu) / sizeof(irApplianceMenu[0]);
    const MenuItem *const irDeviceControlMenus[] = {irLightMenu, irTvMenu, irAcMenu, irFanMenu, irSpeakerMenu};
    const int irDeviceControlMenuCounts[] = {7, 8, 7, 5, 14};

    const MenuItem *const applianceControlMenus[] = {lightControlMenu, tvControlMenu, acControlMenu, fanControlMenu, speakerControlMenu, othersControlMenu};
    const int applianceControlMenuCounts[] = {13, 12, 11, 7, 10, 3};

    const MenuItem *getCurrentMenu(int &count, int &selection)
    {
        switch (State::menu.currentMode)
        {
        case State::MENU:
            count = getMainMenuCount();
            selection = State::menu.menuSelection;
            return getMainMenuItems();
        case State::SWITCHBOT_APPLIANCE_SELECT:
            count = APPLIANCE_MENU_COUNT;
            selection = State::menu.menuSelection;
            return applianceMenu;
        case State::WAKE_ON_LAN:
            count = WOL_MENU_COUNT;
            selection = State::menu.menuSelection;
            return wolMenu;
        case State::DEVICE_CONTROL:
            if (State::menu.deviceSelection < sizeof(applianceControlMenus) / sizeof(MenuItem **))
            {
                count = applianceControlMenuCounts[State::menu.deviceSelection];
                selection = State::menu.commandSelection;
                return applianceControlMenus[State::menu.deviceSelection];
            }
        case State::IR_APPLIANCE_SELECT:
            count = IR_APPLIANCE_MENU_COUNT;
            selection = State::menu.menuSelection;
            return irApplianceMenu;
        case State::IR_DEVICE_CONTROL:
            if (State::menu.deviceSelection < sizeof(irDeviceControlMenus) / sizeof(MenuItem **))
            {
                count = irDeviceControlMenuCounts[State::menu.deviceSelection];
                selection = State::menu.commandSelection;
                return irDeviceControlMenus[State::menu.deviceSelection];
            }
        default:
            count = 0;
            selection = 0;
            return nullptr;
        }
    }
}

//================================================================
// ユーティリティ関数
//================================================================
namespace Utils
{
    void setRGB(int r, int g, int b)
    {
        analogWrite(Pins::LED_R, r);
        analogWrite(Pins::LED_G, g);
        analogWrite(Pins::LED_B, b);
    }

    // ノンブロッキングLED点滅リクエスト
    void requestBlinkLED(String color, int times, int duration)
    {
        mutex_enter_blocking(&data_mutex);
        State::timers.ledBlinkActive = true;
        State::timers.ledBlinkColor = color;
        State::timers.ledBlinkRemainingToggles = times * 2; // 点灯と消灯で2倍
        State::timers.ledBlinkInterval = duration;
        State::timers.ledBlinkNextToggle = millis(); // 即時開始
        State::timers.ledCurrentState = false;       // 最初はOFF->ONにするため
        mutex_exit(&data_mutex);
    }

    // メインループから呼び出す
    void handleLEDBlink()
    {
        // v19.3.0: mutex取得前にフラグを事前チェック（mutex競合を軽減）
        if (!State::timers.ledBlinkActive)
        {
            return;
        }
        // v19.5.0: try_enterでデッドロック回避（取れなければ次回に延期）
        uint32_t owner;
        if (!mutex_try_enter(&data_mutex, &owner))
        {
            return;
        }
        if (!State::timers.ledBlinkActive)
        {
            mutex_exit(&data_mutex);
            return;
        }

        if (millis() >= State::timers.ledBlinkNextToggle)
        {
            if (State::timers.ledBlinkRemainingToggles > 0)
            {
                State::timers.ledCurrentState = !State::timers.ledCurrentState;
                if (State::timers.ledCurrentState)
                {
                    // 点灯
                    if (State::timers.ledBlinkColor == "green")
                        setRGB(0, 255, 0);
                    else if (State::timers.ledBlinkColor == "yellow")
                        setRGB(255, 255, 0);
                    else if (State::timers.ledBlinkColor == "blue")
                        setRGB(0, 0, 255);
                    else if (State::timers.ledBlinkColor == "red")
                        setRGB(255, 0, 0);
                    else if (State::timers.ledBlinkColor == "white")
                        setRGB(255, 255, 255);
                }
                else
                {
                    // 消灯
                    setRGB(0, 0, 0);
                }
                State::timers.ledBlinkRemainingToggles--;
                State::timers.ledBlinkNextToggle = millis() + State::timers.ledBlinkInterval;
            }
            else
            {
                if (State::timers.ledBlinkInfinite)
                {
                    // 無限点滅: カウンタをリセットして繰り返す
                    State::timers.ledBlinkRemainingToggles = 2;
                }
                else
                {
                    // 終了
                    setRGB(0, 0, 0);
                    State::timers.ledBlinkActive = false;
                }
            }
        }
        mutex_exit(&data_mutex);
    }

    void requestBlinkLEDInfinite(String color, int intervalMs)
    {
        mutex_enter_blocking(&data_mutex);
        State::timers.ledBlinkActive = true;
        State::timers.ledBlinkInfinite = true;
        State::timers.ledBlinkColor = color;
        State::timers.ledBlinkRemainingToggles = 2;
        State::timers.ledBlinkInterval = intervalMs;
        State::timers.ledBlinkNextToggle = millis();
        State::timers.ledCurrentState = false;
        mutex_exit(&data_mutex);
    }

    void stopLEDBlink()
    {
        mutex_enter_blocking(&data_mutex);
        State::timers.ledBlinkActive = false;
        State::timers.ledBlinkInfinite = false;
        State::timers.ledBlinkRemainingToggles = 0;
        setRGB(0, 0, 0);
        mutex_exit(&data_mutex);
    }

    void handleSmoothIllumination()
    {
        unsigned long hue = millis() / 10;
        hue %= 360;
        float s = 1.0, v = 1.0, r, g, b;
        int i = floor(hue / 60.0);
        float f = hue / 60.0 - i, p = v * (1.0 - s), q = v * (1.0 - (s * f)), t = v * (1.0 - (s * (1.0 - f)));
        switch (i % 6)
        {
        case 0:
            r = v, g = t, b = p;
            break;
        case 1:
            r = q, g = v, b = p;
            break;
        case 2:
            r = p, g = v, b = t;
            break;
        case 3:
            r = p, g = q, b = v;
            break;
        case 4:
            r = t, g = p, b = v;
            break;
        case 5:
            r = v, g = p, b = q;
            break;
        }
        setRGB(r * 255, g * 255, b * 255);
    }

    // v19.0.0: バックライトのPWM制御によるフェード
    void setBacklightBrightness(int brightness)
    {
        brightness = constrain(brightness, 0, 255);
        analogWrite(Pins::LCD_BACKLIGHT, brightness);
        State::system.backlightCurrentBrightness = brightness;
    }

    void startBacklightFade(bool turnOn)
    {
        State::system.backlightFading = true;
        State::system.backlightTargetOn = turnOn;
        State::system.backlightFadeStartTime = millis();
        // 現在の輝度から開始（フェードイン/アウト両方に対応）
        if (turnOn)
        {
            setBacklightBrightness(0); // フェードイン開始時は0から
        }
        else
        {
            setBacklightBrightness(255); // フェードアウト開始時は255から
        }
    }

    void updateBacklightFade()
    {
        if (!State::system.backlightFading)
            return;

        unsigned long elapsed = millis() - State::system.backlightFadeStartTime;

        if (elapsed >= BACKLIGHT_FADE_MS)
        {
            // フェード完了
            setBacklightBrightness(State::system.backlightTargetOn ? 255 : 0);
            State::system.backlightFading = false;
        }
        else
        {
            // フェード中
            float progress = (float)elapsed / BACKLIGHT_FADE_MS;
            int brightness;

            if (State::system.backlightTargetOn)
            {
                // フェードイン: 0 → 255
                brightness = (int)(progress * 255);
            }
            else
            {
                // フェードアウト: 255 → 0
                brightness = (int)((1.0 - progress) * 255);
            }

            setBacklightBrightness(brightness);
        }
    }

    void toggleBacklightMode()
    {
        State::system.backlightAlwaysOn = !State::system.backlightAlwaysOn;
        startBacklightFade(State::system.backlightAlwaysOn);
    }

    void toggleIlluminationMode()
    {
        State::system.illuminationOn = !State::system.illuminationOn;
        if (!State::system.illuminationOn)
        {
            setRGB(0, 0, 0);
        }
    }

    void rebootDevice()
    {
        Display::showTimedMessage("Rebooting...", "", 1000, true);
        delay(1000); // Wait for message to show
        rp2040.reboot();
    }
}

//================================================================
// ディスプレイ管理
//================================================================
namespace Display
{
    void printLcdLine(int line, const char *text)
    {
        char buf[LCD_COLS + 1];
        snprintf(buf, sizeof(buf), "%-*s", LCD_COLS, text);
        State::lcd.setCursor(0, line);
        State::lcd.print(buf);
    }

    void showTimedMessage(const char *line0, const char *line1, unsigned long duration, bool returnToMain)
    {
        mutex_enter_blocking(&data_mutex);
        strncpy(State::timers.timedMessageLine0, line0, LCD_COLS);
        State::timers.timedMessageLine0[LCD_COLS] = '\0';
        strncpy(State::timers.timedMessageLine1, line1, LCD_COLS);
        State::timers.timedMessageLine1[LCD_COLS] = '\0';
        State::timers.timedMessageEndTime = millis() + duration;
        State::timers.returnToMainAfterMessage = returnToMain;
        State::timers.messageNeedsRedraw = true; // 新しいメッセージを表示するために再描画を要求
        mutex_exit(&data_mutex);
    }

    // 画面の各パーツを描画するヘルパー関数
    namespace
    { // 無名名前空間でこのファイル内からのみアクセス可能にする
        void drawTime(time_t t)
        {
            char buf[LCD_COLS + 1];
            if (State::system.ntpInitialized && t > 100000)
            {
                strftime(buf, sizeof(buf), "%m/%d(%a) %H:%M:%S", localtime(&t));
            }
            else
            {
                // ★ アニメーション表示
                static int animationState = 0;
                static unsigned long lastAnimationTime = 0;
                const char *baseText = "Connecting Network";

                if (millis() - lastAnimationTime > 300)
                { // 300msごとに表示を更新
                    lastAnimationTime = millis();
                    animationState = (animationState + 1) % 4;
                }

                switch (animationState)
                {
                case 0:
                    snprintf(buf, sizeof(buf), "%s  ", baseText);
                    break; // __
                case 1:
                    snprintf(buf, sizeof(buf), "%s. ", baseText);
                    break; // ._
                case 2:
                    snprintf(buf, sizeof(buf), "%s..", baseText);
                    break; // ..
                case 3:
                    snprintf(buf, sizeof(buf), "%s .", baseText);
                    break; // _.
                }
            }
            printLcdLine(0, buf);
        }

        void drawSensors()
        {
            // ★ オーバーレイメッセージがアクティブな場合はそれを表示
            if (millis() < State::timers.timedOverlayEndTime)
            {
                printLcdLine(1, State::timers.timedOverlayLine1);
                return;
            }

            // 通常のセンサー表示（室内）
            char buf[LCD_COLS + 1];
            if (!State::system.dht20_initialized)
            {
                snprintf(buf, sizeof(buf), "Inside: Init Failed!");
            }
            else if (State::sensors.temperature > -999.0)
            {
                snprintf(buf, sizeof(buf), "T:%4.1fC  H:%4.1f%%", State::sensors.temperature, State::sensors.humidity);
            }
            else
            {
                snprintf(buf, sizeof(buf), "T: Reading...");
            }
            printLcdLine(1, buf);
        }

        void drawOutdoorSensors()
        {
            char buf[LCD_COLS + 1];
            if (!State::system.dht22_outdoor_initialized)
            {
                snprintf(buf, sizeof(buf), "Outdoor: Not Connect");
            }
            else if (State::sensors.outdoorTemperature > -999.0)
            {
                snprintf(buf, sizeof(buf), "O:%4.1fC  H:%4.1f%%", State::sensors.outdoorTemperature, State::sensors.outdoorHumidity);
            }
            else
            {
                snprintf(buf, sizeof(buf), "Outdoor: Reading...");
            }
            printLcdLine(2, buf);
        }

        void drawLightningInfo()
        {
            char buf[LCD_COLS + 1];
            if (State::sensors.lastEventType == "Lightning")
            {
                snprintf(buf, sizeof(buf), "Last Evt:%s", State::sensors.lastEventTime);
                printLcdLine(2, buf);
                snprintf(buf, sizeof(buf), "Distance: %d km", State::sensors.lastLightningDistance);
                printLcdLine(3, buf);
            }
            else if (State::sensors.lastEventType == "Noise")
            {
                snprintf(buf, sizeof(buf), "Last Evt:%s", State::sensors.lastEventTime);
                printLcdLine(2, buf);
                printLcdLine(3, "Distance: Noise");
            }
            else
            {
                printLcdLine(2, "Last Event: None");
                if (!State::system.as3935_initialized)
                {
                    printLcdLine(3, "AS3935 Init Failed");
                }
                else
                {
                    printLcdLine(3, "Distance: ---");
                }
            }
        }

        void drawDistance()
        {
            char buf[LCD_COLS + 1];
            if (State::sensors.lastEventType == "Lightning")
            {
                snprintf(buf, sizeof(buf), "Thunder Dist: %d km", State::sensors.lastLightningDistance);
            }
            else if (State::sensors.lastEventType == "Noise")
            {
                snprintf(buf, sizeof(buf), "Thunder Dist: Noise");
            }
            else
            {
                snprintf(buf, sizeof(buf), "Thunder Dist: ---");
            }
            printLcdLine(3, buf);
        }
    }

    // メイン画面の表示を更新する
    void updateMainDisplay()
    {
        static time_t lastDisplayedTime = 0;
        static unsigned long lastSensorDraw = 0;
        static unsigned long lastTimeDraw = 0;
        int effectiveDisplayMode;

        bool hasLightningEvent = (State::sensors.lastEventType == "Lightning" || State::sensors.lastEventType == "Noise");
        
        if (hasLightningEvent)
        {
            effectiveDisplayMode = 1;
        }
        else
        {
            effectiveDisplayMode = State::system.displayMode;
        }

        if (State::system.forceMainScreenRedraw)
        {
            State::lcd.clear();
            lastDisplayedTime = 0;
            lastTimeDraw = 0;
            drawSensors();
            
            if (effectiveDisplayMode == 1)
            {
                drawLightningInfo();
            }
            else
            {
                drawOutdoorSensors();
                drawDistance();
            }
            lastSensorDraw = millis();
            State::system.forceMainScreenRedraw = false;
        }

        time_t now = time(nullptr);

        // v19.0.0: 時刻表示は秒が変わった時のみ更新（高速化）
        if (now != lastDisplayedTime)
        {
            drawTime(now);
            lastDisplayedTime = now;
            lastTimeDraw = millis();
        }

        // センサー表示は500msごとに更新
        if (millis() - lastSensorDraw > 500)
        {
            drawSensors();
            
            if (effectiveDisplayMode == 1)
            {
                drawLightningInfo();
            }
            else
            {
                drawOutdoorSensors();
                drawDistance();
            }
            
            lastSensorDraw = millis();
        }
    }

    // メニュー画面を描画する
    void drawMenu()
    {
        int itemCount = 0, currentSelection = 0;
        const Menu::MenuItem *menuItems = Menu::getCurrentMenu(itemCount, currentSelection);
        if (!menuItems)
            return;

        int page = currentSelection / LCD_ROWS;
        for (int i = 0; i < LCD_ROWS; i++)
        {
            int index = page * LCD_ROWS + i;
            char buf[LCD_COLS + 1];
            if (index < itemCount)
            {
                sprintf(buf, "%s%s", (index == currentSelection ? ">" : " "), menuItems[index].text);
            }
            else
            {
                strcpy(buf, "");
            }
            printLcdLine(i, buf);
        }
    }

    // 雷履歴画面を描画する
    void drawHistoryScreen()
    {
        printLcdLine(0, "--- Lightning Log ---");
        for (int i = 0; i < HISTORY_SIZE; i++)
        {
            char buf[LCD_COLS + 1];
            if (i < State::history.count)
            {
                int idx = (State::history.index - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
                if (State::history.records[idx].type == "Noise")
                {
                    snprintf(buf, sizeof(buf), "%d: %s Noise", i + 1, State::history.records[idx].timestamp);
                }
                else
                {
                    snprintf(buf, sizeof(buf), "%d: %s %2dkm", i + 1, State::history.records[idx].timestamp, State::history.records[idx].distance);
                }
            }
            else
            {
                snprintf(buf, sizeof(buf), "%d: ---", i + 1);
            }
            printLcdLine(i + 1, buf);
        }
    }

    // 超音波センサー監視画面を描画する
    void drawUltrasonicMonitorScreen()
    {
        // 子機からのプッシュが 2 秒以上絶えたら自動退出
        unsigned long lastUpdate;
        mutex_enter_blocking(&data_mutex);
        lastUpdate = State::timers.lastDistanceUpdateTime;
        mutex_exit(&data_mutex);
        if (lastUpdate > 0 && millis() - lastUpdate > 2000) {
            // 子機が止まった → メイン画面へ
            requestSendDistanceStop = true; // 孟幸に停止が届いていなければ送信
            mutex_enter_blocking(&data_mutex);
            Menu::changeMode(State::MAIN_DISPLAY);
            State::timers.lastDistanceUpdateTime = 0;
            mutex_exit(&data_mutex);
            return;
        }

        printLcdLine(0, "Ultrasonic Sensor");
        char buf[LCD_COLS + 1];
        mutex_enter_blocking(&data_mutex);
        float dist = State::sensors.ultrasonicDistance;
        mutex_exit(&data_mutex);
        if (State::childIpAddress == "")
            snprintf(buf, sizeof(buf), "Child not found");
        else if (dist == -2.0)
            snprintf(buf, sizeof(buf), "Child offline");
        else if (dist < 0)
            snprintf(buf, sizeof(buf), "Waiting...");
        else
            snprintf(buf, sizeof(buf), "Dist: %.1f cm", dist);
        printLcdLine(1, buf);
        printLcdLine(2, "");
        printLcdLine(3, "(Long press to exit)");
    }

    // センサー診断画面を描画する
    void drawDiagnosticsScreen()
    {
        static unsigned long lastReadTime = 0;
        if (millis() - lastReadTime > 1000)
        { // 1秒ごとに更新
            lastReadTime = millis();

            printLcdLine(0, "--- Sensor Diag ---");
            char buf[LCD_COLS + 1];

            uint8_t noise = State::lightning.readNoiseLevel();
            uint8_t watchdog = State::lightning.readWatchdogThreshold();
            snprintf(buf, sizeof(buf), "Noise:%d Watchdog:%d", noise, watchdog);
            printLcdLine(1, buf);

            uint8_t spike = State::lightning.readSpikeRejection();
            uint8_t intReg = State::lightning.readInterruptReg();
            snprintf(buf, sizeof(buf), "Spike:%d IntReg:0x%02X", spike, intReg);
            printLcdLine(2, buf);

            int irqPinState = digitalRead(Pins::LIGHTNING_IRQ);
            snprintf(buf, sizeof(buf), "IRQ Pin State: %d", irqPinState);
            printLcdLine(3, buf);
        }
    }

    // 現在のモードに応じて画面を更新する
    void update()
    {
        // v19.5.0: mutex_enter_blockingの代わりにtry_enterスピンでデッドロック回避
        // Core1がmutexを長時間保持中でも、Core0がWatchdogタイムアウトしない
        uint32_t owner;
        unsigned long spinStart = millis();
        while (!mutex_try_enter(&data_mutex, &owner)) {
            petWatchdog();
            if (millis() - spinStart > 2000) return; // 2秒待っても取れなければこのフレームはスキップ
            delayMicroseconds(100);
        }

        // ★ フルスクリーンメッセージの期限切れを処理
        if (State::timers.timedMessageEndTime != 0 && millis() >= State::timers.timedMessageEndTime)
        {
            State::timers.timedMessageEndTime = 0;
            if (State::timers.returnToMainAfterMessage)
            {
                Menu::changeMode(State::MAIN_DISPLAY);
            }
            else
            {
                State::system.needsRedraw = true;
            }
        }

        // ★ お風呂通知アクティブ中は画面を完全に上書き
        if (State::system.bathAlertActive)
        {
            if (State::system.needsRedraw || State::system.forceMainScreenRedraw)
            {
                State::lcd.clear();
                printLcdLine(0, "** Bath Ready! **");
                // 現在時刻を取得して表示
                char timeBuf[LCD_COLS + 1] = "Time: --:--         ";
                time_t now = time(nullptr);
                if (now > 100000) {
                    struct tm* t = localtime(&now);
                    char tmp[6];
                    strftime(tmp, sizeof(tmp), "%H:%M", t);
                    snprintf(timeBuf, sizeof(timeBuf), "Time: %s", tmp);
                }
                printLcdLine(1, timeBuf);
                printLcdLine(2, "");
                printLcdLine(3, "[Press to clear]");
                State::system.needsRedraw = false;
                State::system.forceMainScreenRedraw = false;
            }
            mutex_exit(&data_mutex);
            return;
        }

        // ★ フルスクリーンメッセージが表示中の場合、他のすべてを上書き
        // ★ フルスクリーンメッセージが表示中の場合、他のすべてを上書き
        if (State::timers.timedMessageEndTime != 0)
        {
            // メッセージがアクティブになった瞬間、または内容更新時のみ描画
            if (State::timers.messageNeedsRedraw)
            {
                State::lcd.clear();
                printLcdLine(0, State::timers.timedMessageLine0);
                printLcdLine(1, State::timers.timedMessageLine1);
                State::timers.messageNeedsRedraw = false;
            }
            mutex_exit(&data_mutex);
            return;
        }

        // ★ オーバーレイメッセージの期限切れを処理
        if (State::timers.timedOverlayEndTime != 0 && millis() >= State::timers.timedOverlayEndTime)
        {
            State::timers.timedOverlayEndTime = 0;
            State::system.forceMainScreenRedraw = true; // メイン画面を再描画してセンサー情報を復元
        }

        if (State::system.needsRedraw)
        {
            // lcd.clear() はちらつきの原因になるため、強制再描画フラグがある場合のみ使用する
             if (State::system.forceMainScreenRedraw)
             {
                 State::lcd.clear();
             }
        }

        switch (State::menu.currentMode)
        {
        case State::MAIN_DISPLAY:
            updateMainDisplay();
            break;
        case State::ULTRASONIC_MONITOR:
            drawUltrasonicMonitorScreen();
            break;
        case State::HISTORY:
            drawHistoryScreen();
            break;
        case State::SENSOR_DIAGNOSTICS:
            drawDiagnosticsScreen();
            break;
        default: // MENU, DEVICE_CONTROLなど
            if (State::system.needsRedraw)
                drawMenu();
            break;
        }

        State::system.needsRedraw = false;
        State::system.forceMainScreenRedraw = false;
        mutex_exit(&data_mutex);
    }

    // ディスプレイの初期化
    void init()
    {
        State::lcd.begin(LCD_COLS, LCD_ROWS);
        Display::printLcdLine(0, "System Starting...");
    }
}

//================================================================
// ネットワーク管理 (Core1)
//================================================================
namespace Network
{

    namespace
    {
        // v19.0.0: WiFi接続の安定性向上 - 1つのSSIDに対して接続を試みる
        bool connectToSingleNetwork(const char *ssid, const char *password)
        {
            // v19.1.7: 特定のSSID（iPhone等）の場合は強制的にDHCPを使用する
            bool effectiveUseStaticIP = USE_STATIC_IP;
            if (effectiveUseStaticIP)
            {
                // SSIDに "iPhone" が含まれる場合はDHCPにフォールバック
                if (strstr(ssid, "iPhone") != nullptr)
                {
                    if (DEBUG) Serial.println("[WiFi] iPhone接続を検出: 強制的にDHCPを使用します");
                    effectiveUseStaticIP = false;
                }
            }

            if (DEBUG)
            {
                Serial.printf("[WiFi] 接続開始: %s (%s)\n", ssid, effectiveUseStaticIP ? "固定IP" : "DHCP");
                Serial.flush();
            }

            // 完全にクリーンな状態から開始
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            delay(500); // v19.0.0: 確実に切断するために待機
            WiFi.mode(WIFI_STA);
            // v19.0.0: Pico WのWiFiライブラリは自動再接続がデフォルトで有効
            delay(100);

            // 静的IP設定
            if (effectiveUseStaticIP)
            {
                IPAddress local_IP(STATIC_IP_BYTES[0], STATIC_IP_BYTES[1], STATIC_IP_BYTES[2], STATIC_IP_BYTES[3]);
                IPAddress gateway(GATEWAY_BYTES[0], GATEWAY_BYTES[1], GATEWAY_BYTES[2], GATEWAY_BYTES[3]);
                IPAddress subnet(SUBNET_BYTES[0], SUBNET_BYTES[1], SUBNET_BYTES[2], SUBNET_BYTES[3]);
                IPAddress dns(PRIMARY_DNS_BYTES[0], PRIMARY_DNS_BYTES[1], PRIMARY_DNS_BYTES[2], PRIMARY_DNS_BYTES[3]);

                if (DEBUG)
                {
                    Serial.printf("[WiFi] 固定IP設定: %s\n", local_IP.toString().c_str());
                    Serial.flush();
                }

                // v19.0.0: Pico Wのconfigはvoidを返すため、戻り値チェックなし
                WiFi.config(local_IP, dns, gateway, subnet);
            }
            else
            {
                if (DEBUG)
                {
                    Serial.println("[WiFi] DHCP使用");
                    Serial.flush();
                }
            }

            // 接続開始
            WiFi.begin(ssid, password);

            // 接続待機（最大30秒、v19.0.0: より細かいチェック）
            unsigned long startTime = millis();
            unsigned long timeout = 30000;
            int dotCount = 0;
            uint8_t lastStatus = WL_IDLE_STATUS;

            while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < timeout)
            {
                delay(500);
                uint8_t currentStatus = WiFi.status();

                if (DEBUG)
                {
                    // ステータスが変わった時のみ詳細表示
                    if (currentStatus != lastStatus)
                    {
                        Serial.printf("\n[WiFi] Status: %d ", currentStatus);
                        lastStatus = currentStatus;
                        dotCount = 0;
                    }
                    Serial.print(".");
                    if (++dotCount >= 60)
                    {
                        Serial.println();
                        dotCount = 0;
                    }
                    Serial.flush();
                }

                // v19.0.0: より確実な失敗検出
                if (currentStatus == WL_CONNECT_FAILED ||
                    currentStatus == WL_NO_SSID_AVAIL ||
                    currentStatus == WL_CONNECTION_LOST)
                {
                    if (DEBUG)
                    {
                        Serial.printf("\n[WiFi] 接続失敗 (ステータス: %d)\n", currentStatus);
                        Serial.flush();
                    }
                    WiFi.disconnect(true);
                    delay(100);
                    return false;
                }
            }

            // v19.0.0: 接続成功の確認をより厳密に
            if (WiFi.status() == WL_CONNECTED)
            {
                // IPアドレスが正しく割り当てられているか確認
                IPAddress ip = WiFi.localIP();
                if (ip[0] == 0)
                {
                    if (DEBUG)
                    {
                        Serial.println("\n[WiFi] IPアドレス取得失敗");
                        Serial.flush();
                    }
                    WiFi.disconnect(true);
                    return false;
                }

                if (DEBUG)
                {
                    Serial.println();
                    Serial.println("[WiFi] ✓ 接続成功!");
                    Serial.printf("  SSID: %s\n", WiFi.SSID().c_str());
                    Serial.printf("  IP: %s\n", ip.toString().c_str());
                    Serial.printf("  GW: %s\n", WiFi.gatewayIP().toString().c_str());
                    Serial.printf("  Mask: %s\n", WiFi.subnetMask().toString().c_str());
                    Serial.printf("  DNS: %s\n", WiFi.dnsIP().toString().c_str());
                    Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());
                    Serial.printf("  Channel: %d\n", WiFi.channel());
                    Serial.flush();
                }
                return true;
            }

            if (DEBUG)
            {
                Serial.printf("\n[WiFi] タイムアウト (ステータス: %d)\n", WiFi.status());
                Serial.flush();
            }
            WiFi.disconnect(true);
            delay(100);
            return false;
        }

        // v19.1.9: WiFiスキャンを行い、登録されているネットワークがあれば接続する
        bool attemptWiFiConnection()
        {
            if (DEBUG)
            {
                Serial.println("[WiFi] ========== 接続処理開始 (スキャン方式) ==========");
                Serial.flush();
            }

            // 1. 周囲のWiFiをスキャン
            if (DEBUG) Serial.println("[WiFi] ネットワークをスキャン中...");
            int n = WiFi.scanNetworks();
            
            if (n == 0) {
                if (DEBUG) Serial.println("[WiFi] ネットワークが見つかりませんでした");
                return false;
            }

            if (DEBUG) Serial.printf("[WiFi] %d 個のネットワークを検出\n", n);

            // 2. 登録済み認証情報（優先度順）とスキャン結果を照合
            for (int i = 0; i < numWifiCredentials; i++)
            {
                const auto &cred = wifiCredentials[i];
                if (strlen(cred.ssid) == 0 || strcmp(cred.ssid, "-----") == 0) continue;

                // スキャン結果の中から、この登録SSIDを探す
                for (int j = 0; j < n; ++j) {
                    // WiFi.SSID(j) returns const char* on RP2040
                    if (String(WiFi.SSID(j)) == String(cred.ssid)) {
                        if (DEBUG) {
                            Serial.printf("[WiFi] 登録済みネットワークを発見: %s (RSSI: %d)\n", WiFi.SSID(j), WiFi.RSSI(j));
                        }
                        
                        // 3. 発見したネットワークに接続試行
                        if (connectToSingleNetwork(cred.ssid, cred.password))
                        {
                            if (DEBUG)
                            {
                                Serial.println("[WiFi] ========== 接続完了 ==========");
                                Serial.flush();
                            }
                            return true;
                        }
                        // 接続に失敗した場合は、次の候補を探す
                        break; 
                    }
                }
            }

            if (DEBUG)
            {
                Serial.println("[WiFi] ========== 有効な接続先が見つかりませんでした ==========");
                Serial.flush();
            }
            return false;
        }
    } // namespace

    void initOTA()
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            ArduinoOTA.setHostname("pico-lightning-sensor");

            ArduinoOTA.onStart([]()
                               {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else
                type = "filesystem";
            if (DEBUG) Serial.println("[OTA] Start updating " + type); });
            ArduinoOTA.onEnd([]()
                             {
            if (DEBUG) Serial.println("\n[OTA] End"); });
            ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                                  {
            if (DEBUG) Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100))); });
            ArduinoOTA.onError([](ota_error_t error)
                               {
            if (DEBUG) {
                Serial.printf("[OTA] Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                else if (error == OTA_END_ERROR) Serial.println("End Failed");
            } });

            ArduinoOTA.begin();
            if (DEBUG)
                Serial.println("[OTA] Ready.");
        }
        else
        {
            if (DEBUG)
                Serial.println("[OTA] Init Failed: WiFi not connected.");
        }
    }

    String urlEncode(const char *msg)
    {
        const char *hex = "0123456789abcdef";
        String encodedMsg = "";
        while (*msg != '\0')
        {
            if (('a' <= *msg && *msg <= 'z') || ('A' <= *msg && *msg <= 'Z') || ('0' <= *msg && *msg <= '9') || *msg == '-' || *msg == '_' || *msg == '.' || *msg == '~')
            {
                encodedMsg += *msg;
            }
            else
            {
                encodedMsg += '%';
                encodedMsg += hex[*msg >> 4];
                encodedMsg += hex[*msg & 15];
            }
            msg++;
        }
        return encodedMsg;
    }

    String createTempHumParams()
    {
        float temp, hum, outdoor_temp, outdoor_hum;
        mutex_enter_blocking(&data_mutex);
        temp = State::sensors.temperature;
        hum = State::sensors.humidity;
        outdoor_temp = State::sensors.outdoorTemperature;
        outdoor_hum = State::sensors.outdoorHumidity;
        mutex_exit(&data_mutex);

        if (temp > -100)
        {
            String params = "?sheet=" + urlEncode("温湿度") + "&temp=" + String(temp, 1) + "&hum=" + String(hum, 1);
            
            // 屋外センサーの値が有効な場合は追加
            bool outdoor_valid = (outdoor_temp > -100) && (outdoor_hum > -100);
            if (outdoor_valid)
            {
                params += "&outdoor_temp=" + String(outdoor_temp, 1) + "&outdoor_hum=" + String(outdoor_hum, 1);
                if (DEBUG)
                    Serial.printf("[GAS] Adding outdoor params: outdoor_temp=%.1f, outdoor_hum=%.1f\n", outdoor_temp, outdoor_hum);
            }
            return params;
        }
        return "";
    }

    void addAuthHeaders(HTTPClient &http)
    {
        String token = SWITCHBOT_TOKEN;
        String secret = SWITCHBOT_SECRET;
        time_t t_val;
        time(&t_val);
        char t_str[15];
        sprintf(t_str, "%lu", (unsigned long)t_val);
        String t = String(t_str) + "000";
        String nonce = "";
        for (int i = 0; i < 16; i++)
            nonce += String(random(16), HEX);
        String dataToSign = token + t + nonce;
        byte hmacResult[32];
        Crypto::hmac_sha256((const uint8_t *)secret.c_str(), secret.length(), (const uint8_t *)dataToSign.c_str(), dataToSign.length(), hmacResult);
        String sign = Crypto::base64_encode(hmacResult, 32);
        http.addHeader("Authorization", token);
        http.addHeader("t", t);
        http.addHeader("nonce", nonce);
        http.addHeader("sign", sign);
    }

    void sendLineNotification(String message)
    {
        mutex_enter_blocking(&wifi_mutex);
        if (WiFi.status() != WL_CONNECTED)
        {
            mutex_exit(&wifi_mutex);
            return;
        }
        WiFiClientSecure client;
        client.setInsecure();
        HTTPClient http;
        if (http.begin(client, "https://api.line.me/v2/bot/message/push"))
        {
            http.addHeader("Content-Type", "application/json");
            http.addHeader("Authorization", "Bearer " + String(LINE_CHANNEL_ACCESS_TOKEN));
            JsonDocument doc;
            doc["to"] = LINE_USER_ID;
            JsonObject msgObj = doc["messages"].to<JsonArray>().add<JsonObject>();
            msgObj["type"] = "text";
            msgObj["text"] = message;
            String requestBody;
            serializeJson(doc, requestBody);
            http.POST(requestBody);
            http.end();
        }
        mutex_exit(&wifi_mutex);
    }

    void sendSwitchBotCommand(const char *deviceId, const char *command, const char *parameter, const char *commandType)
    {
        // Core0から呼ばれた場合はCore1にキューイングして即リターン（ブロッキングHTTPS禁止）
        if (get_core_num() == 0) {
            mutex_enter_blocking(&data_mutex);
            strncpy(pendingSwitchBotCmd.deviceId, deviceId, sizeof(pendingSwitchBotCmd.deviceId) - 1);
            strncpy(pendingSwitchBotCmd.command, command, sizeof(pendingSwitchBotCmd.command) - 1);
            strncpy(pendingSwitchBotCmd.parameter, parameter, sizeof(pendingSwitchBotCmd.parameter) - 1);
            strncpy(pendingSwitchBotCmd.commandType, commandType, sizeof(pendingSwitchBotCmd.commandType) - 1);
            pendingSwitchBotCmd.pending = true;
            mutex_exit(&data_mutex);
            return;
        }
        // Core1から呼ばれた場合は直接実行
        mutex_enter_blocking(&wifi_mutex);
        if (WiFi.status() != WL_CONNECTED)
        {
            mutex_exit(&wifi_mutex);
            return;
        }
        WiFiClientSecure secureClient;
        secureClient.setInsecure();
        HTTPClient http;
        if (http.begin(secureClient, "https://api.switch-bot.com/v1.1/devices/" + String(deviceId) + "/commands"))
        {
            addAuthHeaders(http);
            http.addHeader("Content-Type", "application/json; charset=utf-8");
            JsonDocument doc;
            doc["command"] = command;
            doc["parameter"] = parameter;
            doc["commandType"] = commandType;
            String jsonBody;
            serializeJson(doc, jsonBody);
            http.POST(jsonBody);
            http.end();
        }
        mutex_exit(&wifi_mutex);
    }

    bool parseMacAddress(const char *macStr, byte *macArray)
    {
        if (strlen(macStr) != 17)
        {
            return false;
        }
        int values[6];
        int count = sscanf(macStr, "%x:%x:%x:%x:%x:%x", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]);
        if (count != 6)
        {
            return false;
        }
        for (int i = 0; i < 6; ++i)
        {
            macArray[i] = (byte)values[i];
        }
        return true;
    }

    void sendWakeOnLan(const char *macStr)
    {
        byte mac[6];
        if (!parseMacAddress(macStr, mac))
        {
            if (DEBUG)
                Serial.printf("[WoL] Error: Invalid MAC address format: %s\n", macStr);
            return;
        }

        mutex_enter_blocking(&wifi_mutex);

        // マジックパケットを作成
        byte magicPacket[102];
        memset(magicPacket, 0xFF, 6); // 先頭6バイトを0xFFで埋める
        for (int i = 0; i < 16; i++)
        {
            memcpy(&magicPacket[6 + i * 6], mac, 6); // MACアドレスを16回繰り返す
        }

        // 複数回送信して確実性を向上（3回送信）
        int successCount = 0;
        for (int attempt = 0; attempt < 3; attempt++)
        {
            WiFiUDP udp;

            // ブロードキャストアドレスとポート9を指定
            if (udp.beginPacket(IPAddress(255, 255, 255, 255), 9))
            {
                size_t written = udp.write(magicPacket, sizeof(magicPacket));
                if (udp.endPacket() && written == sizeof(magicPacket))
                {
                    successCount++;
                    if (DEBUG)
                        Serial.printf("[WoL] Packet sent successfully (attempt %d/3)\n", attempt + 1);
                }
                else if (DEBUG)
                {
                    Serial.printf("[WoL] Packet send failed (attempt %d/3)\n", attempt + 1);
                }
            }
            else if (DEBUG)
            {
                Serial.printf("[WoL] UDP setup failed (attempt %d/3)\n", attempt + 1);
            }

            udp.stop(); // UDPを確実に停止

            if (attempt < 2)
                delay(50); // 次の送信まで少し待機（50ms）
        }

        if (DEBUG)
        {
            Serial.printf("[WoL] Magic packet transmission complete: %d/3 successful\n", successCount);
        }

        mutex_exit(&wifi_mutex);
    }

    void requestDistance_internal()
    {
        String ip;
        mutex_enter_blocking(&data_mutex);
        ip = State::childIpAddress;
        mutex_exit(&data_mutex);

        if (ip == "")
        {
            mutex_enter_blocking(&data_mutex);
            State::sensors.ultrasonicDistance = -2.0;
            mutex_exit(&data_mutex);
            return;
        }

        float dist = -4.0;
        mutex_enter_blocking(&wifi_mutex);
        HTTPClient http;
        WiFiClient client;

        // v19.0.0: エラーハンドリング強化
        if (http.begin(client, "http://" + ip + "/distance"))
        {
            http.setTimeout(600);
            http.addHeader("Connection", "close");
            int httpCode = http.GET();

            if (httpCode == HTTP_CODE_OK)
            {
                String payload = http.getString();
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, payload);

                if (error == DeserializationError::Ok)
                {
                    if (doc["distance"].is<float>() || doc["distance"].is<int>())
                    {
                        dist = doc["distance"];
                    }
                    else
                    {
                        dist = -3.0;
                    }
                }
                else
                {
                    if (DEBUG)
                        Serial.printf("[Core1] JSON parse error: %s\n", error.c_str());
                    dist = -3.0;
                }
            }
            else
            {
                if (DEBUG)
                    Serial.printf("[Core1] HTTP error: %d\n", httpCode);
                dist = (httpCode == HTTPC_ERROR_CONNECTION_FAILED) ? -2.0 : -5.0;
            }
            http.end();
        }
        else
        {
            if (DEBUG)
                Serial.println("[Core1] HTTP begin failed");
        }
        mutex_exit(&wifi_mutex);

        mutex_enter_blocking(&data_mutex);
        State::sensors.ultrasonicDistance = dist;
        mutex_exit(&data_mutex);
    }

    // v19.6.0: NTP同期を完全非同期化
    // delay(200)×60 = 12秒ブロッキングを廃止し、ループをまたぐステートマシンに変更
    void handleConnection()
    {
        static unsigned long lastNtpAttempt = 0;
        static int ntpRetryCount = 0;
        static int lastSyncMinute = -1;
        static bool ntpWaiting = false;       // NTP応答待機中フラグ
        static unsigned long ntpWaitStart = 0; // NTP待機開始時刻
        const unsigned long NTP_RETRY_INTERVAL = 15000;
        const unsigned long NTP_WAIT_TIMEOUT = 12000; // 最大12秒待機（非ブロッキング）
        const int MAX_NTP_RETRIES = 5;

        bool ntpInit;
        mutex_enter_blocking(&data_mutex);
        ntpInit = State::system.ntpInitialized;
        mutex_exit(&data_mutex);

        // --- NTP応答待機中の処理 ---
        if (ntpWaiting)
        {
            time_t now = time(nullptr);
            if (now > 1577836800 && now < 2209017600)
            {
                // 同期成功
                ntpWaiting = false;
                if (DEBUG) Serial.println("[Core1] NTP Sync Successful!");
                mutex_enter_blocking(&data_mutex);
                State::system.ntpInitialized = true;
                mutex_exit(&data_mutex);
                struct tm *si = localtime(&now);
                lastSyncMinute = si->tm_min;
                if (!ntpInit) Utils::requestBlinkLED("green", 3, 166);
                ntpRetryCount = 0;
            }
            else if (millis() - ntpWaitStart > NTP_WAIT_TIMEOUT)
            {
                // タイムアウト
                ntpWaiting = false;
                ntpRetryCount++;
                if (DEBUG) Serial.printf("[Core1] NTP Sync Timeout (%d/%d)\n", ntpRetryCount, MAX_NTP_RETRIES);
                if (ntpRetryCount >= MAX_NTP_RETRIES) ntpRetryCount = 0;
            }
            // 待機中はここで return して他の処理をブロックしない
            return;
        }

        // --- NTP同期を始めるか判定 ---
        bool shouldSync = false;
        if (!ntpInit)
        {
            shouldSync = true;
        }
        else if (WiFi.status() == WL_CONNECTED)
        {
            time_t now = time(nullptr);
            if (now > 100000)
            {
                struct tm *timeinfo = localtime(&now);
                int currentMinute = timeinfo->tm_min;
                if (currentMinute % 4 == 0 && currentMinute != lastSyncMinute)
                {
                    shouldSync = true;
                    if (DEBUG) Serial.printf("[Core1] Periodic NTP sync at minute %d\n", currentMinute);
                }
            }
        }

        if (WiFi.status() == WL_CONNECTED && shouldSync)
        {
            if (millis() - lastNtpAttempt < NTP_RETRY_INTERVAL) return;
            lastNtpAttempt = millis();

            if (DEBUG) Serial.printf("[Core1] Starting NTP sync (attempt %d/%d)\n", ntpRetryCount + 1, MAX_NTP_RETRIES);

            setenv("TZ", "JST-9", 1);
            tzset();
            NTP.begin(NTP_SERVER);

            // delay()せずにフラグだけ立てて次のループで確認
            ntpWaiting = true;
            ntpWaitStart = millis();
        }
    }

    void handleServerClient()
    {
        // v19.6.0: server.accept()はwifi_mutex不要（受信データの読み取りとは別）
        WiFiClient client = State::server.accept();

        if (client)
        {
            bool isNewChild = false;
            mutex_enter_blocking(&data_mutex);
            if (State::childIpAddress == "")
            {
                if (DEBUG)
                    Serial.println("[Core1] Child device connected.");
                isNewChild = true;
            }
            State::childIpAddress = client.remoteIP().toString();
            mutex_exit(&data_mutex);

            if (isNewChild)
            {
                Utils::requestBlinkLED("blue", 3, 166);
            }

            // 高速読み取り: 最大200msで打ち切り、delay不使用
            unsigned long timeout = millis();
            String requestLine = "";
            bool requestReceived = false;
            while (millis() - timeout < 200)
            {
                while (client.available())
                {
                    char c = client.read();
                    requestLine += c;
                    if (requestLine.endsWith("\r\n\r\n") || requestLine.endsWith("\n\n"))
                    {
                        requestReceived = true;
                        break;
                    }
                }
                if (requestReceived) break;
                delay(1); // v19.3.0: CPU解放してWatchdog / Core間通信を保護
            }
            if (!requestReceived && requestLine.length() > 10)
                requestReceived = true;

            if (!requestReceived)
            {
                if (DEBUG)
                    Serial.println("[Core1] Client request timeout or incomplete.");
                client.stop();
                return;
            }

            if (DEBUG)
                Serial.printf("[Core1] Request received from: %s\n", client.remoteIP().toString().c_str());

            // v2.0.1: リクエストパスの解析（コマンドルーティング対応）
            String path = "";
            int firstSpace = requestLine.indexOf(' ');
            int secondSpace = requestLine.indexOf(' ', firstSpace + 1);
            if (firstSpace >= 0 && secondSpace > firstSpace)
            {
                path = requestLine.substring(firstSpace + 1, secondSpace);
            }

            if (DEBUG)
                Serial.printf("[Core1] Request path: %s\n", path.c_str());

            // コマンドリクエストの処理: /api/cmd?action=xxx
            if (path.startsWith("/api/cmd"))
            {
                String action = "";
                int actionIdx = path.indexOf("action=");
                if (actionIdx >= 0)
                {
                    action = path.substring(actionIdx + 7);
                    // URLパラメータの&以降を除去
                    int ampIdx = action.indexOf('&');
                    if (ampIdx >= 0) action = action.substring(0, ampIdx);
                }

                String cmdResult = "unknown";

                if (action == "menu")
                {
                    // メニュー表示: Normalモードの場合MENUモードに切替
                    mutex_enter_blocking(&data_mutex);
                    if (State::menu.currentMode == State::MAIN_DISPLAY)
                    {
                        State::menu.currentMode = State::MENU;
                        State::menu.menuSelection = 0;
                        cmdResult = "ok";
                    }
                    else
                    {
                        // 既にメニューモードの場合はMAIN_DISPLAYに戻す
                        State::menu.currentMode = State::MAIN_DISPLAY;
                        cmdResult = "ok";
                    }
                    mutex_exit(&data_mutex);
                }
                else if (action == "cursor")
                {
                    // カーソル移動（メニュー内で次の項目へ）
                    mutex_enter_blocking(&data_mutex);
                    State::menu.menuSelection++;
                    cmdResult = "ok";
                    mutex_exit(&data_mutex);
                }
                else if (action == "history")
                {
                    // 雷履歴表示
                    mutex_enter_blocking(&data_mutex);
                    State::menu.currentMode = State::HISTORY;
                    cmdResult = "ok";
                    mutex_exit(&data_mutex);
                }
                else if (action == "backlight_temp")
                {
                    // バックライト一時点灯
                    mutex_enter_blocking(&data_mutex);
                    State::timers.backlightOn = millis();
                    cmdResult = "ok";
                    mutex_exit(&data_mutex);
                }
                else if (action == "backlight_toggle")
                {
                    // バックライト切替（常時ON/自動OFF）
                    mutex_enter_blocking(&data_mutex);
                    State::system.backlightAlwaysOn = 
                        !State::system.backlightAlwaysOn;
                    cmdResult = State::system.backlightAlwaysOn 
                        ? "always_on" : "auto_off";
                    mutex_exit(&data_mutex);
                }
                else if (action == "rgb_toggle")
                {
                    // RGB照明切替
                    mutex_enter_blocking(&data_mutex);
                    State::system.illuminationOn = 
                        !State::system.illuminationOn;
                    cmdResult = State::system.illuminationOn 
                        ? "on" : "off";
                    mutex_exit(&data_mutex);
                }
                else if (action == "resync")
                {
                    // 時刻再同期
                    requestResyncNtp = true;
                    cmdResult = "ok";
                }
                else if (action == "reboot")
                {
                    // 再起動（レスポンス送信後に実行）
                    cmdResult = "rebooting";
                }

                // コマンドレスポンス送信
                String cmdResponse = "{\"action\":\"" + action + 
                    "\",\"result\":\"" + cmdResult + "\"}";
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/json");
                client.println("Connection: close");
                client.printf("Content-Length: %d\r\n", cmdResponse.length());
                client.println();
                client.print(cmdResponse);
                client.flush();
                delay(1);
                client.stop();

                // 再起動コマンドの場合、レスポンス送信後に再起動
                if (action == "reboot")
                {
                    delay(500);
                    rp2040.reboot();
                }

                if (DEBUG)
                    Serial.printf("[Core1] Command executed: %s -> %s\n", 
                        action.c_str(), cmdResult.c_str());
                return; // コマンド処理完了
            }

            // IR送信コマンド: /api/ir/send?device=XXX&cmd=YYY
            if (path.startsWith("/api/ir/send"))
            {
                String device = "";
                String cmd = "";
                int devIdx = path.indexOf("device=");
                int cmdIdx = path.indexOf("cmd=");
                if (devIdx >= 0)
                {
                    device = path.substring(devIdx + 7);
                    int ampIdx = device.indexOf('&');
                    if (ampIdx >= 0) device = device.substring(0, ampIdx);
                }
                if (cmdIdx >= 0)
                {
                    cmd = path.substring(cmdIdx + 4);
                    int ampIdx = cmd.indexOf('&');
                    if (ampIdx >= 0) cmd = cmd.substring(0, ampIdx);
                }

                // IR送信フラグを設定（Core0で処理）
                mutex_enter_blocking(&data_mutex);
                strncpy((char*)irSendDevice, device.c_str(), 
                    sizeof(irSendDevice) - 1);
                strncpy((char*)irSendCommand, cmd.c_str(), 
                    sizeof(irSendCommand) - 1);
                irSendRequested = true;
                mutex_exit(&data_mutex);

                String irResponse = "{\"device\":\"" + device + 
                    "\",\"cmd\":\"" + cmd + "\",\"result\":\"queued\"}";
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/json");
                client.println("Connection: close");
                client.printf("Content-Length: %d\r\n", irResponse.length());
                client.println();
                client.print(irResponse);
                client.flush();
                delay(1);
                client.stop();

                if (DEBUG)
                    Serial.printf("[Core1] IR command queued: %s/%s\n", 
                        device.c_str(), cmd.c_str());
                return;
            }

            // お風呂通知: /bath_alert
            if (path.startsWith("/bath_alert"))
            {
                mutex_enter_blocking(&data_mutex);
                State::system.bathAlertActive = true;
                State::system.needsRedraw = true;
                State::system.forceMainScreenRedraw = true;
                mutex_exit(&data_mutex);
                Utils::requestBlinkLEDInfinite("white", 1000);
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: text/plain");
                client.println("Connection: close");
                client.println("Content-Length: 2\r\n");
                client.print("OK");
                client.flush();
                delay(1);
                client.stop();
                if (DEBUG)
                    Serial.println("[Core1] Bath alert received from child.");
                return;
            }

            // 距離測定終了: /distance_stop (子機からの完了通知)
            if (path.startsWith("/distance_stop"))
            {
                requestExitUltrasonic = true;
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: text/plain");
                client.println("Connection: close");
                client.println("Content-Length: 2\r\n");
                client.print("OK");
                client.flush();
                delay(1);
                client.stop();
                if (DEBUG)
                    Serial.println("[Core1] Distance stop received from child.");
                return;
            }

            // 距離データ受信: /distance_update?v=XXX (子機プッシュ)
            if (path.startsWith("/distance_update"))
            {
                int vIdx = path.indexOf("v=");
                if (vIdx >= 0) {
                    int val10 = path.substring(vIdx + 2).toInt(); // distance*10
                    float dist = (val10 >= 0) ? val10 / 10.0f : -1.0f;
                    mutex_enter_blocking(&data_mutex);
                    State::sensors.ultrasonicDistance = dist;
                    State::timers.lastDistanceUpdateTime = millis();
                    mutex_exit(&data_mutex);
                }
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: text/plain");
                client.println("Connection: close");
                client.println("Content-Length: 2\r\n");
                client.print("OK");
                client.flush();
                delay(1);
                client.stop();
                return;
            }

            // お風呂クリア: /bath_clear (子機からの終了通知)
            if (path.startsWith("/bath_clear"))
            {
                mutex_enter_blocking(&data_mutex);
                State::system.bathAlertActive = false;
                State::system.forceMainScreenRedraw = true;
                mutex_exit(&data_mutex);
                Utils::stopLEDBlink();
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: text/plain");
                client.println("Connection: close");
                client.println("Content-Length: 2\r\n");
                client.print("OK");
                client.flush();
                delay(1);
                client.stop();
                if (DEBUG)
                    Serial.println("[Core1] Bath clear received from child.");
                return;
            }

            // デフォルト: センサーデータのJSON応答（従来の動作）
            bool isFirstAccessTarget = (path == "/" || path.startsWith("/?") || path.startsWith("/api/sensors"));
            JsonDocument responseDoc;
            mutex_enter_blocking(&data_mutex);
            responseDoc["temperature"] = State::sensors.temperature;
            responseDoc["humidity"] = State::sensors.humidity;
            // 屋外温度センサーの値を追加
            if (State::system.dht22_outdoor_initialized && State::sensors.outdoorTemperature > -999.0)
            {
                responseDoc["outdoor_temperature"] = State::sensors.outdoorTemperature;
                responseDoc["outdoor_humidity"] = State::sensors.outdoorHumidity;
            }
            responseDoc["last_event_time"] = State::sensors.lastEventTime;
            responseDoc["last_event_type"] = State::sensors.lastEventType;
            if (State::sensors.lastEventType == "Lightning")
            {
                responseDoc["last_lightning_dist"] = State::sensors.lastLightningDistance;
            }
            mutex_exit(&data_mutex);

            String responseBody;
            size_t jsonSize = serializeJson(responseDoc, responseBody);

            if (jsonSize == 0)
            {
                if (DEBUG)
                    Serial.println("[Core1] Error: Failed to serialize JSON response.");
                client.println("HTTP/1.1 500 Internal Server Error\r\nConnection: close\r\n");
                client.stop();
                return;
            }

            // v19.1.0: HTTPレスポンスヘッダーの改善（高速化）
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: application/json");
            client.println("Connection: close");
            client.println("Cache-Control: no-cache");
            client.printf("Content-Length: %d\r\n", responseBody.length());
            client.println(); // 空行でヘッダー終了
            client.print(responseBody);
            client.flush(); // 送信完了を確認
            delay(1);       // 最小限の待機時間（1ms）
            client.stop();

            // ルートURLまたは/api/sensorsへの初回アクセス成功時のみ、1回だけ青色3回点滅
            if (isFirstAccessTarget)
            {
                bool shouldBlinkFirstAccess = false;
                mutex_enter_blocking(&data_mutex);
                if (!State::system.firstHttpAccessBlinkDone)
                {
                    State::system.firstHttpAccessBlinkDone = true;
                    shouldBlinkFirstAccess = true;
                }
                mutex_exit(&data_mutex);

                if (shouldBlinkFirstAccess)
                {
                    Utils::requestBlinkLED("blue", 3, 166);
                }
            }

            if (DEBUG)
                Serial.printf("[Core1] Response sent: %d bytes\n", jsonSize);
        }
    }

    bool connectToWiFi()
    {
        return attemptWiFiConnection();
    }

    // v19.1.8: WiFi再接続処理の改善（起動直後のリカバリ強化）
void checkWiFiConnection()
{
    static unsigned long lastReconnectAttempt = 0;
    static unsigned long lastCheckTime = 0;
    static int consecutiveFailures = 0;
    const unsigned long reconnectInterval = 60000;
    const unsigned long checkInterval = 5000; // 5秒ごとにチェック
    const int maxConsecutiveFailures = 3;

    // 頻繁なチェックを避ける
    if (millis() - lastCheckTime < checkInterval)
    {
        return;
    }
    lastCheckTime = millis();

    uint8_t status = WiFi.status();

    if (status != WL_CONNECTED)
    {
        if (DEBUG)
        {
            Serial.printf("[Core1] WiFi status: %d (not connected)\n", status);
        }

        bool shouldReconnect = false;
        
        // v19.1.8: 起動後5分以内は、再接続間隔を短くする（10秒）
        unsigned long currentInterval = reconnectInterval;
        if (millis() < 300000) {
            currentInterval = 10000; 
        }

        mutex_enter_blocking(&wifi_mutex);
        if (millis() - lastReconnectAttempt > currentInterval)
        {
            shouldReconnect = true;
            lastReconnectAttempt = millis();
        }
        mutex_exit(&wifi_mutex);

        if (shouldReconnect)
        {
            if (DEBUG)
                Serial.printf("[Core1] WiFi disconnected. Attempting to reconnect (Interval: %lu ms)...\n", currentInterval);

            // mutexロックなしで接続試行（WD誤検知防止）
            if (connectToWiFi())
            {
                if (DEBUG)
                    Serial.println("[Core1] WiFi reconnected successfully.");

                // IPアドレスを共有バッファに更新
                mutex_enter_blocking(&data_mutex);
                strcpy((char *)sharedIpAddress, WiFi.localIP().toString().c_str());
                State::system.ntpInitialized = false; // NTP再同期を促す
                mutex_exit(&data_mutex);

                consecutiveFailures = 0;
            }
            else
            {
                consecutiveFailures++;
                if (DEBUG)
                    Serial.printf("[Core1] Failed to reconnect (%d/%d). Will try again later.\n",
                                  consecutiveFailures, maxConsecutiveFailures);

                // 連続失敗が多い場合は待機時間を延長（Watchdog petを維持しながら待機）
                if (consecutiveFailures >= maxConsecutiveFailures)
                {
                    if (DEBUG)
                        Serial.println("[Core1] Multiple failures detected. Waiting longer...");
                    // delay(5000)はCore1のループを止めるが、WatchdogはCore0を監視するため問題なし
                    // ただしCore0のpetが届かなくなるよう長時間は危険なので1秒×5回に分割
                    for (int _w = 0; _w < 5; _w++) { delay(1000); }
                }
            }
        }
    }
    else
    {
        // 接続中の場合、失敗カウンタをリセット
        if (consecutiveFailures > 0)
        {
            consecutiveFailures = 0;
        }
    }
}
    void logDataToGoogleSheet(String params)
    {
        mutex_enter_blocking(&wifi_mutex);
        if (WiFi.status() != WL_CONNECTED || String(GAS_URL).startsWith("-----"))
        {
            mutex_exit(&wifi_mutex);
            return;
        }
        WiFiClientSecure client;
        client.setInsecure();
        HTTPClient http;
        String url = String(GAS_URL) + params;

        if (http.begin(client, url))
        {
            http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
            http.setTimeout(5000); // タイムアウトを5秒に設定してフリーズを防止
            http.GET();
            http.end();
        }
        mutex_exit(&wifi_mutex);
    }

    void getSwitchBotDeviceList_internal()
    {
        mutex_enter_blocking(&wifi_mutex);
        if (WiFi.status() != WL_CONNECTED)
        {
            mutex_exit(&wifi_mutex);
            return;
        }
        WiFiClientSecure secureClient;
        secureClient.setInsecure();
        HTTPClient http;
        if (http.begin(secureClient, "https://api.switch-bot.com/v1.1/devices"))
        {
            addAuthHeaders(http);
            if (http.GET() == HTTP_CODE_OK)
            {
                if (DEBUG)
                {
                    Serial.println("\n--- SwitchBot Device List ---");
                    Serial.println(http.getString());
                }
            }
            http.end();
        }
        mutex_exit(&wifi_mutex);
    }

    void sendLineTestMessage_internal()
    {
        sendLineNotification("これは雷センサーからのテスト通知です。");
    }

    void handleWolPolling()
    {
        const unsigned long pollingInterval = 5000;
        static unsigned long previousMillis = 0;

        if (millis() - previousMillis < pollingInterval) return;
        previousMillis = millis();

        if (String(GAS_URL_WOL).startsWith("-----") || WiFi.status() != WL_CONNECTED) return;

        if (DEBUG) Serial.println("[Core1-WoL] Polling GAS for trigger...");

        // v19.6.0: wifi_mutexをリクエストごとに取得・解放（長期保持廃止）
        String payload;
        bool success = false;

        // --- シグナルチェック ---
        String signalCheckUrl = String(GAS_URL_WOL) + "?action=signal";
        for (int retry = 0; retry < 2 && !success; retry++)
        {
            if (retry > 0) delay(100);

            mutex_enter_blocking(&wifi_mutex);
            {
                WiFiClientSecure client;
                HTTPClient http;
                client.setInsecure();
                if (http.begin(client, signalCheckUrl))
                {
                    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
                    http.setTimeout(5000);
                    http.addHeader("Connection", "close");
                    int httpCode = http.GET();
                    if (httpCode == HTTP_CODE_OK)
                    {
                        payload = http.getString();
                        payload.trim();
                        success = true;
                        if (DEBUG && payload != "") Serial.printf("[Core1-WoL] Signal: [%s]\n", payload.c_str());
                    }
                    else if (DEBUG) Serial.printf("[Core1-WoL] Signal HTTP error: %d\n", httpCode);
                    http.end();
                }
            }
            mutex_exit(&wifi_mutex); // リクエスト完了後すぐ解放
        }

        if (!success || payload != "TRIGGER") return;

        if (DEBUG) Serial.println("  -> Trigger detected! Getting command...");
        delay(100);

        // --- コマンド取得 ---
        String commandGetUrl = String(GAS_URL_WOL) + "?action=command";
        success = false;
        for (int retry = 0; retry < 2 && !success; retry++)
        {
            if (retry > 0) delay(100);

            mutex_enter_blocking(&wifi_mutex);
            {
                WiFiClientSecure client2;
                HTTPClient http2;
                client2.setInsecure();
                if (http2.begin(client2, commandGetUrl))
                {
                    http2.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
                    http2.setTimeout(5000);
                    http2.addHeader("Connection", "close");
                    int httpCode = http2.GET();
                    if (httpCode == HTTP_CODE_OK)
                    {
                        payload = http2.getString();
                        payload.trim();
                        success = true;
                        if (DEBUG) Serial.printf("  -> Command: [%s]\n", payload.c_str());
                        if (payload == "デスクトップPC起動") wolTriggerAction = 1;
                        else if (payload == "サーバーPC起動") wolTriggerAction = 2;
                    }
                    else if (DEBUG) Serial.printf("[Core1-WoL] Command HTTP error: %d\n", httpCode);
                    http2.end();
                }
            }
            mutex_exit(&wifi_mutex); // リクエスト完了後すぐ解放
        }

        if (!success) return;

        // --- ACK送信 ---
        delay(100);
        mutex_enter_blocking(&wifi_mutex);
        {
            WiFiClientSecure client3;
            HTTPClient http3;
            client3.setInsecure();
            String ackUrl = String(GAS_URL_WOL) + "?action=ack";
            if (http3.begin(client3, ackUrl))
            {
                http3.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
                http3.setTimeout(3000);
                http3.addHeader("Connection", "close");
                http3.GET();
                http3.end();
                if (DEBUG) Serial.println("  -> ACK sent");
            }
        }
        mutex_exit(&wifi_mutex);
    }

    void handleActionFlags()
    {
        if (requestSendLightningAlert)
        {
            if (DEBUG)
                Serial.printf("[Core1] Received lightning alert request for distance: %dkm\n", lightningDistanceForAlert);
            char msg[60];
            snprintf(msg, sizeof(msg), "雷を検知しました！\n距離: 約%dkm", lightningDistanceForAlert);
            sendLineNotification(msg);
            String params = "?sheet=" + urlEncode("雷の受信履歴") + "&dist=" + String(lightningDistanceForAlert);
            logDataToGoogleSheet(params);
            requestSendLightningAlert = false;
        }
        if (requestSendWolDesktop)
        {
            requestSendWolDesktop = false;
            sendWakeOnLan(MAC_DESKTOP);
            Utils::requestBlinkLED("red", 3, 166); // v19.2.0: LED点滅間隔を統一(166ms)
        }
        if (requestSendWolServer)
        {
            requestSendWolServer = false;
            sendWakeOnLan(MAC_SERVER);
            Utils::requestBlinkLED("red", 3, 166); // v19.2.0: LED点滅間隔を統一(166ms)
        }
        if (requestManualLog)
        {
            logDataToGoogleSheet(createTempHumParams());
            requestManualLog = false;
        }
        if (requestGetSwitchbotIds)
        {
            getSwitchBotDeviceList_internal();
            requestGetSwitchbotIds = false;
        }
        if (requestSendLineTest)
        {
            sendLineTestMessage_internal();
            requestSendLineTest = false;
        }
        if (requestResyncNtp)
        {
            mutex_enter_blocking(&data_mutex);
            State::system.ntpInitialized = false;
            mutex_exit(&data_mutex);
            requestResyncNtp = false;
        }
        if (requestGetUltrasonicDistance)
        {
            requestGetUltrasonicDistance = false;
            // プッシュモデルに切り替えたためここでは何もしない
        }
        if (requestSendDistanceStart)
        {
            requestSendDistanceStart = false;
            String ip;
            mutex_enter_blocking(&data_mutex);
            ip = State::childIpAddress;
            mutex_exit(&data_mutex);
            if (ip != "") {
                WiFiClient startClient;
                startClient.setTimeout(1000); // v19.3.1: 接続タイムアウトを1秒に制限
                if (startClient.connect(ip.c_str(), 80)) {
                    startClient.println("GET /distance_start HTTP/1.1");
                    startClient.print("Host: "); startClient.println(ip);
                    startClient.println("Connection: close\r\n");
                    startClient.flush();
                    startClient.stop();
                }
            }
        }
        if (requestSendDistanceStop)
        {
            requestSendDistanceStop = false;
            requestGetUltrasonicDistance = false; // 未処理のポーリングリクエストもキャンセル
            String ip;
            mutex_enter_blocking(&data_mutex);
            ip = State::childIpAddress;
            mutex_exit(&data_mutex);
            if (ip != "") {
                WiFiClient stopClient;
                stopClient.setTimeout(1000); // v19.3.1: 接続タイムアウトを1秒に制限
                if (stopClient.connect(ip.c_str(), 80)) {
                    stopClient.println("GET /distance_stop HTTP/1.1");
                    stopClient.print("Host: "); stopClient.println(ip);
                    stopClient.println("Connection: close\r\n");
                    stopClient.flush();
                    stopClient.stop();
                }
            }
        }
        // SwitchBotコマンドキュー処理 (Core0のメニュー操作から非同期実行)
        if (pendingSwitchBotCmd.pending) {
            char deviceId[64] = {0}, command[32] = {0}, parameter[64] = {0}, commandType[16] = {0}; // v19.3.0: NULL終端保証
            mutex_enter_blocking(&data_mutex);
            strncpy(deviceId,     pendingSwitchBotCmd.deviceId,     sizeof(deviceId) - 1);
            strncpy(command,      pendingSwitchBotCmd.command,      sizeof(command) - 1);
            strncpy(parameter,    pendingSwitchBotCmd.parameter,    sizeof(parameter) - 1);
            strncpy(commandType,  pendingSwitchBotCmd.commandType,  sizeof(commandType) - 1);
            pendingSwitchBotCmd.pending = false;
            mutex_exit(&data_mutex);
            sendSwitchBotCommand(deviceId, command, parameter, commandType);
        }
        // お風呂クリア通知 (Core0のボタン操作から非同期送信)
        if (requestSendBathClear) {
            requestSendBathClear = false;
            WiFiClient clearClient;
            clearClient.setTimeout(1000); // v19.3.1: 接続タイムアウトを1秒に制限
            if (clearClient.connect(CHILD_IP, CHILD_PORT)) {
                clearClient.println("GET /bath_clear HTTP/1.1");
                clearClient.print("Host: "); clearClient.println(CHILD_IP);
                clearClient.println("Connection: close\r\n");
                clearClient.flush();
                clearClient.stop();
            }
        }
    }

    void update()
    {
        handleConnection();
        handleServerClient();
        checkWiFiConnection();
        ArduinoOTA.handle();
        handleWolPolling();
        handleActionFlags();
    }

    void init()
    {
        if (DEBUG)
        {
            Serial.println("[WiFi] ========== 初期化開始 ==========");
            Serial.flush();
        }

        if (connectToWiFi())
        {
            mutex_enter_blocking(&data_mutex);
            strcpy((char *)sharedIpAddress, WiFi.localIP().toString().c_str());
            mutex_exit(&data_mutex);
            initOTA();
            State::server.begin();

            if (DEBUG)
            {
                Serial.println("[WiFi] サーバー起動完了");
                Serial.flush();
            }
        }
        else
        {
            if (DEBUG)
            {
                Serial.println("[WiFi] 初期接続に失敗しました");
                Serial.flush();
            }
        }
    }
}

//================================================================
// センサー管理 (Core0)
//================================================================
namespace Sensors
{
    void handleLightningInterrupt()
    {
        State::lightningInterruptFlag = true;
    }

    void addHistoryRecord(String type, int distance, const char *timestamp)
    {
        mutex_enter_blocking(&data_mutex);
        strcpy(State::history.records[State::history.index].timestamp, timestamp);
        State::history.records[State::history.index].type = type;
        State::history.records[State::history.index].distance = distance;
        State::history.index = (State::history.index + 1) % HISTORY_SIZE;
        if (State::history.count < HISTORY_SIZE)
        {
            State::history.count++;
        }
        mutex_exit(&data_mutex);
    }

    void handleLightning()
    {
        State::lightningInterruptFlag = false;

        delay(5);
        int intVal = State::lightning.readInterruptReg();
        if (DEBUG)
            Serial.printf("[Core0-Sensor] Interrupt Register: 0x%02X\n", intVal);

        char timestamp[20];
        if (intVal == 0x01 || intVal == 0x08)
        {
            time_t now = time(nullptr);
            if (now > 100000)
            {
                strftime(timestamp, sizeof(timestamp), "%m/%d %H:%M", localtime(&now));
                mutex_enter_blocking(&data_mutex);
                strcpy(State::sensors.lastEventTime, timestamp);
                mutex_exit(&data_mutex);
            }
            else
            {
                strcpy(timestamp, "Time N/A");
                mutex_enter_blocking(&data_mutex);
                strcpy(State::sensors.lastEventTime, timestamp);
                mutex_exit(&data_mutex);
            }
        }

        if (intVal == 0x01)
        {
            if (DEBUG)
                Serial.println("[Core0-Sensor] Noise detected.");
            mutex_enter_blocking(&data_mutex);
            State::sensors.lastEventType = "Noise";
            if (State::system.as3935_initialized && State::system.currentNoiseLevel < 7)
            {
                State::system.currentNoiseLevel++;
                State::lightning.setNoiseLevel(State::system.currentNoiseLevel);
            }
            mutex_exit(&data_mutex);
            addHistoryRecord("Noise", 0, timestamp);
        }
        else if (intVal == 0x08)
        {
            if (DEBUG)
                Serial.println("[Core0-Sensor] Lightning detected.");
            Utils::requestBlinkLED("yellow", 3, 166); // ノンブロッキング呼び出しに変更
            int distance = State::lightning.distanceToStorm();
            mutex_enter_blocking(&data_mutex);
            State::sensors.lastLightningDistance = distance;
            State::sensors.lastEventType = "Lightning";
            // Core1に通知処理を依頼するためのフラグを立てる
            lightningDistanceForAlert = distance;
            requestSendLightningAlert = true;
            mutex_exit(&data_mutex);
            addHistoryRecord("Lightning", distance, timestamp);
        }
        mutex_enter_blocking(&data_mutex);
        State::system.forceMainScreenRedraw = true;
        mutex_exit(&data_mutex);
    }

    // 温湿度センサーの値を更新する
    void updateDht()
    {
        static int dht22FailCount = 0;  // DHT22連続失敗カウント
        
        // --- DHT20（室内）の読み取り ---
        bool dht_ok;
        mutex_enter_blocking(&data_mutex);
        dht_ok = State::system.dht20_initialized;
        mutex_exit(&data_mutex);

        if (dht_ok)
        {
            float temp_raw = State::dht20.getTemperature();
            float hum_raw = State::dht20.getHumidity();

            if (!isnan(temp_raw) && !isnan(hum_raw))
            {
                mutex_enter_blocking(&data_mutex);
                State::sensors.temperature = round(temp_raw * 10.0) / 10.0;
                if (hum_raw >= 0 && hum_raw <= 1.0)
                {
                    hum_raw *= 100.0;
                }
                State::sensors.humidity = round(hum_raw * 10.0) / 10.0;
                mutex_exit(&data_mutex);
            }
        }
        
        // --- DHT22（屋外）の読み取り ---
        // 最小読み取り間隔を遵守（DHT22仕様：2秒以上）
        unsigned long currentTime = millis();
        if (currentTime - lastDHT22ReadTime < DHT22_MIN_READ_INTERVAL_MS)
        {
            if (DEBUG)
                Serial.printf("[Core0-Sensor] DHT22: Skipping read (too soon, %lu ms since last)\n", 
                             currentTime - lastDHT22ReadTime);
            // 前回値がある場合は維持
            if (lastValidTemp > -999.0 && lastValidHum > -999.0)
            {
                mutex_enter_blocking(&data_mutex);
                State::sensors.outdoorTemperature = lastValidTemp;
                State::sensors.outdoorHumidity = lastValidHum;
                mutex_exit(&data_mutex);
            }
            return;
        }
        
        // リトライなし - 1回だけ読み取る（Core0ブロック防止）
        float hum_raw = g_dht22.readHumidity();
        float temp_raw = g_dht22.readTemperature();
        bool readSuccess = (!isnan(hum_raw) && !isnan(temp_raw));
        
        lastDHT22ReadTime = currentTime; // 読み取り試行時刻を更新
        
        if (!readSuccess)
        {
            dht22FailCount++;
            if (DEBUG)
                Serial.printf("[Core0-Sensor] DHT22: All retries failed, NaN (%d/5)\n", dht22FailCount);
            
            // 前回の有効値がある場合は維持（一時的なエラーに対する耐性）
            if (lastValidTemp > -999.0 && lastValidHum > -999.0)
            {
                if (DEBUG)
                    Serial.printf("[Core0-Sensor] DHT22: Using last valid values (T=%.1f, H=%.1f)\n",
                                 lastValidTemp, lastValidHum);
                mutex_enter_blocking(&data_mutex);
                State::sensors.outdoorTemperature = lastValidTemp;
                State::sensors.outdoorHumidity = lastValidHum;
                mutex_exit(&data_mutex);
            }
            
            // 5回連続失敗で切断扱い
            if (dht22FailCount >= 5)
            {
                mutex_enter_blocking(&data_mutex);
                if (State::system.dht22_outdoor_initialized)
                {
                    State::system.dht22_outdoor_initialized = false;
                    State::sensors.outdoorTemperature = -999.0;
                    State::sensors.outdoorHumidity = -999.0;
                    lastValidTemp = -999.0;  // 前回値もクリア
                    lastValidHum = -999.0;
                    if (DEBUG)
                        Serial.println("[Core0-Sensor] DHT22 disconnected");
                }
                mutex_exit(&data_mutex);
            }
            return;
        }
        
        dht22FailCount = 0;
        
        // キャリブレーション適用
        float temp_calibrated = temp_raw + DHT22_TEMP_OFFSET;
        float hum_calibrated = hum_raw + DHT22_HUM_OFFSET;
        
        if (hum_calibrated < 0.0) hum_calibrated = 0.0;
        if (hum_calibrated > 100.0) hum_calibrated = 100.0;

        if (DEBUG)
            Serial.printf("[Core0-Sensor] DHT22: T=%.1f, H=%.1f\n", temp_calibrated, hum_calibrated);
        
        // 有効範囲チェック
        if (temp_calibrated >= -40.0 && temp_calibrated <= 80.0 && hum_calibrated >= 0.0 && hum_calibrated <= 100.0)
        {
            mutex_enter_blocking(&data_mutex);
            State::system.dht22_outdoor_initialized = true;
            State::sensors.outdoorTemperature = round(temp_calibrated * 10.0) / 10.0;
            State::sensors.outdoorHumidity = round(hum_calibrated * 10.0) / 10.0;
            mutex_exit(&data_mutex);
            
            // 有効値を保存
            lastValidTemp = State::sensors.outdoorTemperature;
            lastValidHum = State::sensors.outdoorHumidity;
        }
    }

    void update()
    {
        if (State::lightningInterruptFlag)
        {
            handleLightning();
        }
    }

    // センサーのキャリブレーションを実行する関数
    void calibrateSensor()
    {
        mutex_enter_blocking(&data_mutex);
        bool as3935_ok = State::system.as3935_initialized;
        mutex_exit(&data_mutex);

        if (as3935_ok)
        {
            if (State::lightning.calibrateOsc())
            {
                if (DEBUG)
                    Serial.println("[Core0-Sensor] Antenna tuning successful.");
            }
            else
            {
                if (DEBUG)
                    Serial.println("[Core0-Sensor] Antenna tuning failed.");
            }
        }
    }

    void init()
    {
        if (DEBUG)
            Serial.println("[Core0] Initializing Sensors...");

        if (DEBUG)
            Serial.print("  - Scanning for DHT20... ");
        Wire.beginTransmission(0x38);
        if (Wire.endTransmission() == 0)
        {
            if (DEBUG)
                Serial.print("found. Initializing... ");
            if (State::dht20.begin() == 0)
            {
                State::system.dht20_initialized = true;
                if (DEBUG)
                    Serial.println("OK.");
            }
            else
            {
                if (DEBUG)
                    Serial.println("FAILED.");
            }
        }
        else
        {
            if (DEBUG)
                Serial.println("NOT found.");
        }

        delay(100);

        if (DEBUG)
            Serial.print("  - Scanning for AS3935... ");
        Wire.beginTransmission(Pins::AS3935_ADDR);
        if (Wire.endTransmission() == 0)
        {
            if (DEBUG)
                Serial.print("found. Initializing... ");
            if (State::lightning.begin(Wire))
            {
                State::system.as3935_initialized = true;
                if (DEBUG)
                    Serial.println("OK.");
                if (DEBUG)
                    Serial.println("    - Configuring AS3935 sensor...");
                State::lightning.resetSettings();
                State::lightning.setIndoorOutdoor(OUTDOOR);
                State::lightning.setNoiseLevel(INITIAL_NOISE_LEVEL);
                State::system.currentNoiseLevel = INITIAL_NOISE_LEVEL;
                State::lightning.watchdogThreshold(LIGHTNING_WATCHDOG_THRESHOLD);
                State::lightning.spikeRejection(LIGHTNING_SPIKE_REJECTION);
            }
            else
            {
                if (DEBUG)
                    Serial.println("FAILED.");
            }
        }
        else
        {
            if (DEBUG)
                Serial.println("NOT found.");
        }

        // DHT22屋外温度センサーの初期化
        if (DEBUG)
            Serial.println("  - Initializing DHT22 outdoor sensor (GP17)...");
        g_dht22.begin();
        delay(1000);  // センサー安定化待機（1秒）
        State::system.dht22_outdoor_initialized = false;  // 最初の読み取り成功後にtrueに
        if (DEBUG)
            Serial.println("    - DHT22 begin complete (will verify on first read).");

        if (DEBUG)
            Serial.println("[Core0] Sensor Initialization Complete.");
    }
}

//================================================================
// 入力処理 (Core0)
//================================================================
namespace Input
{
    uint8_t buttonState = HIGH;
    uint8_t lastButtonState = HIGH;
    unsigned long lastDebounceTime = 0;
    unsigned long debounceDelay = 50;
    unsigned long buttonPressTime = 0;
    bool longPressTriggered = false;

    void handleButton()
    {
        int reading = digitalRead(Pins::BUTTON);

        if (reading != lastButtonState)
        {
            lastDebounceTime = millis();
        }

        if ((millis() - lastDebounceTime) > debounceDelay)
        {
            if (reading != buttonState)
            {
                buttonState = reading;
                if (buttonState == LOW)
                {
                    buttonPressTime = millis();
                    longPressTriggered = false;
                }
                else
                {
                    if (!longPressTriggered)
                    {
                        // v19.5.0: try_enterでデッドロック回避
                        uint32_t owner_btn;
                        unsigned long btnSpin = millis();
                        bool btnAcquired = false;
                        while (millis() - btnSpin < 500) {
                            if (mutex_try_enter(&data_mutex, &owner_btn)) {
                                btnAcquired = true;
                                break;
                            }
                            petWatchdog();
                            delayMicroseconds(100);
                        }
                        if (!btnAcquired) {
                            // mutex取得失敗：このフレームはスキップ
                        } else {
                        State::timers.lastActivity = millis();
                        State::system.needsRedraw = true;
                        if (State::system.bathAlertActive)
                        {
                            // お風呂通知中: 短押しでクリア (バックライト処理をスキップ)
                            State::system.bathAlertActive = false;
                            State::system.forceMainScreenRedraw = true;
                            mutex_exit(&data_mutex);
                            btnAcquired = false;
                            Utils::stopLEDBlink();
                            requestSendBathClear = true; // Core1に子機への通知を依頼
                        }
                        else if (State::menu.currentMode == State::MAIN_DISPLAY)
                        {
                            if (!State::system.backlightAlwaysOn)
                            {
                                // 修正: 既にバックライトが点灯中（タイマー稼働中）の場合は何もしない
                                if (State::timers.backlightOn == 0)
                                {
                                    Utils::startBacklightFade(true); // v19.0.0: PWM制御でフェードイン
                                    State::timers.backlightOn = millis();
                                }
                            }
                        }
                        else if (State::menu.currentMode == State::HISTORY || State::menu.currentMode == State::SENSOR_DIAGNOSTICS)
                        {
                            // 短押しは無効
                        }
                        else
                        {
                            int count = 0, currentSelection = 0;
                            Menu::getCurrentMenu(count, currentSelection);
                            if (count > 0)
                            {
                                if (State::menu.currentMode == State::DEVICE_CONTROL)
                                {
                                    State::menu.commandSelection = (State::menu.commandSelection + 1) % count;
                                }
                                else
                                {
                                    State::menu.menuSelection = (State::menu.menuSelection + 1) % count;
                                }
                            }
                        }
                        if (btnAcquired) mutex_exit(&data_mutex);
                        } // if (btnAcquired)
                    }
                }
            }
        }

        if (buttonState == LOW && !longPressTriggered && (millis() - buttonPressTime > LONG_PRESS_DURATION_MS))
        {
            longPressTriggered = true;
            // v19.5.0: try_enterでデッドロック回避
            uint32_t owner_lp;
            unsigned long lpSpin = millis();
            bool lpAcquired = false;
            while (millis() - lpSpin < 500) {
                if (mutex_try_enter(&data_mutex, &owner_lp)) {
                    lpAcquired = true;
                    break;
                }
                petWatchdog();
                delayMicroseconds(100);
            }
            if (!lpAcquired) {
                lastButtonState = reading;
                return;
            }
            State::timers.lastActivity = millis();
            State::system.needsRedraw = true;
            switch (State::menu.currentMode)
            {
            case State::MAIN_DISPLAY:
                Menu::changeMode(State::MENU);
                break;
            case State::HISTORY:
            case State::ULTRASONIC_MONITOR:
            case State::SENSOR_DIAGNOSTICS:
                // ULTRASONIC_MONITOR 終了時は子機にも通知 (フラグ経由でCore1が送信)
                if (State::menu.currentMode == State::ULTRASONIC_MONITOR) {
                    requestSendDistanceStop = true;
                }
                Menu::changeMode(State::MAIN_DISPLAY);
                break;
            default:
            {
                int count = 0, currentSelection = 0;
                const Menu::MenuItem *menu = Menu::getCurrentMenu(count, currentSelection);
                if (menu && currentSelection < count)
                {
                    mutex_exit(&data_mutex);
                    menu[currentSelection].action();
                    return;
                }
                break;
            }
            }
            mutex_exit(&data_mutex);
        }
        lastButtonState = reading;
    }
}

//================================================================
// メイン制御 (Core0)
//================================================================
namespace Menu
{
    void changeMode(State::Mode newMode)
    {
        // この関数は既にMutexで保護されたコンテキストから呼ばれるか、
        // action()から呼ばれるので、ここではMutexをかけない
        State::menu.currentMode = newMode;
        State::menu.menuSelection = 0;
        State::menu.commandSelection = 0;
        State::system.needsRedraw = true;
        State::timers.lastActivity = millis();
        if (newMode == State::MAIN_DISPLAY)
        {
            State::system.forceMainScreenRedraw = true;
        }
    }

    void checkInactivity()
    {
        // v19.5.0: try_enterスピンでデッドロック回避
        uint32_t owner;
        if (!mutex_try_enter(&data_mutex, &owner)) return;
        if (State::menu.currentMode != State::MAIN_DISPLAY &&
            State::menu.currentMode != State::ULTRASONIC_MONITOR &&
            State::menu.currentMode != State::SENSOR_DIAGNOSTICS)
        {
            if (millis() - State::timers.lastActivity > INACTIVITY_TIMEOUT_MS)
            {
                changeMode(State::MAIN_DISPLAY);
            }
        }
        mutex_exit(&data_mutex);
    }
}

//================================================================
// 定期実行タスク (Core0)
//================================================================
void handlePeriodicTasks()
{
    Utils::updateBacklightFade();
    Utils::handleLEDBlink(); // LED点滅処理の更新

    // 温湿度センサーの読み取り (3秒ごと、500msオフセット)
    static unsigned long lastDhtRead = 500;
    if (millis() - lastDhtRead > SENSOR_READ_INTERVAL_MS)
    {
        lastDhtRead = millis();
        petWatchdog(); // DHT読み取り前にpet（読み取りに最大2.5秒かかる場合がある）
        Sensors::updateDht();
        petWatchdog(); // DHT読み取り後にpet
    }

    // v19.0.0: バックライト自動消灯チェック（フェードアウト）
    if (!State::system.backlightAlwaysOn &&
        State::timers.backlightOn > 0 &&
        !State::system.backlightFading &&
        State::system.backlightCurrentBrightness > 0)
    {
        if (millis() - State::timers.backlightOn > BACKLIGHT_DURATION_MS)
        {
            Utils::startBacklightFade(false);
            State::timers.backlightOn = 0;
        }
    }

    // スプレッドシートへの定期記録 (毎時10分ごと)
    static int lastLoggedMinute = -1;
    bool ntpReady;
    mutex_enter_blocking(&data_mutex);
    ntpReady = State::system.ntpInitialized;
    mutex_exit(&data_mutex);

    if (ntpReady)
    {
        time_t now = time(nullptr);
        struct tm *timeinfo = localtime(&now);
        int currentMinute = timeinfo->tm_min;

        if (currentMinute % 10 == 0 && currentMinute != lastLoggedMinute)
        {
            lastLoggedMinute = currentMinute;
            requestManualLog = true; // 手動ログと同じ仕組みで記録をリクエスト
            if (DEBUG)
                Serial.printf("[Core0] Triggering periodic log for minute: %d\n", currentMinute);
        }
    }
}

// ★フリーズ後に実行される自動再起動処理
void error_loop()
{
    pinMode(Pins::LCD_BACKLIGHT, OUTPUT);
    digitalWrite(Pins::LCD_BACKLIGHT, HIGH);
    State::lcd.begin(LCD_COLS, LCD_ROWS);

    // エラーメッセージを表示（3回点滅してからリブート）
    for (int i = 0; i < 3; i++)
    {
        State::lcd.clear();
        Display::printLcdLine(0, "  Crash Detected! ");
        Display::printLcdLine(1, "      Error!      ");
        Display::printLcdLine(2, "  Auto Rebooting  ");
        delay(500);
        State::lcd.clear();
        delay(500);
    }

    // 最後にリブートメッセージを表示してから再起動
    State::lcd.clear();
    Display::printLcdLine(0, "  Crash Detected! ");
    Display::printLcdLine(1, "   Rebooting...   ");
    delay(1000);

    // watchdog_rebootで自動再起動
    watchdog_reboot(0, 0, 0);
}

//================================================================
// Core1 (通信・Watchdog担当)
//================================================================
void setup1()
{
    Serial.begin(115200);
    if (DEBUG)
        Serial.println("[Core1] Setup started.");
    Network::init();
    core1_wifi_complete = true; // Core0に通知
    if (DEBUG)
        Serial.println("[Core1] WiFi init complete. Waiting for Core0...");
    while (!core0_hw_init_complete)
    {
        sleep_ms(100);
    }
    if (DEBUG)
        Serial.println("[Core1] Core0 setup complete. Starting main loop.");
}

void loop1()
{
    // Watchdog: Core0がフリーズしていないか監視
    // v19.5.0: Core0の起動完了後のみ監視開始（起動時の誤点火防止）
    if (core0_hw_init_complete && millis() - lastCore0Pet > FREEZE_TIMEOUT_MS)
    {
        if (DEBUG)
            Serial.println("[Core1] Watchdog: Core0 freeze detected! Rebooting...");
        watchdog_hw->scratch[0] = CRASH_MAGIC_VALUE; // クラッシュしたことを記録
        watchdog_reboot(0, 0, 0);
    }

    // ★ デバッグ情報の定期的なシリアル出力
    if (DEBUG)
    {
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint > 2000)
        { // 2秒ごとに出力
            lastDebugPrint = millis();
            char ipAddr[16];
            mutex_enter_blocking(&data_mutex);
            strcpy(ipAddr, (const char *)sharedIpAddress);
            mutex_exit(&data_mutex);
            uint8_t status = WiFi.status();
            const char *statusStr = (status == WL_CONNECTED) ? "Connected" : "Disconnected";
            Serial.printf("[Core1 Network Status] WiFi: %s, IP: %s\n", statusStr, ipAddr);
        }
    }

    Network::update();
    delay(10); // Core1のループ負荷を軽減
}

//================================================================
// Core0 (UI/センサー担当)
//================================================================
void setup()
{
    // ★★★ フリーズ後の再起動かチェック ★★★
    if (watchdog_hw->scratch[0] == CRASH_MAGIC_VALUE)
    {
        watchdog_hw->scratch[0] = 0; // フラグをクリア
        error_loop();                // v19.2.0: エラー表示後に自動再起動
    }

    Serial.begin(115200);
    delay(100);
    if (DEBUG)
        Serial.println("--- System Booting (Core0) ---");

    // ★★★ ハードウェアの初期化 ★★★
    if (DEBUG)
        Serial.println("[Core0] Initializing Hardware...");
    pinMode(Pins::LCD_BACKLIGHT, OUTPUT);
    pinMode(Pins::LED_R, OUTPUT);
    pinMode(Pins::LED_G, OUTPUT);
    pinMode(Pins::LED_B, OUTPUT);
    pinMode(Pins::BUTTON, INPUT_PULLUP);
    pinMode(Pins::LIGHTNING_IRQ, INPUT);

    Wire.setSDA(Pins::I2C_SDA);
    Wire.setSCL(Pins::I2C_SCL);
    Wire.begin();

    // v19.0.0: バックライトのPWM初期化
    Utils::setBacklightBrightness(255);
    Display::init(); // "System Starting..." is shown here.

    // ★★★ 起動演出 ★★★
    Utils::setRGB(255, 255, 255);
    unsigned long bootAnimStart =  millis();

    // ★★★ センサーと割り込みの初期化 ★★★
    Sensors::init();

    if (State::system.as3935_initialized)
    {
        if (DEBUG)
            Serial.println("[Core0] Attaching lightning sensor interrupt...");
        attachInterrupt(digitalPinToInterrupt(Pins::LIGHTNING_IRQ), Sensors::handleLightningInterrupt, RISING);
    }

    // IR Remote Init
    IRRemote::init();

    // 子機IPアドレスの初期設定 (接続前から既知のIPを使用)
    // v19.4.0: フリーズ原因（ヒープ破壊）防止のため、ミューテックスで保護
    mutex_enter_blocking(&data_mutex);
    State::childIpAddress = CHILD_IP;
    mutex_exit(&data_mutex);

    // ★★★ 起動演出の終了（1秒間待機） ★★★
    while (millis() - bootAnimStart < 1000) {
        delay(10);
    }

    // ★★★ メイン画面へ移行（起動メッセージ消去） ★★★
    Menu::changeMode(State::MAIN_DISPLAY);

    // 先に一度画面を描画して「System Starting...」を消す
    Display::update();

    // バックライトのフェードアウト開始とLED消灯
    if (!State::system.backlightAlwaysOn)
    {
        Utils::startBacklightFade(false); // フェードアウトさせる
    }
    Utils::setRGB(0, 0, 0);

    if (DEBUG)
        Serial.println("--- [Core0] Boot Complete ---");
    core0_hw_init_complete = true; // Core1にハードウェア初期化完了を通知
}
void loop()
{
    petWatchdog(); // Watchdogをリセット（最優先）

    // ★ デバッグ情報の定期的なシリアル出力
    if (DEBUG)
    {
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint > 2000)
        { // 2秒ごとに出力
            lastDebugPrint = millis();
            mutex_enter_blocking(&data_mutex);
            float temp = State::sensors.temperature;
            float hum = State::sensors.humidity;
            String eventType = State::sensors.lastEventType;
            mutex_exit(&data_mutex);
            Serial.printf("[Core0 Sensor Status] Temp: %.1f C, Humidity: %.1f %%, Last Event: %s\n", temp, hum, eventType.c_str());
        }
    }

    // ★ 起動後の初回ログ
    bool ntpReady;
    bool initialLogDone;
    mutex_enter_blocking(&data_mutex);
    ntpReady = State::system.ntpInitialized;
    initialLogDone = State::system.initialLogSent;
    mutex_exit(&data_mutex);

    if (ntpReady && !initialLogDone)
    {
        requestManualLog = true;
        mutex_enter_blocking(&data_mutex);
        State::system.initialLogSent = true;
        mutex_exit(&data_mutex);
        if (DEBUG)
            Serial.println("[Core0] Triggering initial spreadsheet log.");
    }

    // --- 常時実行するタスク ---
    Input::handleButton(); // ボタン入力は常に監視
    Sensors::update();     // 雷割り込みフラグは常にチェック
    IRRemote::handle();    // IR受信チェック

    // v19.0.0: バックライトのPWMフェード更新（フェード中のみ高頻度実行）
    if (State::system.backlightFading)
    {
        Utils::updateBacklightFade();
    }

    petWatchdog(); // Display::update()のmutex待ち後にもリセット
    Display::update();       // 画面表示は常に更新
    petWatchdog(); // 表示処理後にもリセット
    Menu::checkInactivity(); // 無操作タイムアウトを監視

    // --- 周期的に実行するタスク (2秒サイクル) ---
    handlePeriodicTasks();
    petWatchdog(); // 周期タスク後にもリセット

    // ★ Core1からのWoLトリガーを処理 (非同期)
    if (wolTriggerAction > 0)
    {
        int triggerType = wolTriggerAction;
        wolTriggerAction = 0; // フラグをクリア

        if (DEBUG)
            Serial.printf("[Core0] Received WoL trigger: Type %d\n", triggerType);

        const char *targetName = (triggerType == 1) ? "Desktop PC" : "Server PC";

        // ★ LCDの2行目にメッセージを上書き表示 (タイマー式)
        // v19.5.0: try_enterでデッドロック回避
        if (mutex_try_enter(&data_mutex, nullptr)) {
            snprintf(State::timers.timedOverlayLine1, LCD_COLS + 1, "WoL Sent: %s", targetName);
            State::timers.timedOverlayEndTime = millis() + 2000;
            State::system.forceMainScreenRedraw = true; // メイン画面を再描画してオーバーレイを表示
            mutex_exit(&data_mutex);
        }

        // ★ WoLパケット送信とLED点滅をCore1に依頼
        if (triggerType == 1)
        {
            requestSendWolDesktop = true;
        }
        else
        {
            requestSendWolServer = true;
        }
    }

    if (State::system.illuminationOn)
    {
        Utils::handleSmoothIllumination();
    }

    // Core1からのULTRASONIC_MONITOR終了指示を処理 (子機が自ら終了したとき)
    if (requestExitUltrasonic)
    {
        requestExitUltrasonic = false;
        requestGetUltrasonicDistance = false; // 残留ポーリングも同時キャンセル
        // v19.5.0: try_enterでデッドロック回避
        if (mutex_try_enter(&data_mutex, nullptr)) {
            if (State::menu.currentMode == State::ULTRASONIC_MONITOR) {
                Menu::changeMode(State::MAIN_DISPLAY);
            }
            mutex_exit(&data_mutex);
        }
    }

    // ★ Core1からのAPI経由IR送信リクエストを処理
    if (irSendRequested)
    {
        // フラグとコマンドデータを安全にコピー
        char device[32] = {0};
        char cmd[32] = {0};
        mutex_enter_blocking(&data_mutex);
        strncpy(device, (const char*)irSendDevice, sizeof(device));
        strncpy(cmd, (const char*)irSendCommand, sizeof(cmd));
        irSendRequested = false;
        mutex_exit(&data_mutex);

        if (DEBUG)
            Serial.printf("[Core0] IR API dispatch: device=%s, cmd=%s\n",
                device, cmd);

        // デバイス名とコマンド名から信号を検索して送信
        const IrSignal* signal = nullptr;
        String dev(device);
        String c(cmd);

        if (dev == "LIGHT")
        {
            if      (c == "POWER")      signal = &signal_LIGHT_POWER;
            else if (c == "BRIGHT_UP")  signal = &signal_LIGHT_BRIGHT_UP;
            else if (c == "BRIGHT_DOWN") signal = &signal_LIGHT_BRIGHT_DOWN;
            else if (c == "COLOR_COOL") signal = &signal_LIGHT_COLOR_COOL;
            else if (c == "COLOR_WARM") signal = &signal_LIGHT_COLOR_WARM;
            else if (c == "SLEEP_TIMER") signal = &signal_LIGHT_SLEEP_TIMER;
            else if (c == "FULL_LIGHT") signal = &signal_LIGHT_FULL_LIGHT;
        }
        else if (dev == "TV")
        {
            if      (c == "POWER")       signal = &signal_TV_POWER;
            else if (c == "INPUT")       signal = &signal_TV_INPUT;
            else if (c == "VOL_UP")      signal = &signal_TV_VOL_UP;
            else if (c == "VOL_DOWN")    signal = &signal_TV_VOL_DOWN;
            else if (c == "CH_UP")       signal = &signal_TV_CH_UP;
            else if (c == "CH_DOWN")     signal = &signal_TV_CH_DOWN;
            else if (c == "MUTE")        signal = &signal_TV_MUTE;
            else if (c == "TERRESTRIAL") signal = &signal_TV_TERRESTRIAL;
            else if (c == "BS_CS")       signal = &signal_TV_BS_CS;
            else if (c == "UP")          signal = &signal_TV_UP;
            else if (c == "DOWN")        signal = &signal_TV_DOWN;
            else if (c == "LEFT")        signal = &signal_TV_LEFT;
            else if (c == "RIGHT")       signal = &signal_TV_RIGHT;
            else if (c == "OK")          signal = &signal_TV_OK;
            else if (c == "MENU")        signal = &signal_TV_MENU;
            else if (c == "HOME")        signal = &signal_TV_HOME;
            else if (c == "RETURN")      signal = &signal_TV_RETURN;
            else if (c == "GUIDE")       signal = &signal_TV_GUIDE;
            else if (c == "DDATA")       signal = &signal_TV_DDATA;
            else if (c == "SUBMENU")     signal = &signal_TV_SUBMENU;
            else if (c == "REC_LIST")    signal = &signal_TV_REC_LIST;
            else if (c == "DISPLAY")     signal = &signal_TV_DISPLAY;
            else if (c == "CC")          signal = &signal_TV_CC;
            else if (c == "AUDIO")       signal = &signal_TV_AUDIO;
            else if (c == "BLUE")        signal = &signal_TV_BLUE;
            else if (c == "RED")         signal = &signal_TV_RED;
            else if (c == "GREEN")       signal = &signal_TV_GREEN;
            else if (c == "YELLOW")      signal = &signal_TV_YELLOW;
            else if (c == "REWIND")      signal = &signal_TV_REWIND;
            else if (c == "PLAY")        signal = &signal_TV_PLAY;
            else if (c == "PAUSE")       signal = &signal_TV_PAUSE;
            else if (c == "FFWD")        signal = &signal_TV_FFWD;
            else if (c == "PREV")        signal = &signal_TV_PREV;
            else if (c == "REC")         signal = &signal_TV_REC;
            else if (c == "STOP")        signal = &signal_TV_STOP;
            else if (c == "NEXT")        signal = &signal_TV_NEXT;
            else if (c == "3DIGIT")      signal = &signal_TV_3DIGIT;
            else if (c == "NETFLIX")     signal = &signal_TV_NETFLIX;
            else if (c == "HULU")        signal = &signal_TV_HULU;
            else if (c == "UNEXT")       signal = &signal_TV_UNEXT;
            else if (c == "ABEMATV")     signal = &signal_TV_ABEMATV;
            else if (c == "YOUTUBE")     signal = &signal_TV_YOUTUBE;
            else if (c == "TVER")        signal = &signal_TV_TVER;
            // 数字チャンネル
            else if (c == "1")           signal = &signal_TV_1;
            else if (c == "2")           signal = &signal_TV_2;
            else if (c == "3")           signal = &signal_TV_3;
            else if (c == "4")           signal = &signal_TV_4;
            else if (c == "5")           signal = &signal_TV_5;
            else if (c == "6")           signal = &signal_TV_6;
            else if (c == "7")           signal = &signal_TV_7;
            else if (c == "8")           signal = &signal_TV_8;
            else if (c == "9")           signal = &signal_TV_9;
            else if (c == "10")          signal = &signal_TV_10;
            else if (c == "11")          signal = &signal_TV_11;
            else if (c == "12")          signal = &signal_TV_12;
        }
        else if (dev == "AIRCON")
        {
            if      (c == "POWER")          signal = &signal_AIRCON_POWER;
            else if (c == "COOL_ON")        signal = &signal_AIRCON_COOL_ON;
            else if (c == "HEAT_ON")        signal = &signal_AIRCON_HEAT_ON;
            else if (c == "OFF")            signal = &signal_AIRCON_OFF;
            else if (c == "TEMP_UP")        signal = &signal_AIRCON_TEMP_UP;
            else if (c == "TEMP_DOWN")      signal = &signal_AIRCON_TEMP_DOWN;
            else if (c == "MODE")           signal = &signal_AIRCON_MODE;
            else if (c == "FAN")            signal = &signal_AIRCON_FAN;
            else if (c == "SWING")          signal = &signal_AIRCON_SWING;
            else if (c == "ECO")            signal = &signal_AIRCON_ECO;
            else if (c == "FAN_AWAY")       signal = &signal_AIRCON_FAN_AWAY;
            else if (c == "SLEEP")          signal = &signal_AIRCON_SLEEP;
            else if (c == "STREAMER_CLEAN") signal = &signal_AIRCON_STREAMER_CLEAN;
            else if (c == "TIMER_OFF")      signal = &signal_AIRCON_TIMER_OFF;
            else if (c == "TIMER_ON")       signal = &signal_AIRCON_TIMER_ON;
            else if (c == "CANCEL")         signal = &signal_AIRCON_CANCEL;
        }
        else if (dev == "FAN")
        {
            if      (c == "POWER")      signal = &signal_FAN_POWER;
            else if (c == "SPEED_UP")   signal = &signal_FAN_SPEED_UP;
            else if (c == "SPEED_DOWN") signal = &signal_FAN_SPEED_DOWN;
            else if (c == "SWING")      signal = &signal_FAN_SWING;
            else if (c == "TIMER")      signal = &signal_FAN_TIMER;
        }
        else if (dev == "SPEAKER")
        {
            if      (c == "POWER")       signal = &signal_SPEAKER_POWER;
            else if (c == "VOL_UP")      signal = &signal_SPEAKER_VOL_UP;
            else if (c == "VOL_DOWN")    signal = &signal_SPEAKER_VOL_DOWN;
            else if (c == "MUTE")        signal = &signal_SPEAKER_MUTE;
            else if (c == "PLAY_PAUSE")  signal = &signal_SPEAKER_PLAY_PAUSE;
            else if (c == "STOP")        signal = &signal_SPEAKER_STOP;
            else if (c == "NEXT")        signal = &signal_SPEAKER_NEXT;
            else if (c == "PREV")        signal = &signal_SPEAKER_PREV;
            else if (c == "FFWD")        signal = &signal_SPEAKER_FFWD;
            else if (c == "REWIND")      signal = &signal_SPEAKER_REWIND;
            else if (c == "FOLDER_NEXT") signal = &signal_SPEAKER_FOLDER_NEXT;
            else if (c == "FOLDER_PREV") signal = &signal_SPEAKER_FOLDER_PREV;
            else if (c == "SPEED_UP")    signal = &signal_SPEAKER_SPEED_UP;
            else if (c == "SPEED_DOWN")  signal = &signal_SPEAKER_SPEED_DOWN;
            else if (c == "SOURCE_SD")   signal = &signal_SPEAKER_SOURCE_SD;
            else if (c == "SOURCE_FM")   signal = &signal_SPEAKER_SOURCE_FM;
            else if (c == "SOURCE_LINE") signal = &signal_SPEAKER_SOURCE_LINE;
            else if (c == "SOUND")       signal = &signal_SPEAKER_SOUND;
            else if (c == "BT_RX")       signal = &signal_SPEAKER_BT_RX;
            else if (c == "BT_TX")       signal = &signal_SPEAKER_BT_TX;
            else if (c == "DISPLAY")     signal = &signal_SPEAKER_DISPLAY;
            else if (c == "DIMMER")      signal = &signal_SPEAKER_DIMMER;
        }

        if (signal != nullptr)
        {
            IRRemote::sendIrSignal(signal);
            if (DEBUG)
                Serial.printf("[Core0] IR sent: %s/%s\n", device, cmd);
        }
        else
        {
            if (DEBUG)
                Serial.printf("[Core0] IR unknown: %s/%s\n", device, cmd);
        }
    }

    petWatchdog();
    delay(10); // Core0のループ負荷を軽減
}
