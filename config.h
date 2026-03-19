#ifndef CONFIG_H
#define CONFIG_H

// --- IRremote警告抑制 ---
// RP2040でSEND_PWM_BY_TIMERがデフォルト有効の警告を抑制
#define SUPPRESS_SEND_PWM_BY_TIMER_INFO
// LED_BUILTINを無効化してフィードバックLED警告を抑制
#define NO_LED_FEEDBACK_CODE

//================================================================
// ★★★ 設定ファイル (このファイルに機密情報を入力してください) ★★★
//================================================================

// --- デバッグ設定 ---
// trueにすると、動作状況がシリアルモニタに詳細に出力され、デバッグ用メニューが表示されます。
// falseにすると、出力とメニューが抑制されます。
constexpr bool DEBUG = false;

// --- 赤外線リモコン設定 ---
const int IR_SEND_PIN = 19;    // 赤外線送信ピン (GP19)
const int IR_RECEIVE_PIN = 18; // 赤外線受信ピン (GP18)
const int MAX_RAW_DATA_LEN = 300;  // Raw信号の最大長
const int MICROS_PER_TICK = 50;    // IRremoteのティック単位（マイクロ秒）

// --- WiFi設定 (優先順位順に3つまで設定) ---
struct WiFiCredential
{
  const char *ssid;
  const char *password;
};

const WiFiCredential wifiCredentials[] = {
    {"YOUR_SSID_1", "YOUR_PASSWORD_1"}, // 優先度1
    {"YOUR_SSID_2", "YOUR_PASSWORD_2"}, // 優先度2
    {"YOUR_SSID_3", "YOUR_PASSWORD_3"}  // 優先度3
};
const int numWifiCredentials = sizeof(wifiCredentials) / sizeof(wifiCredentials[0]);

const char *NTP_SERVER = "ntp.nict.jp";

// --- PCのMACアドレス (Wake on LAN用) ---
// ★★★ MACアドレスを "00:1A:2B:3C:4D:5E" の形式で直接入力してください ★★★
const char *MAC_DESKTOP = "00:00:00:00:00:01";
const char *MAC_SERVER = "00:00:00:00:00:02";

// --- 静的IPアドレス設定 (固定IPを使用する場合) ---
// v5.2.0: IPアドレスの値をここに設定
// trueにするとIPアドレスを固定します。falseにするとDHCPから自動取得します。
const bool USE_STATIC_IP = true;                    // ★★★ 静的IPを使う場合は true に変更 ★★★
const byte STATIC_IP_BYTES[] = {192, 168, 10, 210}; // 固定IP
const byte GATEWAY_BYTES[] = {192, 168, 10, 1};     // ゲートウェイ (ルーター)
const byte SUBNET_BYTES[] = {255, 255, 255, 0};     // サブネットマスク
const byte PRIMARY_DNS_BYTES[] = {8, 8, 8, 8};      // DNS (例: Google)
const byte SECONDARY_DNS_BYTES[] = {8, 8, 4, 4};    // (Pico W mbedコアでは現在使用されません)

// --- Google Apps Script ---
const char *GAS_URL = "YOUR_GAS_URL_HERE";
const char *GAS_URL_WOL = "YOUR_GAS_URL_WOL_HERE"; // LINE経由のWoL指示を受け取るGASのURL

// --- SwitchBot API設定 ---
const char *SWITCHBOT_TOKEN = "YOUR_SWITCHBOT_TOKEN_HERE";
const char *SWITCHBOT_SECRET = "YOUR_SWITCHBOT_SECRET_HERE";
const char *DEVICE_ID_LIGHT = "DEVICE_ID_LIGHT_HERE";
const char *DEVICE_ID_TV = "DEVICE_ID_TV_HERE";
const char *DEVICE_ID_AC = "DEVICE_ID_AC_HERE";
const char *DEVICE_ID_FAN = "DEVICE_ID_FAN_HERE";
const char *DEVICE_ID_SPEAKER = "DEVICE_ID_SPEAKER_HERE";
const char *DEVICE_ID_OTHERS = "DEVICE_ID_OTHERS_HERE";

// --- LINE Messaging API 設定 ---
const char *LINE_CHANNEL_ACCESS_TOKEN = "YOUR_LINE_CHANNEL_ACCESS_TOKEN_HERE";
const char *LINE_USER_ID = "YOUR_LINE_USER_ID_HERE";

// --- 子機IPアドレス ---
const char *CHILD_IP   = "192.168.10.215"; // ultrasonic_sensor固定IP
const int   CHILD_PORT = 80;

#endif // CONFIG_H
