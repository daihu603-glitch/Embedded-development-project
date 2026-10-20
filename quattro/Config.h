#ifndef CONFIG_H
#define CONFIG_H

#include <WiFi.h>  // 需要包含以使用IPAddress类型

// 模式选择，true 为 AP 模式，false 为 STA 模式(默认)
const bool USE_AP_MODE = true;

// STA 路由器模式：将 USE_AP_MODE 改为 false 后，修改下面的名称、密码和 IP。
// 路由器必须是 2.4 GHz；local_ip 与 gateway 的前三段必须和路由器一致。
const char *ssid = "IQOO Neo9 Pro";
const char *password = "12345678";
IPAddress local_ip(192, 168, 2, 192);  // 设置固定 IP 地址
IPAddress gateway(192, 168, 2, 1);     // 路由器网关地址
IPAddress subnet(255, 255, 255, 0);    // 子网掩码

// AP 热点模式：当前 USE_AP_MODE = true，ESP32 会创建下面名称和密码的热点。
// 要修改机器人 Wi-Fi 名称：改 ap_ssid；要修改密码：改 ap_password（至少 8 位）。
// 要修改 App 访问地址：改 ap_local_ip，并在手机 App 的“机器人 IP 地址”同步填入该地址。
const char *ap_ssid = "予你";
const char *ap_password = "45453968";
IPAddress ap_local_ip(192, 168, 55, 233);
IPAddress ap_gateway(192, 168, 55, 233);  // AP 本机作为热点网关
IPAddress ap_subnet(255, 255, 255, 0);

#endif
