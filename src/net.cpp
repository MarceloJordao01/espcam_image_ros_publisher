#include "net.h"
#include "app_config.h"
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

void wifi_connect(const char* ssid, const char* psk) {
  Serial.printf("[wifi] connecting to SSID: %s\n", ssid);
  WiFi.begin(ssid, psk);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[wifi] connected, IP: %s\n", WiFi.localIP().toString().c_str());
}

void microros_setup_transport(const IPAddress& agent_ip, uint16_t agent_port) {
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PSK, agent_ip, agent_port);
}

bool microros_ping_ok(int timeout_ms, int attempts) {
  return rmw_uros_ping_agent(timeout_ms, attempts) == RMW_RET_OK;
}
