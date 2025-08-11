#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <IPAddress.h>

void wifi_connect(const char* ssid, const char* psk);
void microros_setup_transport(const IPAddress& agent_ip, uint16_t agent_port);
bool microros_ping_ok(int timeout_ms, int attempts);
