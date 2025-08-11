#include <Arduino.h>
#include <IPAddress.h>

#include "app_config.h"
#include "state.h"
#include "camera.h"
#include "net.h"
#include "ros_image_pub.h"

static IPAddress agent_ip;
static AppState state = AppState::WAITING_AGENT;
static bool need_transport_reset = true;

static RosImagePublisher image_pub;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!agent_ip.fromString(AGENT_IP)) {
    Serial.println("[app] invalid AGENT_IP string");
  } else {
    Serial.printf("[app] agent: %s:%u\n", agent_ip.toString().c_str(), (unsigned)AGENT_PORT);
  }

  // Câmera
  if (!camera_begin()) {
    Serial.println("[app] camera init failed");
  }

  // Wi-Fi
  wifi_connect(WIFI_SSID, WIFI_PSK);

  Serial.println("[app] setup done, entering state machine...");
}

void loop() {
  switch (state) {
    case AppState::WAITING_AGENT: {
      if (need_transport_reset) {
        microros_setup_transport(agent_ip, AGENT_PORT);
        need_transport_reset = false;
      }
      if (microros_ping_ok(100, 1)) {
        Serial.println("[sm] agent reachable");
        state = AppState::AGENT_AVAILABLE;
      } else {
        delay(150);
      }
    } break;

    case AppState::AGENT_AVAILABLE: {
      if (image_pub.init(ROS_NODE_NAME, ROS_TOPIC)) {
        Serial.println("[sm] entities created -> CONNECTED");
        state = AppState::AGENT_CONNECTED;
      } else {
        Serial.println("[sm] create entities failed -> WAITING_AGENT");
        state = AppState::WAITING_AGENT;
        need_transport_reset = true;
        delay(300);
      }
    } break;

    case AppState::AGENT_CONNECTED: {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[sm] Wi-Fi lost -> DISCONNECTED");
        state = AppState::AGENT_DISCONNECTED;
        break;
      }

      camera_fb_t* fb = camera_capture();
      if (!fb) {
        Serial.println("[app] camera capture failed");
      } else {
        bool ok = image_pub.publish(fb);
        Serial.printf("[ros] published image (%u bytes) %s\n",
                      (unsigned)fb->len, ok ? "OK" : "ERR");
        if (!ok || !microros_ping_ok(50, 1)) {
          Serial.println("[sm] agent lost -> DISCONNECTED");
          state = AppState::AGENT_DISCONNECTED;
        }
        camera_release(fb);
      }

      delay(CAPTURE_PERIOD_MS);
    } break;

    case AppState::AGENT_DISCONNECTED: {
      image_pub.fini();
      state = AppState::WAITING_AGENT;
      need_transport_reset = true;
      delay(200);
    } break;
  }
}
