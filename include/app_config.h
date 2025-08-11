#pragma once
#include <Arduino.h>

// ======== Wi-Fi ========
#define WIFI_SSID  "<SSID>"
#define WIFI_PSK   "<PSK>"

// ======== Agente micro-ROS ========
// Use string "x.x.x.x" (será convertida para IPAddress no main)
#define AGENT_IP   "<AGENT_IP>"
#define AGENT_PORT 8888

// ======== Câmera ========
//
// CAM_XCLK_FREQ: frequência do clock externo (XCLK) que o ESP32 envia ao sensor.
// - Valores comuns: 10 MHz ou 20 MHz (OV2640 funciona bem em 20 MHz).
// - Reduzir pode melhorar estabilidade elétrica em cabos ruins, mas aumenta latência.
// - Alterar isso em runtime normalmente exige reinicializar a câmera.
//
#define CAM_XCLK_FREQ 20000000

// CAM_FRAME_SIZE: resolução do frame antes da compressão JPEG.
// - OV2640: QQVGA(160x120), QVGA(320x240), CIF(352x288), VGA(640x480),
//           SVGA(800x600), XGA(1024x768), SXGA(1280x1024), UXGA(1600x1200), etc.
// - Resoluções maiores exigem PSRAM e aumentam latência e banda no micro-ROS/XRCE.
// - Pode ser ajustado em runtime: sensor_t* s=esp_camera_sensor_get(); s->set_framesize(s, FRAMESIZE_VGA);
//   (não precisa reinicializar a câmera).
//
#define CAM_FRAME_SIZE FRAMESIZE_QVGA // 320x240

// CAM_JPEG_QUALITY: 0–63 (quanto MENOR o número, melhor a qualidade e MAIOR o tamanho!)
// - Faixa usual: 10–20. 12 costuma equilibrar nitidez e tamanho p/ Wi-Fi.
// - Pode ser ajustado em runtime: sensor_t* s=esp_camera_sensor_get(); s->set_quality(s, 14);
//   (sem reinicializar a câmera).
//
#define CAM_JPEG_QUALITY 10

// CAM_FB_COUNT: quantidade de framebuffers internos.
// - 1: menor uso de RAM; 2: “double buffering” (maior throughput), consome mais RAM/PSRAM.
// - Em resoluções médias/altas geralmente requer PSRAM.
// - Alterar normalmente implica reinicializar a câmera.
//
#define CAM_FB_COUNT 1

// ======== ROS ========
#define ROS_NODE_NAME   "esp32cam_node"
#define ROS_TOPIC       "image_raw/compressed"
#define ROS_FRAME_ID    "esp32_camera"
#define ROS_FORMAT      "jpeg"

// ======== Loop ========

// CAPTURE_PERIOD_MS define o intervalo entre capturas/publicações (ex.: 1000 => ~1 Hz).
// Em runtime, a taxa pode ser trocada se você usar uma variável em vez de #define
//
#define CAPTURE_PERIOD_MS 500  // ~1 Hz
