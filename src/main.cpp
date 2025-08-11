#include <Arduino.h>
#include <WiFi.h>                         // Biblioteca Wi-Fi do ESP32
#include <esp_camera.h>                   // Biblioteca da câmera ESP32-CAM
#include <micro_ros_platformio.h>         // Micro-ROS (PlatformIO)
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/compressed_image.h>

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
// #define CAMERA_MODEL_ESP_EYE  // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_CAMS3_UNIT  // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"


// declaramos o agent_ip sem inicializar
static IPAddress   agent_ip;

rcl_publisher_t image_pub;
sensor_msgs__msg__CompressedImage img_msg;  // mensagem de imagem comprimida (JPEG)

// -------------------------------
// Máquina de estados da conexão
// -------------------------------
enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state = WAITING_AGENT; // inicia esperando o agente

// Objetos do RCL que precisamos recriar/destruir quando reconectar
static rcl_allocator_t allocator;
static rclc_support_t  support;
static rcl_node_t      node;

// Marca que já preparamos os campos estáticos de img_msg
static bool img_msg_static_ready = false;

// Controle simples para rearmar transporte quando voltar a esperar agente
static bool need_transport_reset = true;

// Prototipos
static void prepare_img_msg_static_fields();
static bool create_entities();
static void destroy_entities();

void setup() {
  // Converte a string macro em IPAddress
  if (! agent_ip.fromString(AGENT_IP) ) {
    Serial.println("Falha ao converter DAGENT_IP para IPAddress");
  } else {
    Serial.print("Agente ROS em: ");
    Serial.print(agent_ip);
    Serial.print(":");
    Serial.println(AGENT_PORT);
  }

  Serial.begin(115200);
  delay(1000);  // Pequena pausa para inicializar

  // **Inicialização da câmera ESP32-CAM**
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  // Configura formato e resolução da imagem
  config.xclk_freq_hz = 20000000;          // Frequência do clock da câmera (20MHz)
  config.pixel_format = PIXFORMAT_JPEG;    // Formato de captura JPEG (comprimido)
  config.frame_size   = FRAMESIZE_QVGA;    // Tamanho do frame: QVGA (320x240) (ajuste conforme necessário)
  config.jpeg_quality = 12;                // Qualidade JPEG (0-63, menor = melhor qualidade e imagem maior)
  config.fb_count     = 1;                 // Número de framebuffers (1 é suficiente para transmissão simples)

  esp_err_t cam_err = esp_camera_init(&config);
  if (cam_err != ESP_OK) {
    Serial.printf("Falha na inicialização da câmera: 0x%x\n", cam_err);
  } else {
    Serial.println("Câmera inicializada com sucesso.");
  }

  // **Conectar ao Wi-Fi**
  Serial.printf("Conectando ao Wi-Fi SSID: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");  // aguarda conexão
  }
  Serial.println("\nWi-Fi conectado.");
  Serial.print("Endereço IP do ESP32: ");
  Serial.println(WiFi.localIP());

  // Prepara a mensagem CompressedImage (campos estáticos) apenas uma vez
  prepare_img_msg_static_fields();

  Serial.println("Setup concluído, máquina de estados iniciada (WAITING_AGENT)...");
}

void loop() {
  switch (state) {
    case WAITING_AGENT: {
      // (Re)arma o transporte apenas uma vez quando entramos nesse estado
      if (need_transport_reset) {
        // **Inicializar transporte Micro-ROS via Wi-Fi**
        set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PSK, agent_ip, AGENT_PORT);
        need_transport_reset = false;
      }

      // Tenta pingar o agente rapidamente, sem bloquear muito
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        Serial.println("Agente Micro-ROS alcançável via Wi-Fi.");
        state = AGENT_AVAILABLE;
      } else {
        // ainda aguardando
        delay(150);
      }
    } break;

    case AGENT_AVAILABLE: {
      // **Inicialização do nó e publisher ROS 2**
      if (create_entities()) {
        Serial.println("Entidades ROS2 criadas. Estado: AGENT_CONNECTED.");
        state = AGENT_CONNECTED;
      } else {
        Serial.println("Falha ao criar entidades. Voltando para WAITING_AGENT...");
        state = WAITING_AGENT;
        // na próxima volta, rearmamos transporte
        need_transport_reset = true;
        delay(300);
      }
    } break;

    case AGENT_CONNECTED: {
      // Se Wi-Fi caiu, tratamos como desconectado
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Wi-Fi desconectado. Estado: AGENT_DISCONNECTED.");
        state = AGENT_DISCONNECTED;
        break;
      }

      // Captura uma frame da câmera
      camera_fb_t *fb = esp_camera_fb_get();  // obtém frame buffer com imagem JPEG
      if (!fb) {
        Serial.println("Falha ao capturar imagem da câmera");
      } else {
        // Redimensiona/aloja buffer de dados da mensagem se necessário
        size_t image_len = fb->len;
        if (img_msg.data.capacity < image_len) {
          // Realoca buffer de dados para comportar a imagem atual
          if (img_msg.data.data != NULL) {
            free(img_msg.data.data);
          }
          img_msg.data.data = (uint8_t*) malloc(image_len);
          img_msg.data.capacity = image_len;
        }
        // Copia os bytes da imagem JPEG para a mensagem ROS
        memcpy(img_msg.data.data, fb->buf, image_len);
        img_msg.data.size = image_len;
        // Preenche timestamp do header com tempo atual (millis)
        img_msg.header.stamp.sec = millis() / 1000;
        img_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

        // Publica a mensagem no tópico ROS2
        rcl_ret_t rc = rcl_publish(&image_pub, &img_msg, NULL);
        Serial.printf("Imagem publicada no tópico ROS (tamanho %u bytes)\n", (unsigned)image_len);

        // Se o publish falhar, consideramos desconectado
        if (rc != RCL_RET_OK) {
          Serial.printf("Erro no publish (rc=%d). Estado: AGENT_DISCONNECTED.\n", (int)rc);
          state = AGENT_DISCONNECTED;
        } else {
          // Checagem leve de presença do agente para detectar queda
          if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) {
            Serial.println("Perda do agente detectada via ping. Estado: AGENT_DISCONNECTED.");
            state = AGENT_DISCONNECTED;
          }
        }

        // Libera o frame buffer da câmera
        esp_camera_fb_return(fb);
      }

      // Intervalo entre capturas (ajuste conforme necessário para controlar frequência)
      delay(1000);  // aqui definido ~1 Hz (1 imagem por segundo)
    } break;

    case AGENT_DISCONNECTED: {
      // Destrói entidades e volta a procurar o agente
      destroy_entities();
      state = WAITING_AGENT;
      need_transport_reset = true; // rearmar transporte
      // pequeno atraso para não ficar rodando 100% CPU
      delay(200);
    } break;
  }
}

// -------------------------------
// Implementações auxiliares
// -------------------------------

// Prepara a mensagem CompressedImage (campos estáticos)
static void prepare_img_msg_static_fields() {
  if (img_msg_static_ready) return;

  // Inicializa estrutura da mensagem
  sensor_msgs__msg__CompressedImage__init(&img_msg);

  // Preenche o campo header.frame_id (ID de referência da câmera)
  // Importante: apontamos para literal e NÃO chamamos __fini para esse campo
  const char* frameID = "esp32_camera";
  img_msg.header.frame_id.data = const_cast<char*>(frameID);
  img_msg.header.frame_id.size = strlen(frameID);
  img_msg.header.frame_id.capacity = img_msg.header.frame_id.size + 1;

  // Define o formato da imagem como "jpeg"
  const char* fmt = "jpeg";
  img_msg.format.data = const_cast<char*>(fmt);
  img_msg.format.size = strlen(fmt);
  img_msg.format.capacity = img_msg.format.size + 1;

  // Buffer dinâmico será alocado sob demanda no loop()
  img_msg.data.data = NULL;
  img_msg.data.size = 0;
  img_msg.data.capacity = 0;

  img_msg_static_ready = true;
}

// Cria suporte, nó e publisher
static bool create_entities() {
  allocator = rcl_get_default_allocator();

  rcl_ret_t rc;
  // Inicializa suporte RCL (tempo de execução do cliente ROS 2)
  rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) {
    Serial.printf("rclc_support_init falhou (rc=%d)\n", (int)rc);
    return false;
  }

  // Cria um nó ROS 2 chamado "esp32cam_node"
  rc = rclc_node_init_default(&node, "esp32cam_node", "", &support);
  if (rc != RCL_RET_OK) {
    Serial.printf("rclc_node_init_default falhou (rc=%d)\n", (int)rc);
    // encerra o support/context antes de sair
    rcl_shutdown(&support.context);
    rclc_support_fini(&support);
    return false;
  }

  // Inicializa um publisher que publicará no tópico "image_raw/compressed" do tipo sensor_msgs/CompressedImage
  rc = rclc_publisher_init_default(
    &image_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
    "image_raw/compressed"
  );
  if (rc != RCL_RET_OK) {
    Serial.printf("rclc_publisher_init_default falhou (rc=%d)\n", (int)rc);
    // Tenta limpar o nó e o support criados, evita vazamento
    rcl_node_fini(&node);
    rcl_shutdown(&support.context);
    rclc_support_fini(&support);
    return false;
  }

  return true;
}

// Destrói publisher e nó; libera nosso buffer dinâmico; finaliza support/context
static void destroy_entities() {
  // Finaliza publisher (se o nó existir)
  if (image_pub.impl != NULL) {
    (void) rcl_publisher_fini(&image_pub, &node);
    // zera o handle para evitar uso após finalização
    image_pub = rcl_publisher_t();
  }
  // Finaliza nó
  if (node.impl != NULL) {
    (void) rcl_node_fini(&node);
    node = rcl_node_t();
  }

  // Finaliza o context/support do RCL (essencial para permitir novo init)
  rcl_shutdown(&support.context);
  rclc_support_fini(&support);
  support = rclc_support_t(); // limpa struct

  // NÃO chamamos sensor_msgs__msg__CompressedImage__fini(&img_msg)
  // porque os campos de string apontam para literais. Apenas liberamos
  // o buffer de dados que nós mesmos alocamos.
  if (img_msg.data.data != NULL) {
    free(img_msg.data.data);
    img_msg.data.data = NULL;
    img_msg.data.size = 0;
    img_msg.data.capacity = 0;
  }
}
