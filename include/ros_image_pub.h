#pragma once
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <esp_camera.h>

class RosImagePublisher {
public:
  RosImagePublisher();
  ~RosImagePublisher();

  bool init(const char* node_name, const char* topic);
  void fini();
  bool is_ready() const;

  // Copia bytes JPEG do frame e publica; se publish falha, retorna false
  bool publish(camera_fb_t* fb);

private:
  void prepare_static_fields();
  bool create_entities(const char* node_name, const char* topic);
  void destroy_entities();

  rcl_allocator_t allocator_;
  rclc_support_t  support_;
  rcl_node_t      node_;
  rcl_publisher_t pub_;
  sensor_msgs__msg__CompressedImage msg_;
  bool msg_static_ready_;
};
