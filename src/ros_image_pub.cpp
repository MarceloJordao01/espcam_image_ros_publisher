#include "ros_image_pub.h"
#include "app_config.h"
#include <cstring>

RosImagePublisher::RosImagePublisher()
: allocator_(), support_(), node_(), pub_(), msg_(), msg_static_ready_(false) {}

RosImagePublisher::~RosImagePublisher() { fini(); }

bool RosImagePublisher::init(const char* node_name, const char* topic) {
  prepare_static_fields();
  return create_entities(node_name, topic);
}

void RosImagePublisher::fini() {
  destroy_entities();
  // Libera nosso buffer dinâmico (strings apontam para literais)
  if (msg_.data.data) {
    free(msg_.data.data);
    msg_.data.data = nullptr;
    msg_.data.size = 0;
    msg_.data.capacity = 0;
  }
}

bool RosImagePublisher::is_ready() const {
  return pub_.impl != nullptr && node_.impl != nullptr;
}

bool RosImagePublisher::publish(camera_fb_t* fb) {
  if (!is_ready() || !fb) return false;

  size_t image_len = fb->len;
  if (msg_.data.capacity < image_len) {
    if (msg_.data.data) free(msg_.data.data);
    msg_.data.data = (uint8_t*) malloc(image_len);
    msg_.data.capacity = image_len;
  }
  memcpy(msg_.data.data, fb->buf, image_len);
  msg_.data.size = image_len;

  // timestamp (millis)
  uint32_t ms = millis();
  msg_.header.stamp.sec = ms / 1000;
  msg_.header.stamp.nanosec = (ms % 1000) * 1000000;

  rcl_ret_t rc = rcl_publish(&pub_, &msg_, nullptr);
  return rc == RCL_RET_OK;
}

void RosImagePublisher::prepare_static_fields() {
  if (msg_static_ready_) return;
  sensor_msgs__msg__CompressedImage__init(&msg_);

  // frame_id
  const char* frame_id = ROS_FRAME_ID;
  msg_.header.frame_id.data = const_cast<char*>(frame_id);
  msg_.header.frame_id.size = strlen(frame_id);
  msg_.header.frame_id.capacity = msg_.header.frame_id.size + 1;

  // format "jpeg"
  const char* fmt = ROS_FORMAT;
  msg_.format.data = const_cast<char*>(fmt);
  msg_.format.size = strlen(fmt);
  msg_.format.capacity = msg_.format.size + 1;

  msg_.data.data = nullptr;
  msg_.data.size = 0;
  msg_.data.capacity = 0;

  msg_static_ready_ = true;
}

bool RosImagePublisher::create_entities(const char* node_name, const char* topic) {
  allocator_ = rcl_get_default_allocator();

  rcl_ret_t rc = rclc_support_init(&support_, 0, NULL, &allocator_);
  if (rc != RCL_RET_OK) {
    Serial.printf("[ros] support init failed: %d\n", (int)rc);
    return false;
  }

  rc = rclc_node_init_default(&node_, node_name, "", &support_);
  if (rc != RCL_RET_OK) {
    Serial.printf("[ros] node init failed: %d\n", (int)rc);
    rcl_ret_t rc = rcl_shutdown(&support_.context);
    (void)rc;
    rclc_support_fini(&support_);
    return false;
  }

  rc = rclc_publisher_init_default(
    &pub_, &node_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
    topic
  );
  if (rc != RCL_RET_OK) {
    Serial.printf("[ros] publisher init failed: %d\n", (int)rc);
    rcl_ret_t rc2 = rcl_node_fini(&node_);
    (void)rc2;
    rcl_ret_t rc = rcl_shutdown(&support_.context);
    (void)rc;
    rclc_support_fini(&support_);
    return false;
  }

  return true;
}

void RosImagePublisher::destroy_entities() {
  if (pub_.impl) {    
    rcl_ret_t rc3 = rcl_publisher_fini(&pub_, &node_);
    (void)rc3;
    pub_ = rcl_publisher_t();
  }
  if (node_.impl) {
    
    node_ = rcl_node_t();
  }
  if (support_.context.impl) {
    rcl_ret_t rc = rcl_shutdown(&support_.context);
    (void)rc;
    rclc_support_fini(&support_);
    support_ = rclc_support_t();
  }
}
