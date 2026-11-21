idf_component_register(
  SRCS "app_main.cpp" "node.cpp" "gateway.cpp" "imu.cpp"
  INCLUDE_DIRS "."
  REQUIRES driver esp_wifi esp_event esp_netif nvs_flash esp_timer
)
