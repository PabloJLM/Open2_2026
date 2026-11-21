#ifdef ROLE_GW

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "packet.h"
#include "config.h"

// Los marcos binarios salen por USB Serial JTAG con el driver, sin pasar por stdout.
// Los logs de IDF van a la consola por defecto, que no es este puerto.

static QueueHandle_t q_rx;
static volatile bool got_data = false;

// Corre en la tarea de WiFi. Solo encola.
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  if (len != (int)sizeof(imu_pkt_t)) return;
  if (data[0] != PKT_MAGIC) return;

  rx_item_t it;
  it.rx_us = (uint32_t)esp_timer_get_time();
  memcpy(&it.pkt, data, sizeof(imu_pkt_t));
  xQueueSend(q_rx, &it, 0);
  got_data = true;
}

void role_run(void)
{
  usb_serial_jtag_driver_config_t ucfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&ucfg));

  q_rx = xQueueCreate(64, sizeof(rx_item_t));
  ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  int64_t t_mac_us = 0;

  for (;;) {
    rx_item_t it;
    if (xQueueReceive(q_rx, &it, pdMS_TO_TICKS(10)) == pdTRUE) {
      uint8_t fr[3 + sizeof(rx_item_t) + 1];
      fr[0] = FR_SYNC0;
      fr[1] = FR_SYNC1;
      fr[2] = sizeof(rx_item_t);
      memcpy(&fr[3], &it, sizeof(rx_item_t));
      fr[3 + sizeof(rx_item_t)] = crc8((const uint8_t *)&it, sizeof(rx_item_t));
      usb_serial_jtag_write_bytes(fr, sizeof(fr), pdMS_TO_TICKS(20));
    }

    // Mientras no llegue nada, imprime la MAC para usar unicast si se quiere.
    if (!got_data && esp_timer_get_time() - t_mac_us > 2000000) {
      t_mac_us = esp_timer_get_time();
      char s[40];
      int n = snprintf(s, sizeof(s), "GW_MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      usb_serial_jtag_write_bytes(s, n, pdMS_TO_TICKS(20));
    }
  }
}

#endif  // ROLE_GW
